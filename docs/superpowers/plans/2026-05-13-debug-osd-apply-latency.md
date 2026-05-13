# Debug OSD per-apply-call latency stats — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in drone-side debug OSD that surfaces p50/p95/max latency, total call count, and cumulative error count for each of the six parameter-change call sites (FEC, depth, radio, TX power, encoder set, encoder IDR), gated by a new `osd_debug_latency` config flag.

**Architecture:** New `drone/src/dl_latency.[ch]` module owns six fixed-size (N=128) ring buffers plus cumulative counters in process-static state, exposing `begin/end` wrappers around each call site and a `render` function the OSD writer calls at its existing 1 Hz tick. Module always records; only rendering is gated by the flag. Includes a deliberate behavior change in `dl_backend_enc.c` so non-2xx HTTP responses propagate as errors instead of being swallowed.

**Tech Stack:** C11 (POSIX, no extra libs), pytest for e2e, the existing `DL_TEST` registry for C unit tests.

**Spec:** `docs/superpowers/specs/2026-05-13-debug-osd-apply-latency-design.md`.

---

## File Map

**New files:**
- `drone/src/dl_latency.h` — public API: enum + handle + 4 functions.
- `drone/src/dl_latency.c` — ring buffers, counters, percentile compute, render.
- `tests/drone/test_dl_latency.c` — C unit tests using `DL_TEST` + `dl_latency_record_raw` injection helper.

**Modified files:**
- `drone/src/dl_config.h` — add `bool osd_debug_latency` field next to other OSD knobs.
- `drone/src/dl_config.c` — default `false`, parser entry mirroring `osd_enable`.
- `conf/drone.conf.sample` — document the new flag.
- `drone/src/dl_osd.h` — no API change (struct is opaque); comment update is fine.
- `drone/src/dl_osd.c` — add `debug_latency_enabled` + `debug_block[512]` fields, render-and-write order in `flush()`, refresh hook in `dl_osd_write_status`.
- `drone/src/dl_applier.c` — call `dl_latency_init()` once at startup before backends open.
- `drone/src/dl_backend_tx.c` — wrap `send_and_recv` inside `send_fec`/`send_depth`/`send_radio`.
- `drone/src/dl_backend_radio.c` — wrap `posix_spawnp`→`waitpid` in `run_iw`.
- `drone/src/dl_backend_enc.c` — wrap `http_get` inside `apply_set` and `dl_backend_enc_request_idr` (non-throttle path only); flip the non-2xx-HTTP return from `0` to `-1` and update the surrounding comment.
- `drone/Makefile` — add `dl_latency.c` to both `APPLIER_SRCS` and `TEST_SRCS`; add `test_dl_latency.c` to `TEST_SRCS`; pass `-DDL_TESTING` to the test target.
- `tests/test_drone_e2e.py` — add 4 new tests; extend `make_encoder_server` to support a configurable HTTP status.

---

## Task 1: `dl_latency` skeleton + empty render + single-sample tests

Lay down the module with the public API, the static state, the no-op-when-empty render, and the simplest two tests (empty render, single sample with rc=0). Build hookup comes in Task 4 — for this task you'll temporarily compile the new test file via a one-shot manual command to drive TDD, then wire into the Makefile once everything's green.

**Files:**
- Create: `drone/src/dl_latency.h`
- Create: `drone/src/dl_latency.c`
- Create: `tests/drone/test_dl_latency.c`

- [ ] **Step 1: Write the header**

Create `drone/src/dl_latency.h`:

```c
/* dl_latency.h — per-apply-call latency stats for the debug OSD.
 *
 * Process-singleton ring buffers (N=128) per tracked call site, plus
 * cumulative call count, error count and sticky max. Single-threaded.
 * Always records; the OSD writer decides whether to render based on
 * cfg->osd_debug_latency.
 *
 * See docs/superpowers/specs/2026-05-13-debug-osd-apply-latency-design.md.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum {
    DL_LAT_FEC = 0,
    DL_LAT_DPTH,
    DL_LAT_RADIO,
    DL_LAT_TXPWR,
    DL_LAT_ENC,
    DL_LAT_IDR,
    DL_LAT__COUNT,
} dl_lat_call_t;

/* Opaque; carries the start timestamp and the call id so `end` can't
 * accidentally desync from `begin`. */
typedef struct {
    uint64_t      t_ns;
    dl_lat_call_t which;
} dl_lat_handle_t;

void dl_latency_init(void);

dl_lat_handle_t dl_latency_begin(dl_lat_call_t which);
void dl_latency_end(dl_lat_handle_t h, int rc);

/* Renders all six lines into `out`, including msposd `&L<row>&F30 `
 * prefixes and trailing newlines. Returns bytes written (excluding
 * trailing NUL). */
size_t dl_latency_render(char *out, size_t out_len);

#ifdef DL_TESTING
/* Test-only direct sample injection. Bypasses clock_gettime so unit
 * tests can drive the module deterministically. `ms_u64` is clamped
 * to UINT16_MAX before storage. */
void dl_latency_record_raw(dl_lat_call_t which, uint64_t ms_u64, int rc);
#endif
```

- [ ] **Step 2: Write the failing tests (empty + single sample)**

Create `tests/drone/test_dl_latency.c`:

```c
/* test_dl_latency.c — unit tests for dl_latency. */
#include "dl_latency.h"
#include "../test_main.h"

#include <string.h>

DL_TEST(latency_empty_renders_dashes) {
    dl_latency_init();
    char buf[1024];
    size_t n = dl_latency_render(buf, sizeof(buf));
    DL_ASSERT(n > 0);
    DL_ASSERT(n < sizeof(buf));
    /* Each line uses `--` for the three ms fields when filled==0. */
    DL_ASSERT(strstr(buf, "FEC  ") != NULL);
    DL_ASSERT(strstr(buf, "DPTH ") != NULL);
    DL_ASSERT(strstr(buf, "RADIO") != NULL);
    DL_ASSERT(strstr(buf, "TXPWR") != NULL);
    DL_ASSERT(strstr(buf, "ENC  ") != NULL);
    DL_ASSERT(strstr(buf, "IDR  ") != NULL);
    DL_ASSERT(strstr(buf, "p50= --") != NULL);
    DL_ASSERT(strstr(buf, "p95= --") != NULL);
    DL_ASSERT(strstr(buf, "mx= --") != NULL);
    DL_ASSERT(strstr(buf, "n=0") != NULL);
    DL_ASSERT(strstr(buf, "e=0") != NULL);
}

DL_TEST(latency_records_single_sample) {
    dl_latency_init();
    dl_latency_record_raw(DL_LAT_FEC, 7, 0);

    char buf[1024];
    dl_latency_render(buf, sizeof(buf));

    /* FEC line should now read p50=  7 p95=  7 mx=  7 n=1 e=0. */
    const char *line = strstr(buf, "FEC  ");
    DL_ASSERT(line != NULL);
    /* Search only within this line (until next newline). */
    const char *eol = strchr(line, '\n');
    DL_ASSERT(eol != NULL);
    char one_line[256];
    size_t llen = (size_t)(eol - line);
    DL_ASSERT(llen < sizeof(one_line));
    memcpy(one_line, line, llen);
    one_line[llen] = '\0';

    DL_ASSERT(strstr(one_line, "p50=  7") != NULL);
    DL_ASSERT(strstr(one_line, "p95=  7") != NULL);
    DL_ASSERT(strstr(one_line, "mx=  7") != NULL);
    DL_ASSERT(strstr(one_line, "n=1")   != NULL);
    DL_ASSERT(strstr(one_line, "e=0")   != NULL);
}
```

- [ ] **Step 3: Implement `dl_latency.c` minimally to pass the two tests**

Create `drone/src/dl_latency.c`:

```c
/* dl_latency.c — see dl_latency.h. */
#include "dl_latency.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DL_LAT_WINDOW 128

struct dl_lat_slot {
    uint16_t samples_ms[DL_LAT_WINDOW];
    uint8_t  head;
    uint8_t  filled;
    uint64_t n_total;
    uint64_t n_err;
    uint16_t max_ms;
};

static struct dl_lat_slot g_slots[DL_LAT__COUNT];

static const char *tag_of(dl_lat_call_t w) {
    switch (w) {
    case DL_LAT_FEC:   return "FEC  ";
    case DL_LAT_DPTH:  return "DPTH ";
    case DL_LAT_RADIO: return "RADIO";
    case DL_LAT_TXPWR: return "TXPWR";
    case DL_LAT_ENC:   return "ENC  ";
    case DL_LAT_IDR:   return "IDR  ";
    case DL_LAT__COUNT: break;
    }
    return "?????";
}

void dl_latency_init(void) {
    memset(g_slots, 0, sizeof(g_slots));
}

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

dl_lat_handle_t dl_latency_begin(dl_lat_call_t which) {
    dl_lat_handle_t h = { .t_ns = now_ns(), .which = which };
    return h;
}

static void record_clamped(dl_lat_call_t which, uint64_t ms_u64, int rc) {
    if ((unsigned)which >= DL_LAT__COUNT) return;
    struct dl_lat_slot *s = &g_slots[which];
    uint16_t ms = (ms_u64 > UINT16_MAX) ? UINT16_MAX : (uint16_t)ms_u64;
    s->samples_ms[s->head] = ms;
    s->head = (uint8_t)((s->head + 1) % DL_LAT_WINDOW);
    if (s->filled < DL_LAT_WINDOW) s->filled++;
    s->n_total++;
    if (rc != 0) s->n_err++;
    if (ms > s->max_ms) s->max_ms = ms;
}

void dl_latency_end(dl_lat_handle_t h, int rc) {
    uint64_t end = now_ns();
    uint64_t ms = (end - h.t_ns + 500000ull) / 1000000ull;
    record_clamped(h.which, ms, rc);
}

#ifdef DL_TESTING
void dl_latency_record_raw(dl_lat_call_t which, uint64_t ms_u64, int rc) {
    record_clamped(which, ms_u64, rc);
}
#endif

static int cmp_u16(const void *a, const void *b) {
    uint16_t x = *(const uint16_t *)a;
    uint16_t y = *(const uint16_t *)b;
    return (x > y) - (x < y);
}

static void compute_percentiles(const struct dl_lat_slot *s,
                                uint16_t *p50, uint16_t *p95) {
    uint16_t tmp[DL_LAT_WINDOW];
    memcpy(tmp, s->samples_ms, sizeof(uint16_t) * s->filled);
    qsort(tmp, s->filled, sizeof(uint16_t), cmp_u16);
    *p50 = tmp[s->filled / 2];
    *p95 = tmp[(s->filled * 95) / 100];
}

size_t dl_latency_render(char *out, size_t out_len) {
    size_t off = 0;
    for (int i = 0; i < DL_LAT__COUNT; ++i) {
        const struct dl_lat_slot *s = &g_slots[i];
        const char *tag = tag_of((dl_lat_call_t)i);
        int row = 51 + i;
        int n;
        if (s->filled == 0) {
            n = snprintf(out + off, out_len - off,
                         "&L%d&F30 %s p50= -- p95= -- mx= -- n=%llu e=%llu\n",
                         row, tag,
                         (unsigned long long)s->n_total,
                         (unsigned long long)s->n_err);
        } else {
            uint16_t p50 = 0, p95 = 0;
            compute_percentiles(s, &p50, &p95);
            n = snprintf(out + off, out_len - off,
                         "&L%d&F30 %s p50=%3u p95=%3u mx=%3u n=%llu e=%llu\n",
                         row, tag,
                         (unsigned)p50, (unsigned)p95, (unsigned)s->max_ms,
                         (unsigned long long)s->n_total,
                         (unsigned long long)s->n_err);
        }
        if (n < 0 || (size_t)n >= out_len - off) {
            if (off < out_len) out[off] = '\0';
            return off;
        }
        off += (size_t)n;
    }
    return off;
}
```

- [ ] **Step 4: Build and run the two new tests directly**

Run:

```
cc -std=c11 -D_GNU_SOURCE -DDL_TESTING -Wall -Wextra -Wformat=2 \
  -Idrone/src -Itests/drone \
  -o /tmp/dl-latency-test \
  drone/src/dl_latency.c tests/drone/test_dl_latency.c tests/drone/test_main.c \
  && /tmp/dl-latency-test
```

Expected output ends with `2 tests, 2 passed, 0 failed` (or whatever the count is — exactly 2 here).

- [ ] **Step 5: Commit**

```
git add drone/src/dl_latency.h drone/src/dl_latency.c tests/drone/test_dl_latency.c
git commit -m "$(cat <<'EOF'
dl_latency: skeleton + empty/single-sample tests

New module owning per-call-site ring buffers and a render API for
the debug OSD. This commit stops at the two simplest unit tests
(empty render = dashes; single sample renders correctly). Subsequent
tasks add ring wrap, percentile, error-counter, clamp, and the
backend integration.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: ring wrap, max sticky, percentiles, error counter, clamp

Add the remaining behavior tests against the implementation already written. No production-code changes expected — these tests should pass against the Task 1 implementation. If any fail, fix the implementation accordingly.

**Files:**
- Modify: `tests/drone/test_dl_latency.c` (append tests)

- [ ] **Step 1: Append five more tests**

Append to `tests/drone/test_dl_latency.c`:

```c
DL_TEST(latency_records_error_counter) {
    dl_latency_init();
    dl_latency_record_raw(DL_LAT_FEC, 4, -1);
    dl_latency_record_raw(DL_LAT_FEC, 5,  0);
    dl_latency_record_raw(DL_LAT_FEC, 6, -1);

    char buf[1024];
    dl_latency_render(buf, sizeof(buf));
    const char *line = strstr(buf, "FEC  ");
    DL_ASSERT(line != NULL);
    DL_ASSERT(strstr(line, "n=3") != NULL);
    DL_ASSERT(strstr(line, "e=2") != NULL);
}

DL_TEST(latency_ring_wraps_at_128) {
    dl_latency_init();
    /* Push 200 distinct samples: 1..200. The last 128 (73..200) win. */
    for (uint32_t i = 1; i <= 200; ++i) {
        dl_latency_record_raw(DL_LAT_RADIO, i, 0);
    }
    char buf[1024];
    dl_latency_render(buf, sizeof(buf));
    const char *line = strstr(buf, "RADIO");
    DL_ASSERT(line != NULL);
    const char *eol = strchr(line, '\n');
    DL_ASSERT(eol != NULL);

    /* p50 over [73..200] = element at index 64 (sorted) = 73+64 = 137. */
    DL_ASSERT(strstr(line, "p50=137") != NULL);
    /* p95 over 128 sorted samples = index (128*95)/100 = 121, value 73+121 = 194. */
    DL_ASSERT(strstr(line, "p95=194") != NULL);
    /* mx sticky over all 200 = 200. */
    DL_ASSERT(strstr(line, "mx=200") != NULL);
    DL_ASSERT(strstr(line, "n=200") != NULL);
}

DL_TEST(latency_max_is_sticky_after_rollout) {
    dl_latency_init();
    /* One huge sample, then 128 tiny ones; the huge one rolls out of
     * the percentile window but max_ms stays. */
    dl_latency_record_raw(DL_LAT_ENC, 500, 0);
    for (int i = 0; i < 128; ++i) {
        dl_latency_record_raw(DL_LAT_ENC, 1, 0);
    }
    char buf[1024];
    dl_latency_render(buf, sizeof(buf));
    const char *line = strstr(buf, "ENC  ");
    DL_ASSERT(line != NULL);
    DL_ASSERT(strstr(line, "p50=  1") != NULL);
    DL_ASSERT(strstr(line, "p95=  1") != NULL);
    DL_ASSERT(strstr(line, "mx=500") != NULL);
}

DL_TEST(latency_clamps_giant_value) {
    dl_latency_init();
    dl_latency_record_raw(DL_LAT_TXPWR, 999999, 0);  /* > UINT16_MAX */
    char buf[1024];
    dl_latency_render(buf, sizeof(buf));
    const char *line = strstr(buf, "TXPWR");
    DL_ASSERT(line != NULL);
    /* UINT16_MAX = 65535. */
    DL_ASSERT(strstr(line, "p50=65535") != NULL);
    DL_ASSERT(strstr(line, "mx=65535") != NULL);
}

DL_TEST(latency_render_fits_buffer) {
    dl_latency_init();
    /* Worst case: large n_total / n_err on every slot. */
    for (int w = 0; w < DL_LAT__COUNT; ++w) {
        for (int i = 0; i < 1000; ++i) {
            dl_latency_record_raw((dl_lat_call_t)w, 123, (i & 1) ? -1 : 0);
        }
    }
    char buf[512];
    size_t n = dl_latency_render(buf, sizeof(buf));
    DL_ASSERT(n > 0);
    DL_ASSERT(n < sizeof(buf));
    DL_ASSERT(buf[n] == '\0');
}
```

- [ ] **Step 2: Rebuild and run**

Run the same compile command as Task 1 Step 4:

```
cc -std=c11 -D_GNU_SOURCE -DDL_TESTING -Wall -Wextra -Wformat=2 \
  -Idrone/src -Itests/drone \
  -o /tmp/dl-latency-test \
  drone/src/dl_latency.c tests/drone/test_dl_latency.c tests/drone/test_main.c \
  && /tmp/dl-latency-test
```

Expected: `7 tests, 7 passed, 0 failed`.

If `latency_ring_wraps_at_128` fails on p50/p95 values, sanity-check the index math: with `filled=128`, `p50 = tmp[64]`, `p95 = tmp[121]`. Sorted samples are `[73, 74, ..., 200]`, so `tmp[64]=137`, `tmp[121]=194`.

- [ ] **Step 3: Commit**

```
git add tests/drone/test_dl_latency.c
git commit -m "$(cat <<'EOF'
dl_latency: error counter, ring wrap, sticky max, clamp, buffer-fit tests

Locks in the remaining invariants for the dl_latency module against
the existing implementation. No production-code change.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: wire `dl_latency` into `drone/Makefile`

Get the new module compiled into both binaries and into the test runner so `make -C drone test` runs the latency unit tests automatically.

**Files:**
- Modify: `drone/Makefile`

- [ ] **Step 1: Add `dl_latency.c` to `APPLIER_SRCS`**

Edit `drone/Makefile`. In the `APPLIER_SRCS` list, add `$(SRCDIR)/dl_latency.c` between `dl_hello.c` and `dl_log.c`:

```make
APPLIER_SRCS := \
    $(SRCDIR)/dl_applier.c \
    $(SRCDIR)/dl_backend_enc.c \
    $(SRCDIR)/dl_backend_radio.c \
    $(SRCDIR)/dl_backend_tx.c \
    $(SRCDIR)/dl_config.c \
    $(SRCDIR)/dl_dbg.c \
    $(SRCDIR)/dl_dedup.c \
    $(SRCDIR)/dl_hello.c \
    $(SRCDIR)/dl_latency.c \
    $(SRCDIR)/dl_log.c \
    $(SRCDIR)/dl_mavlink.c \
    $(SRCDIR)/dl_osd.c \
    $(SRCDIR)/dl_watchdog.c \
    $(SRCDIR)/dl_wire.c \
    $(SRCDIR)/dl_yaml_get.c
```

- [ ] **Step 2: Add `dl_latency.c` and `test_dl_latency.c` to `TEST_SRCS`**

```make
TEST_SRCS := \
    $(TESTDIR)/test_main.c \
    $(TESTDIR)/test_apply_stagger.c \
    $(TESTDIR)/test_wire.c \
    $(TESTDIR)/test_config.c \
    $(TESTDIR)/test_dbg.c \
    $(TESTDIR)/test_dedup.c \
    $(TESTDIR)/test_dl_latency.c \
    $(TESTDIR)/test_mavlink.c \
    $(TESTDIR)/test_watchdog.c \
    $(TESTDIR)/test_dl_yaml.c \
    $(TESTDIR)/test_dl_hello.c \
    $(SRCDIR)/dl_wire.c \
    $(SRCDIR)/dl_config.c \
    $(SRCDIR)/dl_dbg.c \
    $(SRCDIR)/dl_dedup.c \
    $(SRCDIR)/dl_hello.c \
    $(SRCDIR)/dl_latency.c \
    $(SRCDIR)/dl_log.c \
    $(SRCDIR)/dl_mavlink.c \
    $(SRCDIR)/dl_watchdog.c \
    $(SRCDIR)/dl_yaml_get.c
```

- [ ] **Step 3: Pass `-DDL_TESTING` to the test target only**

Change the `$(TESTBIN)` recipe to inject the macro:

```make
$(TESTBIN): $(TEST_SRCS) | $(OUTDIR)
	$(CC) $(CFLAGS) -DDL_TESTING -I$(SRCDIR) -I$(TESTDIR) -o $@ $(TEST_SRCS) $(LDFLAGS)
```

(Only the recipe line changes; everything else in the rule stays.)

- [ ] **Step 4: Build and run the full test suite**

Run: `make -C drone test`
Expected: existing tests still pass and the seven new `latency_*` lines appear with `PASS`. Final line shows the full count (existing total + 7) with `0 failed`.

If a linker error complains about `clock_gettime`, that's because some toolchains need `-lrt`. On glibc Linux it's in libc; this should not be needed. Do not add `-lrt` unless `make -C drone test` actually fails on the link step — and if it does, add it to `LDFLAGS` in the recipe and re-run.

- [ ] **Step 5: Verify the applier still builds**

Run: `make -C drone`
Expected: `drone/build/dl-applier` and `drone/build/dl-inject` both produced, no warnings.

- [ ] **Step 6: Commit**

```
git add drone/Makefile
git commit -m "$(cat <<'EOF'
dl_latency: wire into drone Makefile

APPLIER_SRCS and TEST_SRCS now include dl_latency.c; test binary gets
-DDL_TESTING so the raw-sample injection helper is available. The
seven latency unit tests now run under `make -C drone test`.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: add `osd_debug_latency` config flag

One new bool, defaulted off, parsed, and documented in the sample file.

**Files:**
- Modify: `drone/src/dl_config.h`
- Modify: `drone/src/dl_config.c`
- Modify: `conf/drone.conf.sample`

- [ ] **Step 1: Add field to the config struct**

Edit `drone/src/dl_config.h`. After the existing OSD block (after `osd_update_interval_ms`):

```c
    /* OSD sink (§4B). */
    bool     osd_enable;
    char     osd_msg_path[DL_CONF_MAX_STR];
    uint32_t osd_update_interval_ms;
    /* Debug-OSD: per-apply-call latency stats. Off in prod. */
    bool     osd_debug_latency;
```

- [ ] **Step 2: Default the new field to false in `dl_config_defaults`**

Edit `drone/src/dl_config.c`. After the line `cfg->osd_update_interval_ms = 1000;` (around line 27):

```c
    cfg->osd_enable = true;
    strncpy(cfg->osd_msg_path, "/tmp/MSPOSD.msg", DL_CONF_MAX_STR - 1);
    cfg->osd_update_interval_ms = 1000;
    cfg->osd_debug_latency = false;
```

- [ ] **Step 3: Add a parser entry**

In `drone/src/dl_config.c`, in the key-dispatch block near line 175 (where `osd_*` keys live), add a new branch immediately after the existing `osd_update_interval_ms` branch:

```c
        else if (strcmp(key, "osd_update_interval_ms") == 0) SET_INT_RANGED(osd_update_interval_ms, uint32_t, 100, 60000);
        else if (strcmp(key, "osd_debug_latency") == 0)      SET_BOOL(osd_debug_latency);
```

- [ ] **Step 4: Document the flag in the sample file**

Edit `conf/drone.conf.sample`. Replace the existing OSD block (lines 47–50) with:

```ini
# ---- OSD sink (msposd) ----------------------------------------------
osd_enable              = 1
osd_msg_path            = /tmp/MSPOSD.msg
osd_update_interval_ms  = 1000

# Debug OSD: per-apply-call latency stats (p50/p95/max/n/err) for the
# six parameter-change paths — FEC, DEPTH, RADIO, TXPWR, ENC, IDR.
# Off by default; flip to 1 on a single drone for diagnosis. Has no
# effect when osd_enable = 0 (no file is written at all in that case).
osd_debug_latency       = 0
```

- [ ] **Step 5: Rebuild and run tests**

Run: `make -C drone test`
Expected: still all PASS, including the existing `test_config.c` cases. No new test added in this task — the parser entry is one trivial line that the e2e tests in Task 9 exercise.

Run: `make -C drone`
Expected: clean build.

- [ ] **Step 6: Commit**

```
git add drone/src/dl_config.h drone/src/dl_config.c conf/drone.conf.sample
git commit -m "$(cat <<'EOF'
config: add osd_debug_latency flag (default false)

Gates the new debug OSD lines. Documented in drone.conf.sample with
the dependency on osd_enable.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: integrate `dl_latency` rendering into `dl_osd`

Add the second-block render path to the OSD writer, refresh it from `dl_osd_write_status`, and write it after the status line in `flush()`.

**Files:**
- Modify: `drone/src/dl_osd.c`

- [ ] **Step 1: Add the two new fields to `struct dl_osd` and pull them from cfg**

In `drone/src/dl_osd.c`, replace the struct definition:

```c
struct dl_osd {
    char path[DL_CONF_MAX_STR];
    bool enabled;
    bool debug_latency_enabled;
    char status_line[128];
    char event_line[128];
    char debug_block[512];
};
```

In `dl_osd_open()`, after setting `o->enabled`:

```c
    snprintf(o->path, sizeof(o->path), "%s", cfg->osd_msg_path);
    o->enabled = cfg->osd_enable;
    o->debug_latency_enabled = cfg->osd_debug_latency;
    return o;
```

- [ ] **Step 2: Include the new header**

At the top of `drone/src/dl_osd.c`, alongside the existing includes:

```c
#include "dl_osd.h"
#include "dl_latency.h"
#include "dl_log.h"
```

- [ ] **Step 3: Update `flush()` to write the debug block after the status line**

Replace the body of `flush()`:

```c
static void flush(dl_osd_t *o) {
    if (!o->enabled) return;

    /* Write atomically: path.tmp → rename(path). Avoids msposd reading
     * a half-written buffer. */
    char tmp[DL_CONF_MAX_STR + 8];
    snprintf(tmp, sizeof(tmp), "%s.tmp", o->path);
    FILE *fd = fopen(tmp, "w");
    if (!fd) {
        dl_log_debug("osd: fopen %s: %s", tmp, strerror(errno));
        return;
    }
    if (o->event_line[0])  fprintf(fd, "%s\n", o->event_line);
    if (o->status_line[0]) fprintf(fd, "%s\n", o->status_line);
    if (o->debug_block[0]) fputs(o->debug_block, fd);
    fflush(fd);
    fclose(fd);
    if (rename(tmp, o->path) < 0) {
        dl_log_debug("osd: rename %s -> %s: %s", tmp, o->path, strerror(errno));
        unlink(tmp);
    }
}
```

(`debug_block` already ends each line with `\n`, so use `fputs`, not `fprintf` with `%s\n`.)

- [ ] **Step 4: Refresh the debug block on every status write**

At the bottom of `dl_osd_write_status`, just before `flush(o);`:

```c
    /* Fresh status = the link recovered (or never tripped). Clear any
     * stale event line so a past WATCHDOG/REJECT toast doesn't sit on
     * the OSD forever — msposd will keep rendering the last bytes we
     * wrote, so we have to actively unset. */
    o->event_line[0] = '\0';
    if (o->debug_latency_enabled) {
        dl_latency_render(o->debug_block, sizeof(o->debug_block));
    } else {
        o->debug_block[0] = '\0';
    }
    flush(o);
```

- [ ] **Step 5: Build and run tests**

Run: `make -C drone test` — expected: all pass (no test in this task; covered by e2e in Task 9).
Run: `make -C drone` — expected: clean build.

- [ ] **Step 6: Commit**

```
git add drone/src/dl_osd.c
git commit -m "$(cat <<'EOF'
osd: render dl_latency block under the status line

dl_osd now carries a 512-byte debug_block populated by
dl_latency_render at the existing 1 Hz tick, written after status in
flush(). Rendering is gated by osd_debug_latency.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: call `dl_latency_init` from `dl_applier`

Initialize the singleton once at startup before any backend opens.

**Files:**
- Modify: `drone/src/dl_applier.c`

- [ ] **Step 1: Find the init site**

Run: `grep -n "dl_osd_open\|dl_backend_tx_open" drone/src/dl_applier.c`

Expected: lines around `dl_osd_open(&cfg)` and the backend opens (current line ~256). `dl_latency_init()` must be called before any of these — anywhere after `dl_config_load_or_default(...)` succeeds and before the backend `_open` calls.

- [ ] **Step 2: Add the include and the call**

At the top of `drone/src/dl_applier.c`, add the header next to the other module headers (alphabetical order with `#include "dl_..."`):

```c
#include "dl_latency.h"
```

Find the line immediately before `dl_osd_t *osd = dl_osd_open(&cfg);` (current line 256) and insert one line:

```c
    dl_latency_init();
    dl_osd_t *osd = dl_osd_open(&cfg);
```

- [ ] **Step 3: Build**

Run: `make -C drone && make -C drone test`
Expected: clean build, all tests still pass.

- [ ] **Step 4: Commit**

```
git add drone/src/dl_applier.c
git commit -m "$(cat <<'EOF'
applier: initialize dl_latency before backends open

Single call site at startup; cheap (memset of ~6 KB once).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: instrument the tx-cmd backend (FEC, DEPTH, RADIO)

Wrap each of `send_fec`, `send_depth`, `send_radio` around their `send_and_recv` calls.

**Files:**
- Modify: `drone/src/dl_backend_tx.c`

- [ ] **Step 1: Include the header**

Add to the includes block in `drone/src/dl_backend_tx.c`:

```c
#include "dl_backend_tx.h"
#include "dl_dbg.h"
#include "dl_latency.h"
#include "dl_log.h"
#include "vendored/tx_cmd.h"
```

- [ ] **Step 2: Wrap `send_fec`**

Replace the function body:

```c
static int send_fec(dl_backend_tx_t *bt, uint8_t k, uint8_t n) {
    cmd_req_t req = { .req_id = htonl(next_req_id()), .cmd_id = CMD_SET_FEC };
    cmd_resp_t resp;
    req.u.cmd_set_fec.k = k;
    req.u.cmd_set_fec.n = n;
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_FEC);
    int rc = send_and_recv(bt->fd, &req,
                           offsetof(cmd_req_t, u) + sizeof(req.u.cmd_set_fec),
                           &resp, "set_fec");
    dl_latency_end(h, rc);
    return rc;
}
```

- [ ] **Step 3: Wrap `send_depth`**

```c
static int send_depth(dl_backend_tx_t *bt, uint8_t depth) {
    cmd_req_t req = {
        .req_id = htonl(next_req_id()),
        .cmd_id = CMD_SET_INTERLEAVE_DEPTH,
    };
    cmd_resp_t resp;
    req.u.cmd_set_interleave_depth.depth = depth;
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_DPTH);
    int rc = send_and_recv(bt->fd, &req,
                           offsetof(cmd_req_t, u) +
                               sizeof(req.u.cmd_set_interleave_depth),
                           &resp, "set_interleave_depth");
    dl_latency_end(h, rc);
    return rc;
}
```

- [ ] **Step 4: Wrap `send_radio`**

```c
static int send_radio(dl_backend_tx_t *bt, uint8_t mcs, uint8_t bandwidth) {
    cmd_req_t req = { .req_id = htonl(next_req_id()), .cmd_id = CMD_SET_RADIO };
    cmd_resp_t resp;
    req.u.cmd_set_radio.stbc       = 0;
    req.u.cmd_set_radio.ldpc       = false;
    req.u.cmd_set_radio.short_gi   = false;  /* pinned (§1) */
    req.u.cmd_set_radio.bandwidth  = bandwidth;
    req.u.cmd_set_radio.mcs_index  = mcs;
    req.u.cmd_set_radio.vht_mode   = false;
    req.u.cmd_set_radio.vht_nss    = 1;
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_RADIO);
    int rc = send_and_recv(bt->fd, &req,
                           offsetof(cmd_req_t, u) + sizeof(req.u.cmd_set_radio),
                           &resp, "set_radio");
    dl_latency_end(h, rc);
    return rc;
}
```

- [ ] **Step 5: Build and run tests**

Run: `make -C drone && make -C drone test`
Expected: clean build, all unit tests still pass. (No new unit test in this task — the e2e tests in Task 9 cover this.)

- [ ] **Step 6: Commit**

```
git add drone/src/dl_backend_tx.c
git commit -m "$(cat <<'EOF'
backend_tx: time send_fec / send_depth / send_radio for debug OSD

Wraps each tx_cmd sub-command in dl_latency_begin/end. Covers normal
apply and safe-defaults equally — same helpers in both paths.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: instrument the radio (iw) backend

Wrap `run_iw` around its full posix_spawnp → waitpid lifetime.

**Files:**
- Modify: `drone/src/dl_backend_radio.c`

- [ ] **Step 1: Include the header**

Add to the includes at the top of `drone/src/dl_backend_radio.c`:

```c
#include "dl_backend_radio.h"
#include "dl_dbg.h"
#include "dl_latency.h"
#include "dl_log.h"
```

- [ ] **Step 2: Wrap `run_iw`**

Convert `run_iw` to capture an exit code and route through one `dl_latency_end`:

```c
static int run_iw(const char *wlan, int8_t dBm) {
    char mBm_str[16];
    snprintf(mBm_str, sizeof(mBm_str), "%d", (int)dBm * 100);

    char *const argv[] = {
        (char *)"iw",
        (char *)"dev",
        (char *)wlan,
        (char *)"set",
        (char *)"txpower",
        (char *)"fixed",
        mBm_str,
        NULL,
    };

    dl_lat_handle_t h = dl_latency_begin(DL_LAT_TXPWR);
    pid_t pid;
    int rc_spawn = posix_spawnp(&pid, "iw", NULL, NULL, argv, environ);
    if (rc_spawn != 0) {
        dl_log_warn("radio: posix_spawnp iw: %s", strerror(rc_spawn));
        char detail[96];
        snprintf(detail, sizeof(detail),
                 "{\"errno\":%d,\"dBm\":%d}", rc_spawn, (int)dBm);
        dl_dbg_emit("RADIO_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
        dl_latency_end(h, -1);
        return -1;
    }
    int status;
    if (waitpid(pid, &status, 0) < 0) {
        int saved = errno;
        dl_log_warn("radio: waitpid: %s", strerror(saved));
        char detail[96];
        snprintf(detail, sizeof(detail),
                 "{\"errno\":%d,\"dBm\":%d}", saved, (int)dBm);
        dl_dbg_emit("RADIO_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
        dl_latency_end(h, -1);
        return -1;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        dl_log_warn("radio: iw exited with status %d (txpower=%d dBm)",
                    exit_code, (int)dBm);
        char detail[96];
        snprintf(detail, sizeof(detail),
                 "{\"exit\":%d,\"dBm\":%d}", exit_code, (int)dBm);
        dl_dbg_emit("RADIO_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
        dl_latency_end(h, -1);
        return -1;
    }
    dl_latency_end(h, 0);
    dl_log_info("radio: txpower %d dBm (iw dev %s)", (int)dBm, wlan);
    return 0;
}
```

- [ ] **Step 3: Build and run tests**

Run: `make -C drone && make -C drone test`
Expected: clean build, all tests pass.

- [ ] **Step 4: Commit**

```
git add drone/src/dl_backend_radio.c
git commit -m "$(cat <<'EOF'
backend_radio: time run_iw for debug OSD

Single dl_latency wrap covering posix_spawnp through waitpid. Every
exit path (spawn fail, wait fail, non-zero exit, success) routes
through exactly one dl_latency_end with the right rc.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: instrument the encoder backend and flip the non-2xx return contract

Wrap `apply_set` and `dl_backend_enc_request_idr` (non-throttle path only); change `http_get` non-2xx to return `-1`; update the comment that defends the old behavior.

**Files:**
- Modify: `drone/src/dl_backend_enc.c`

- [ ] **Step 1: Include the header**

Add to the includes at the top of `drone/src/dl_backend_enc.c`:

```c
#include "dl_backend_enc.h"
#include "dl_dbg.h"
#include "dl_latency.h"
#include "dl_log.h"
```

- [ ] **Step 2: Flip the non-2xx-HTTP return and update the comment**

In `drone/src/dl_backend_enc.c`, find the block at the end of `http_get` (currently lines 149–177). Replace from the `int status = parse_http_status(...)` line through the closing `return 0;`:

```c
    int status = parse_http_status(reply, reply_len);
    if (status >= 200 && status < 300) {
        return 0;
    }
    /* Non-2xx (or unparseable): capture the body for the SD log and
     * propagate as an error. This was previously returned 0 to avoid
     * the MAVLink apply_fail path; the debug-OSD work made surfacing
     * application-layer rejects worth the extra STATUSTEXT noise. See
     * docs/superpowers/specs/2026-05-13-debug-osd-apply-latency-design.md
     * §"Encoder error-contract change". */
    dl_log_warn("enc: http %d from %s:%u for %s",
                status, host, port, path);
    const char *body = reply;
    size_t body_len = reply_len;
    for (size_t i = 0; i + 3 < reply_len; ++i) {
        if (reply[i] == '\r' && reply[i+1] == '\n' &&
            reply[i+2] == '\r' && reply[i+3] == '\n') {
            body = reply + i + 4;
            body_len = reply_len - (i + 4);
            break;
        }
    }
    dl_dbg_emit_http("ENC_RESPONSE_BAD", DL_DBG_SEV_WARN,
                     status, body, body_len);
    return -1;
}
```

(The silent-encoder `return 0;` earlier in the function — for the `reply_len == 0` case — stays unchanged.)

- [ ] **Step 3: Wrap `apply_set`**

```c
static int apply_set(dl_backend_enc_t *be,
                     uint16_t bitrate_kbps, uint8_t roi_qp, uint8_t fps) {
    char path[256];
    char *p = path;
    size_t left = sizeof(path);
    int n = snprintf(p, left, "/api/v1/set?video0.bitrate=%u", bitrate_kbps);
    if (n < 0 || (size_t)n >= left) return -1;
    p += n; left -= (size_t)n;
    if (roi_qp != 0) {
        n = snprintf(p, left, "&fpv.roiQp=%u", roi_qp);
        if (n < 0 || (size_t)n >= left) return -1;
        p += n; left -= (size_t)n;
    }
    if (fps != 0) {
        n = snprintf(p, left, "&video0.fps=%u", fps);
        if (n < 0 || (size_t)n >= left) return -1;
        p += n; left -= (size_t)n;
    }
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_ENC);
    int rc = http_get(be->host, be->port, path);
    dl_latency_end(h, rc);
    return rc;
}
```

- [ ] **Step 4: Wrap `dl_backend_enc_request_idr` — non-throttle path only**

```c
int dl_backend_enc_request_idr(dl_backend_enc_t *be, uint64_t now_ms) {
    if (!be) return -1;
    if (be->idr_ever_sent &&
        (now_ms - be->last_idr_ms) < be->min_idr_interval_ms) {
        dl_log_debug("enc: IDR throttled (%u ms since last, min %u)",
                     (unsigned)(now_ms - be->last_idr_ms),
                     (unsigned)be->min_idr_interval_ms);
        return 1;
    }
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_IDR);
    int rc = http_get(be->host, be->port, "/request/idr");
    dl_latency_end(h, rc);
    if (rc == 0) {
        be->last_idr_ms = now_ms;
        be->idr_ever_sent = true;
    }
    return rc;
}
```

(Throttle path returns `1` early without calling `begin`/`end` — the IDR was *not* sent, so it must not count toward `n` or latency.)

- [ ] **Step 5: Build and run tests**

Run: `make -C drone && make -C drone test`
Expected: clean build, all unit tests pass. Existing Python e2e tests in `tests/test_drone_e2e.py` still pass — confirm with `python3 -m pytest tests/test_drone_e2e.py --ignore=tests/test_mavlink_status.py -v`. The mock encoder always returns 200 today, so the contract flip doesn't break existing assertions.

- [ ] **Step 6: Commit**

```
git add drone/src/dl_backend_enc.c
git commit -m "$(cat <<'EOF'
backend_enc: time apply_set/request_idr; non-2xx now returns -1

Wraps the encoder HTTP calls in dl_latency_begin/end. The throttled
IDR path returns early without sampling so 'n' counts real sends only.

Also flips the long-standing 'completed exchange = success' contract:
non-2xx HTTP replies now propagate as -1, so the new ENC error counter
actually fires on majestic 4xx/5xx and the GS sees an APPLY_FAIL
STATUSTEXT. The previous behavior cited 'noisier MAVLink apply_fail
path' as the reason to swallow — with the debug OSD landing, that
signal is worth the extra noise.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: end-to-end Python tests

Four new tests in `tests/test_drone_e2e.py`. Requires a small extension to `make_encoder_server` so a test can force the mock encoder to return HTTP 500.

**Files:**
- Modify: `tests/test_drone_e2e.py`

- [ ] **Step 1: Extend `make_encoder_server` to support a configurable status**

Replace the `_RecordingHandler` and `make_encoder_server` definitions (lines 129–150) with:

```python
class _RecordingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.server.recorded.append(self.path)  # type: ignore[attr-defined]
        self.server.recv_times.append(time.monotonic())  # type: ignore[attr-defined]
        status = getattr(self.server, "response_status", 200)
        body = getattr(self.server, "response_body", b"{}")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args, **kwargs):
        pass  # silence


def make_encoder_server(status: int = 200, body: bytes = b"{}") -> HTTPServer:
    srv = HTTPServer(("127.0.0.1", 0), _RecordingHandler)
    srv.recorded = []  # type: ignore[attr-defined]
    srv.recv_times = []  # type: ignore[attr-defined]
    srv.response_status = status  # type: ignore[attr-defined]
    srv.response_body = body      # type: ignore[attr-defined]
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    srv._thread = t  # type: ignore[attr-defined]
    return srv
```

- [ ] **Step 2: Make `_sandbox` accept a custom encoder status**

In `_sandbox` (around line 297), change the encoder construction. Replace:

```python
    enc = make_encoder_server()
```

with:

```python
    enc = make_encoder_server(
        status=overrides.pop("_encoder_status", 200),
    )
```

(Pop so the bogus key doesn't end up in the drone.conf file.)

- [ ] **Step 3: Add the four new tests at the bottom of `tests/test_drone_e2e.py`**

Append:

```python
def _osd_text(osd_path: Path) -> str:
    if not osd_path.exists():
        return ""
    return osd_path.read_text(errors="replace")


def test_osd_debug_latency_disabled_default(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60)
        # Status line must appear (existing behavior).
        assert _wait_until(
            lambda: "MCS" in _osd_text(s["osd_path"])), "no status line"
        # Debug lines must NOT appear.
        txt = _osd_text(s["osd_path"])
        for tag in ("FEC  ", "DPTH ", "RADIO", "TXPWR", "ENC  ", "IDR  "):
            assert tag not in txt, f"unexpected debug tag {tag!r} in: {txt!r}"


def test_osd_debug_latency_enabled_renders_six_lines(tmp_path: Path):
    with _sandbox(tmp_path, osd_debug_latency=1) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60)
        # Wait for at least one OSD refresh after the first apply.
        assert _wait_until(
            lambda: all(tag in _osd_text(s["osd_path"])
                        for tag in ("FEC  ", "DPTH ", "RADIO",
                                    "TXPWR", "ENC  ", "IDR  ")),
            timeout_s=3.0,
        ), f"missing debug tag(s): {_osd_text(s['osd_path'])!r}"


def test_osd_debug_latency_error_counter_increments(tmp_path: Path):
    with _sandbox(tmp_path, osd_debug_latency=1, _encoder_status=500) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60)
        # After at least one ENC apply, the ENC line should show e>=1.
        def enc_line_has_error():
            txt = _osd_text(s["osd_path"])
            for line in txt.splitlines():
                if "ENC  " in line and "e=0" not in line and "e=" in line:
                    return True
            return False
        assert _wait_until(enc_line_has_error, timeout_s=3.0), \
            f"no ENC error in: {_osd_text(s['osd_path'])!r}"


def test_osd_debug_latency_idr_throttle_not_counted(tmp_path: Path):
    # Long throttle so the second IDR is definitely dropped.
    with _sandbox(tmp_path,
                  osd_debug_latency=1,
                  min_idr_interval_ms=10_000) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        # Two decisions with the IDR flag set, distinct sequence
        # numbers so the dedup doesn't drop the second decision.
        # The applier issues two enc_request_idr calls; the second
        # hits the throttle and returns early WITHOUT calling
        # dl_latency_end (spec §"Tracked call sites"), so IDR n=1.
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, fps=60,
                idr=True, sequence=1)
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, fps=60,
                idr=True, sequence=2)

        def idr_line():
            for line in _osd_text(s["osd_path"]).splitlines():
                if "IDR  " in line:
                    return line
            return None

        # Wait for the IDR line to appear with n=1 (the recorded
        # first IDR; the second is throttled and never sampled).
        def idr_n_is_one():
            line = idr_line()
            return line is not None and "n=1" in line
        assert _wait_until(idr_n_is_one, timeout_s=3.0), \
            f"expected IDR n=1, got: {_osd_text(s['osd_path'])!r}"

        # One more OSD tick: confirm n stays at 1 (the throttled
        # second call must not have crept in).
        time.sleep(s["cfg"]["osd_update_interval_ms"] / 1000.0 + 0.3)
        line = idr_line()
        assert line is not None
        assert "n=1" in line, f"expected n=1 (throttled), got line: {line!r}"
```

- [ ] **Step 4: Run the new tests**

Run: `python3 -m pytest tests/test_drone_e2e.py::test_osd_debug_latency_disabled_default tests/test_drone_e2e.py::test_osd_debug_latency_enabled_renders_six_lines tests/test_drone_e2e.py::test_osd_debug_latency_error_counter_increments tests/test_drone_e2e.py::test_osd_debug_latency_idr_throttle_not_counted --ignore=tests/test_mavlink_status.py -v`

Expected: 4 passed.

If `test_osd_debug_latency_error_counter_increments` flakes, raise the timeout to 5s — the encoder backend will retry on a real connection or the OSD tick (200 ms in the sandbox config) might miss the first render.

- [ ] **Step 5: Run the full Python suite to ensure no regression in existing tests**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -v`
Expected: all green (existing + 4 new).

- [ ] **Step 6: Run the full C suite as a sanity check**

Run: `make -C drone test`
Expected: all green.

- [ ] **Step 7: Commit**

```
git add tests/test_drone_e2e.py
git commit -m "$(cat <<'EOF'
e2e: debug OSD coverage — defaults off, six lines on, ENC error, IDR throttle

Four pytest cases against a real dl-applier in the sandbox. Encoder
fixture now accepts a status code so the error-counter case can force
HTTP 500.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Self-Review

**Spec coverage:** Module API (Tasks 1-2), call-site integration (Tasks 7-9), encoder error-contract change (Task 9), config schema (Task 4), OSD writer changes (Task 5), `dl_latency_init` location (Task 6), C unit tests (Tasks 1-2), e2e tests (Task 10), Makefile wiring (Task 3). All eight spec areas have at least one task.

**Type consistency:** `dl_lat_call_t` enum values, `dl_lat_handle_t` shape (`{t_ns, which}`), API signatures (`begin(which)`, `end(h, rc)`, `render(out, len)`) are identical across the header (Task 1), implementation (Task 1), and every call-site insertion (Tasks 7, 8, 9). Field names `osd_debug_latency` (config), `debug_latency_enabled` / `debug_block` (struct dl_osd) used consistently. Tag strings (`FEC  `, `DPTH `, `RADIO`, `TXPWR`, `ENC  `, `IDR  `) match across module render code, e2e test assertions, and the spec.

**Placeholder scan:** No TODO/TBD/"as appropriate". Every step includes the exact code or command.

---

## Open notes for the implementer

- `cmd_resp_t` and `next_req_id` references in Task 7 are existing types/functions in `dl_backend_tx.c`; the wrap only adds two lines per function.
- The e2e test `test_osd_debug_latency_idr_throttle_not_counted` relies on the applier's existing IDR-on-bitrate-up trigger (see `dl_applier.c` lines 390 and 414). The second inject only raises bitrate further (12000 → 14000), so a second IDR is requested but throttled. Spec §"Tracked call sites" calls out that throttled IDRs are not sampled.
- The C test `latency_ring_wraps_at_128` p50/p95 math: with 200 samples pushed and a 128-slot ring, samples 73–200 survive. Sorted: index 64 = 137, index 121 = 194.
- No GS-side code changes. `gs/dynamic_link/` is untouched throughout the plan.
