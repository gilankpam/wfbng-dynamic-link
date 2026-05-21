# Remove dl IDR path, receive PixelPilot IDR on the drone — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace dl's residual_loss-driven IDR signalling with a UDP listener in `dl-applier` (port 11223) that receives PixelPilot_rk's IDR-token bursts. Remove the GS-side wire bit, burster, policy trigger, and all consumers of the `idr_request` field. Add an OSD counter for received IDR requests.

**Architecture:** Two flow changes. (1) **Add** `dl_idr_listen` UDP module on the drone, register its fd in the existing `poll(2)` loop in `dl_applier.c`, drain bursts in one wake, call existing `dl_backend_enc_request_idr()` once per drain — the existing `min_idr_interval_ms` throttle collapses bursts to a single HTTP call. Counter bumps a new `dl_osd` field rendered as `I<n>` in the status line. (2) **Remove** the wire-format `FLAG_IDR_REQUEST = 0x01` from both `dl_wire.h` and `wire.py` (and three dispatch sites in `dl_applier.c`), the `idr_request` field on the `Decision` dataclass, `idr_burst.py` (deleted), `TrailingLoop`'s IDR return, the `service.py` burster wiring, the `flight_log.py` JSONL field, and the `dl_replay`/`dl_report` consumers. `dl-inject`'s `--idr` CLI flag goes too.

**Tech Stack:** C11 (drone, `poll(2)`, POSIX sockets), Python 3 stdlib asyncio (GS), pytest + custom DL_TEST C harness. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-05-21-remove-dl-idr-design.md`

---

## File Structure

**Drone-side (C) — new:**
- `drone/src/dl_idr_listen.h` — module header, ~4 functions.
- `drone/src/dl_idr_listen.c` — implementation, ~70 lines.
- `tests/drone/test_idr_listen.c` — unit tests.
- `tests/drone/test_osd.c` — unit test for the OSD `I<n>` counter (new file, none exists today).

**Drone-side (C) — modified:**
- `drone/src/dl_config.h` — add `idr_listen_port`, `idr_listen_addr` fields.
- `drone/src/dl_config.c` — parse the two new keys, defaults `11223` / `0.0.0.0`.
- `drone/src/dl_applier.c` — open listener, extend `pfds[]` from 4 to 5, dispatch IDR drain → `dl_backend_enc_request_idr` + `dl_osd_bump_idr`; **also** remove the three `if (... & DL_FLAG_IDR_REQUEST) dl_backend_enc_request_idr(...)` dispatch sites at lines 391–393, 415–417, 488–490.
- `drone/src/dl_osd.h` — declare `dl_osd_bump_idr`.
- `drone/src/dl_osd.c` — add `idr_requests` field to struct, bump function, `I%u` in status-line format.
- `drone/src/dl_wire.h` — remove `DL_FLAG_IDR_REQUEST = 0x01`.
- `drone/src/dl_inject.c` — remove `--idr` CLI long option and `IDR` in dry-run printout.
- `drone/Makefile` — add `dl_idr_listen.c` to `APPLIER_SRCS` and `TEST_SRCS`; add `test_idr_listen.c` and `test_osd.c` to `TEST_SRCS`.
- `tests/drone/test_wire.c` — remove `DL_FLAG_IDR_REQUEST` setter/assertion (lines 9, 38).
- `conf/drone.conf.sample` — document the two new keys.

**GS-side (Python) — deleted:**
- `gs/dynamic_link/idr_burst.py`
- `tests/test_idr_burst.py`

**GS-side (Python) — modified:**
- `gs/dynamic_link/wire.py` — remove `FLAG_IDR_REQUEST`, drop `idr_request` kwarg + bit from `Encoder.encode`.
- `gs/dynamic_link/decision.py` — remove `idr_request: bool` field.
- `gs/dynamic_link/policy.py` — `TrailingLoop.tick()` returns `int` (depth only); remove `idr` local at line 622, the loss-triggered `idr = True` block at lines 624–633, the vanilla-path `idr` block at lines 598–603, and the `idr_request=idr` construction in `Policy.tick()`.
- `gs/dynamic_link/service.py` — remove `IdrBurster` import, instantiation (line 456), and `trigger()` call (lines 574–575).
- `gs/dynamic_link/flight_log.py` — drop `idr_request` from the per-tick JSONL record schema.
- `gs/tools/dl_replay.py` — drop `idr_request=bool(rec["idr_request"])` at line 70.
- `gs/tools/dl_report.py` — drop `summary["idr_requests"]` block at lines 189–193.
- `conf/gs.yaml.sample` — drop any `idr_burst:` block.
- `README.md` — update IDR description to mention PixelPilot dependency + UDP listener port.

**Tests modified (Python):**
- `tests/test_wire_contract.py` — drop `idr_request=True` fixture (line 85) and the `idr_request=False` baseline (line 58).
- `tests/test_phase2_e2e.py` — delete `test_gs_wire_idr_flag_triggers_idr_request` (line 68+), drop `idr_request=False` at line 36.
- `tests/test_flight_log.py` — drop `idr_request=False` at line 258.
- `tests/test_dl_review.py` — drop fields at lines 31, 38, 42.
- `tests/test_dl_replay.py` — drop field at line 27.
- `tests/test_phase3_e2e.py` — drop field at line 71.
- `tests/bench/per_knob_loss_bench.py` — drop field at line 134.
- `tests/test_drone_e2e.py` — delete `test_idr_throttle_drops_duplicates` (line 481) and `test_idr_burst_delivers_multiple_packets` (line 496); add new `test_pixelpilot_udp_token_triggers_idr` and `test_pixelpilot_udp_burst_collapses_to_one_idr`. Update `test_osd_debug_latency_idr_throttle_not_counted` to drive IDR via UDP instead of `idr=True` decision.

---

## Task 1: New module `dl_idr_listen`

**Files:**
- Create: `drone/src/dl_idr_listen.h`
- Create: `drone/src/dl_idr_listen.c`
- Create: `tests/drone/test_idr_listen.c`
- Modify: `drone/Makefile` (add new sources to `APPLIER_SRCS` and `TEST_SRCS`)

- [ ] **Step 1: Write the failing tests**

Create `tests/drone/test_idr_listen.c`:

```c
/* test_idr_listen.c — unit tests for dl_idr_listen. */
#include "dl_idr_listen.h"
#include "test_main.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

DL_TEST(test_idr_listen_port_zero_disabled) {
    dl_idr_listen_t *l = dl_idr_listen_open("127.0.0.1", 0);
    DL_ASSERT(l == NULL);
}

DL_TEST(test_idr_listen_bind_drain_three) {
    /* Fixed high port; the test harness is single-threaded so
     * collisions with a real wfb-ng IDR helper on the dev host are
     * vanishingly rare. */
    const uint16_t PORT = 51123;
    dl_idr_listen_t *l = dl_idr_listen_open("127.0.0.1", PORT);
    DL_ASSERT(l != NULL);
    DL_ASSERT(dl_idr_listen_fd(l) >= 0);

    DL_ASSERT_EQ(dl_idr_listen_drain(l), 0);

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    DL_ASSERT(s >= 0);
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(PORT);
    inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
    const char msg[] = "abc\n";
    for (int i = 0; i < 3; i++) {
        ssize_t r = sendto(s, msg, sizeof(msg) - 1, 0,
                           (struct sockaddr *)&dst, sizeof(dst));
        DL_ASSERT_EQ(r, (ssize_t)(sizeof(msg) - 1));
    }
    usleep(10 * 1000);   /* let kernel deliver to loopback queue */
    DL_ASSERT_EQ(dl_idr_listen_drain(l), 3);
    DL_ASSERT_EQ(dl_idr_listen_drain(l), 0);

    close(s);
    dl_idr_listen_close(l);
}

DL_TEST(test_idr_listen_close_null_safe) {
    dl_idr_listen_close(NULL);  /* must not crash */
    DL_ASSERT(dl_idr_listen_fd(NULL) == -1);
    DL_ASSERT(dl_idr_listen_drain(NULL) == 0);
}
```

- [ ] **Step 2: Run the tests, confirm they fail to compile**

Run: `make -C drone test`
Expected: compilation fails — `dl_idr_listen.h` doesn't exist yet.

- [ ] **Step 3: Write the header**

Create `drone/src/dl_idr_listen.h`:

```c
/* dl_idr_listen.h — UDP listener for PixelPilot IDR-token bursts.
 *
 * PixelPilot_rk sends short UDP datagrams (typically 6 bytes, a
 * random 3-byte ASCII token + newline) to the drone on port 11223
 * when it detects an RTP sequence gap or a decode stall. Anything
 * arriving on this socket is treated as an IDR request; the caller
 * is responsible for throttling the encoder API.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct dl_idr_listen dl_idr_listen_t;

/* Open a non-blocking AF_INET SOCK_DGRAM socket bound to
 * `bind_addr:port`. Returns NULL on error (logged) or if port == 0
 * (disabled). `bind_addr` of NULL or "" binds to 0.0.0.0. */
dl_idr_listen_t *dl_idr_listen_open(const char *bind_addr, uint16_t port);

/* Bound fd for poll(2). Returns -1 if l == NULL. */
int dl_idr_listen_fd(const dl_idr_listen_t *l);

/* Drain all queued datagrams (recvfrom until EAGAIN). Returns the
 * count consumed. Used for logging only — the caller decides whether
 * to actuate IDR based on count > 0. NULL-safe (returns 0). */
size_t dl_idr_listen_drain(dl_idr_listen_t *l);

/* Close and free. NULL-safe. */
void dl_idr_listen_close(dl_idr_listen_t *l);
```

- [ ] **Step 4: Write the implementation**

Create `drone/src/dl_idr_listen.c`:

```c
/* dl_idr_listen.c */
#include "dl_idr_listen.h"
#include "dl_log.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

struct dl_idr_listen {
    int fd;
};

dl_idr_listen_t *dl_idr_listen_open(const char *bind_addr, uint16_t port) {
    if (port == 0) return NULL;

    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        dl_log_err("idr_listen: socket: %s", strerror(errno));
        return NULL;
    }
    int one = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in sa = {0};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port);
    if (!bind_addr || !*bind_addr) {
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, bind_addr, &sa.sin_addr) != 1) {
        dl_log_err("idr_listen: bad bind_addr '%s'", bind_addr);
        close(fd);
        return NULL;
    }
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        dl_log_err("idr_listen: bind %s:%u: %s",
                   bind_addr && *bind_addr ? bind_addr : "0.0.0.0",
                   port, strerror(errno));
        close(fd);
        return NULL;
    }

    dl_idr_listen_t *l = calloc(1, sizeof(*l));
    if (!l) { close(fd); return NULL; }
    l->fd = fd;
    dl_log_info("idr_listen: bound %s:%u",
                bind_addr && *bind_addr ? bind_addr : "0.0.0.0", port);
    return l;
}

int dl_idr_listen_fd(const dl_idr_listen_t *l) {
    return l ? l->fd : -1;
}

size_t dl_idr_listen_drain(dl_idr_listen_t *l) {
    if (!l) return 0;
    size_t count = 0;
    uint8_t buf[64];
    while (1) {
        ssize_t n = recvfrom(l->fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            dl_log_debug("idr_listen: recvfrom: %s", strerror(errno));
            break;
        }
        count++;
    }
    return count;
}

void dl_idr_listen_close(dl_idr_listen_t *l) {
    if (!l) return;
    if (l->fd >= 0) close(l->fd);
    free(l);
}
```

- [ ] **Step 5: Wire into the Makefile**

Edit `drone/Makefile`:

In `APPLIER_SRCS`, add the new source after `$(SRCDIR)/dl_hello.c`:

```make
    $(SRCDIR)/dl_idr_listen.c \
```

In `TEST_SRCS`, add the new sources alongside the existing test files and source dependencies:

```make
    $(TESTDIR)/test_idr_listen.c \
```

and in the source-dependency section of `TEST_SRCS`:

```make
    $(SRCDIR)/dl_idr_listen.c \
```

- [ ] **Step 6: Build + run tests, confirm they pass**

Run: `nix-shell --run 'make -C drone test'`
Expected: all tests pass, including the three new `test_idr_listen_*` cases.

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_idr_listen.h drone/src/dl_idr_listen.c \
        tests/drone/test_idr_listen.c drone/Makefile
git commit -m "$(cat <<'EOF'
drone: add dl_idr_listen UDP module

Foundational module for receiving PixelPilot IDR-token bursts on
port 11223. Caller drains queued datagrams in one poll wake and
decides whether to actuate the encoder; throttle lives in the
existing dl_backend_enc layer.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Config keys + wire listener into the applier poll loop

**Files:**
- Modify: `drone/src/dl_config.h` (add fields)
- Modify: `drone/src/dl_config.c` (parse keys + defaults)
- Modify: `tests/drone/test_config.c` (extend default-values test)
- Modify: `drone/src/dl_applier.c` (open listener, extend `pfds[]`, dispatch)
- Modify: `conf/drone.conf.sample` (document keys)

- [ ] **Step 1: Add the config fields to the struct**

Edit `drone/src/dl_config.h`. Find the existing struct block (search for `min_idr_interval_ms` — it lives next to other backend-tuning fields). Add immediately below `min_idr_interval_ms`:

```c
    /* PixelPilot IDR-token listener. port == 0 disables. */
    uint16_t idr_listen_port;
    char     idr_listen_addr[DL_CONF_MAX_STR];
```

- [ ] **Step 2: Add the parser + defaults**

Edit `drone/src/dl_config.c`. In the defaults-setup function (search for the existing `min_idr_interval_ms = 500;` assignment), add:

```c
    cfg->idr_listen_port = 11223;
    snprintf(cfg->idr_listen_addr, sizeof(cfg->idr_listen_addr), "%s", "0.0.0.0");
```

In the per-key parse ladder (search for the `else if` branches matching `"min_idr_interval_ms"`), add two new branches:

```c
    } else if (strcmp(key, "idr_listen_port") == 0) {
        cfg->idr_listen_port = (uint16_t)strtoul(value, NULL, 10);
    } else if (strcmp(key, "idr_listen_addr") == 0) {
        snprintf(cfg->idr_listen_addr, sizeof(cfg->idr_listen_addr),
                 "%s", value);
```

- [ ] **Step 3: Extend the config defaults test**

Edit `tests/drone/test_config.c`. Find the existing `DL_TEST` that asserts default values (search for `min_idr_interval_ms`). Add two assertions next to it:

```c
    DL_ASSERT_EQ(cfg.idr_listen_port, 11223);
    DL_ASSERT_STR_EQ(cfg.idr_listen_addr, "0.0.0.0");
```

If there's a "parses keys from a tempfile" test, add a key=value line for both new keys and assert they round-trip.

- [ ] **Step 4: Run tests, confirm config side passes**

Run: `nix-shell --run 'make -C drone test'`
Expected: all pass. The applier still compiles because we haven't touched it yet.

- [ ] **Step 5: Open the listener in the applier**

Edit `drone/src/dl_applier.c`. Add include at the top:

```c
#include "dl_idr_listen.h"
```

In the function that sets up runtime state (search for where `listen_fd` is created, near the top of the main applier loop function), after the existing `listen_fd` setup add:

```c
    dl_idr_listen_t *idr_listen = dl_idr_listen_open(cfg.idr_listen_addr,
                                                     cfg.idr_listen_port);
    /* idr_listen == NULL when port=0 (disabled) or on bind failure;
     * either way the poll branch becomes a no-op via fd == -1. */
```

Just before the main `while (!g_stop)` loop (around line 290), extend `pfds[]` from 4 to 5 slots:

```c
    struct pollfd pfds[5];
    pfds[0].fd = listen_fd;      pfds[0].events = POLLIN;
    pfds[1].fd = tick_fd;        pfds[1].events = POLLIN;
    pfds[2].fd = gap_fd;         pfds[2].events = POLLIN;
    pfds[3].fd = hello_timer_fd; pfds[3].events = POLLIN;
    pfds[4].fd = dl_idr_listen_fd(idr_listen);  /* -1 if disabled */
    pfds[4].events = POLLIN;
```

Update the poll call:

```c
        int n = poll(pfds, 5, -1);
```

- [ ] **Step 6: Dispatch the IDR drain**

After the `pfds[3]` handler block and **before** the `while` loop's closing brace, add:

```c
        if (pfds[4].revents & POLLIN) {
            size_t got = dl_idr_listen_drain(idr_listen);
            if (got > 0) {
                dl_log_debug("idr_listen: drained %zu datagram(s)", got);
                dl_backend_enc_request_idr(be, now_monotonic_ms());
            }
        }
```

Find the cleanup section at the bottom of the function (where `listen_fd` is closed). Add:

```c
    dl_idr_listen_close(idr_listen);
```

- [ ] **Step 7: Document in drone.conf.sample**

Edit `conf/drone.conf.sample`. After the existing `min_idr_interval_ms = 500` block (around line 25), add:

```
# ---- PixelPilot IDR-token listener -----------------------------------
# Bind a UDP socket and accept PixelPilot_rk's IDR-request tokens.
# Any datagram arriving on this socket is treated as an IDR request;
# min_idr_interval_ms above governs the actual encoder API rate.
# Set idr_listen_port = 0 to disable (no socket bound).
idr_listen_port = 11223
idr_listen_addr = 0.0.0.0
```

- [ ] **Step 8: Build, confirm everything still compiles**

Run: `nix-shell --run 'make -C drone all test'`
Expected: clean build, all tests still pass. The new dispatch path isn't exercised by unit tests yet; the e2e test in Task 4 covers it.

- [ ] **Step 9: Commit**

```bash
git add drone/src/dl_config.h drone/src/dl_config.c \
        drone/src/dl_applier.c conf/drone.conf.sample \
        tests/drone/test_config.c
git commit -m "$(cat <<'EOF'
drone: wire dl_idr_listen into the applier poll loop

Adds idr_listen_port (default 11223) and idr_listen_addr (default
0.0.0.0) to drone.conf. The applier opens the listener at boot and
drains any inbound datagrams in one poll wake, calling
dl_backend_enc_request_idr() once per drain — the existing
min_idr_interval_ms throttle collapses PixelPilot's 3-packet bursts
to a single encoder API call.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: OSD `I<n>` counter

**Files:**
- Modify: `drone/src/dl_osd.h` (declare `dl_osd_bump_idr`)
- Modify: `drone/src/dl_osd.c` (counter field, bump fn, status-line format)
- Create: `tests/drone/test_osd.c` (renders `I<n>`)
- Modify: `drone/src/dl_applier.c` (call `dl_osd_bump_idr` from the dispatch added in Task 2)
- Modify: `drone/Makefile` (register `test_osd.c`)

- [ ] **Step 1: Write the failing test**

Create `tests/drone/test_osd.c`:

```c
/* test_osd.c — status-line rendering, incl. IDR counter. */
#include "dl_osd.h"
#include "dl_config.h"
#include "dl_wire.h"
#include "test_main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void read_all(const char *path, char *out, size_t outlen) {
    FILE *f = fopen(path, "r");
    if (!f) { out[0] = '\0'; return; }
    size_t n = fread(out, 1, outlen - 1, f);
    out[n] = '\0';
    fclose(f);
}

DL_TEST(test_osd_status_includes_idr_counter) {
    char path[64];
    snprintf(path, sizeof(path), "/tmp/dl-osd-test-%d.msg", (int)getpid());

    dl_config_t cfg = {0};
    cfg.osd_enable = true;
    cfg.osd_debug_latency = false;
    snprintf(cfg.osd_msg_path, sizeof(cfg.osd_msg_path), "%s", path);

    dl_osd_t *o = dl_osd_open(&cfg);
    DL_ASSERT(o != NULL);

    dl_decision_t d = {
        .mcs = 5, .bitrate_kbps = 12000,
        .k = 8, .n = 14, .depth = 2,
        .tx_power_dBm = 18,
    };

    /* Zero counter is rendered as I0. */
    dl_osd_write_status(o, &d, -50);
    char buf[256];
    read_all(path, buf, sizeof(buf));
    DL_ASSERT(strstr(buf, " I0 |") != NULL);

    /* Three bumps → I3. */
    dl_osd_bump_idr(o);
    dl_osd_bump_idr(o);
    dl_osd_bump_idr(o);
    dl_osd_write_status(o, &d, -50);
    read_all(path, buf, sizeof(buf));
    DL_ASSERT(strstr(buf, " I3 |") != NULL);

    dl_osd_close(o);
    unlink(path);
}
```

Register the new file in `drone/Makefile` `TEST_SRCS`:

```make
    $(TESTDIR)/test_osd.c \
```

and add the OSD source (already needed by other tests via include chain; but make it explicit):

```make
    $(SRCDIR)/dl_osd.c \
```

- [ ] **Step 2: Run the test, confirm it fails**

Run: `nix-shell --run 'make -C drone test'`
Expected: compile error — `dl_osd_bump_idr` undeclared.

- [ ] **Step 3: Declare `dl_osd_bump_idr` in the header**

Edit `drone/src/dl_osd.h`. After the existing function declarations and before the convenience-events block, add:

```c
/* Increment the running counter of received IDR requests. Cheap;
 * safe to call from the main poll loop on every drain wake. */
void dl_osd_bump_idr(dl_osd_t *o);
```

- [ ] **Step 4: Implement the counter + format change**

Edit `drone/src/dl_osd.c`.

In `struct dl_osd`, add a new field after `debug_block`:

```c
    uint32_t idr_requests;
```

Add the bump function (after `dl_osd_close`):

```c
void dl_osd_bump_idr(dl_osd_t *o) {
    if (!o) return;
    o->idr_requests++;
}
```

In `dl_osd_write_status`, change the `snprintf` format to include `I%u` before the `|` separator:

```c
    snprintf(o->status_line, sizeof(o->status_line),
             DL_OSD_PREFIX
             "MCS%u %uM (%u,%u)d%u TX%d R%d I%u | &B T&T W&W CPU&C",
             d->mcs,
             (unsigned)((d->bitrate_kbps + 500) / 1000),
             d->k, d->n, d->depth,
             (int)d->tx_power_dBm,
             rssi_dBm,
             (unsigned)o->idr_requests);
```

- [ ] **Step 5: Run the test, confirm it passes**

Run: `nix-shell --run 'make -C drone test'`
Expected: `test_osd_status_includes_idr_counter` passes, all other tests still pass.

- [ ] **Step 6: Bump the counter from the applier**

Edit `drone/src/dl_applier.c`. In the `pfds[4]` dispatch block added in Task 2:

```c
        if (pfds[4].revents & POLLIN) {
            size_t got = dl_idr_listen_drain(idr_listen);
            if (got > 0) {
                dl_log_debug("idr_listen: drained %zu datagram(s)", got);
                dl_osd_bump_idr(osd);
                dl_backend_enc_request_idr(be, now_monotonic_ms());
            }
        }
```

- [ ] **Step 7: Rebuild and confirm tests pass**

Run: `nix-shell --run 'make -C drone all test'`
Expected: clean build, all tests pass.

- [ ] **Step 8: Commit**

```bash
git add drone/src/dl_osd.h drone/src/dl_osd.c \
        drone/src/dl_applier.c \
        tests/drone/test_osd.c drone/Makefile
git commit -m "$(cat <<'EOF'
drone: render IDR request count in OSD status line

Adds a monotonic counter bumped on every poll wake where the new
PixelPilot IDR listener drains at least one datagram. Rendered as
"I<n>" in the existing top-line status string. Operator sees how
many logical IDR requests have arrived since process start without
needing to read journal logs.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Drone E2E test — UDP token triggers encoder IDR

**Files:**
- Modify: `tests/test_drone_e2e.py` (add new tests; keep legacy ones for now — they'll be removed in Task 6 when the wire bit goes away)

- [ ] **Step 1: Write the new e2e tests**

Edit `tests/test_drone_e2e.py`. Find the existing `test_idr_throttle_drops_duplicates` (~line 481) for style reference. Add two new tests after it (do NOT delete the legacy ones yet):

```python
def test_pixelpilot_udp_token_triggers_idr(tmp_path: Path):
    """A single UDP datagram to the applier's IDR port produces
    exactly one /request/idr HTTP call to the mock encoder."""
    with _sandbox(tmp_path) as s:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(b"abc\n", ("127.0.0.1", s["idr_listen_port"]))
        sock.close()
        _wait_for(lambda: len(s["encoder"].recorded) >= 1, timeout=2.0)
        idrs = [p for p in s["encoder"].recorded if p == "/request/idr"]
        assert len(idrs) == 1, s["encoder"].recorded


def test_pixelpilot_udp_burst_collapses_to_one_idr(tmp_path: Path):
    """Three UDP datagrams in quick succession (matching PixelPilot's
    3-packet burst pattern) collapse to one HTTP call, because the
    drone drains them in a single poll wake."""
    with _sandbox(tmp_path, min_idr_interval_ms=500) as s:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        port = s["idr_listen_port"]
        for _ in range(3):
            sock.sendto(b"xyz\n", ("127.0.0.1", port))
        sock.close()
        _wait_for(lambda: len(s["encoder"].recorded) >= 1, timeout=2.0)
        time.sleep(0.3)   # allow any delayed extras to land
        idrs = [p for p in s["encoder"].recorded if p == "/request/idr"]
        assert len(idrs) == 1, s["encoder"].recorded
```

- [ ] **Step 2: Make the sandbox expose the IDR port**

Still in `tests/test_drone_e2e.py`, find `_sandbox` (~line 301). It builds a `drone.conf` dict (around line 335). Add a value for the IDR listen port — pick an ephemeral free port the same way `enc_port` is picked elsewhere in the harness:

```python
        idr_port = _pick_free_port()   # see existing helper used for enc_port
        drone_conf = {
            ...existing keys...,
            "encoder_port":      enc_port,
            ...
            "idr_listen_port":   idr_port,
            "idr_listen_addr":   "127.0.0.1",
            ...
        }
        ...
        yield {
            ...,
            "encoder":          encoder,
            ...,
            "idr_listen_port":  idr_port,
        }
```

If `_pick_free_port` doesn't exist under that name, inline it:

```python
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tmp:
            tmp.bind(("127.0.0.1", 0))
            idr_port = tmp.getsockname()[1]
```

If `_wait_for` doesn't exist as a helper in this file, inline it:

```python
def _wait_for(pred, timeout=2.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return
        time.sleep(interval)
    raise AssertionError(f"timed out waiting for {pred}")
```

Add imports if missing: `import socket`, `import time`.

- [ ] **Step 3: Run the new tests**

Run: `nix-shell --run 'python3 -m pytest tests/test_drone_e2e.py::test_pixelpilot_udp_token_triggers_idr tests/test_drone_e2e.py::test_pixelpilot_udp_burst_collapses_to_one_idr -v --ignore=tests/test_mavlink_status.py'`
Expected: both pass.

- [ ] **Step 4: Run the full drone e2e suite, confirm nothing else broke**

Run: `nix-shell --run 'python3 -m pytest tests/test_drone_e2e.py -v --ignore=tests/test_mavlink_status.py'`
Expected: all existing tests still pass — we have not yet removed the wire bit, so `test_idr_throttle_drops_duplicates` etc. still work.

- [ ] **Step 5: Commit**

```bash
git add tests/test_drone_e2e.py
git commit -m "$(cat <<'EOF'
test: drone e2e for PixelPilot UDP IDR listener

Single-datagram and burst-collapse cases against a real dl-applier
+ mock encoder HTTP server. Verifies the new UDP listener path
end-to-end. Legacy idr_request-flag tests are intentionally left in
place; they will be removed alongside the wire bit in a later
commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Remove the C-side wire bit, dispatch sites, and `dl-inject --idr`

**Files:**
- Modify: `drone/src/dl_wire.h` (remove `DL_FLAG_IDR_REQUEST`)
- Modify: `drone/src/dl_applier.c` (remove the three dispatch sites)
- Modify: `drone/src/dl_inject.c` (remove `--idr` CLI)
- Modify: `tests/drone/test_wire.c` (remove the flag setter + assertion)
- Modify: `tests/test_wire_contract.py` (drop `idr_request=True` and `idr_request=False` cases, since the Python-side `idr_request` kwarg still exists at this point but the C side no longer encodes anything for it)

- [ ] **Step 1: Drop `DL_FLAG_IDR_REQUEST` from the wire header**

Edit `drone/src/dl_wire.h`. Remove lines 35–36:

```c
/* flag bits (decision packet) */
#define DL_FLAG_IDR_REQUEST 0x01u
```

Replace with a single comment line so future readers know the byte is preserved:

```c
/* flag bits (decision packet) — currently unused; reserved zero. */
```

- [ ] **Step 2: Drop the three dispatch sites in `dl_applier.c`**

Edit `drone/src/dl_applier.c`.

Remove lines 391–393 (inside the EQUAL/first-apply branch):

```c
                        if (d.flags & DL_FLAG_IDR_REQUEST) {
                            dl_backend_enc_request_idr(be, now);
                        }
```

Remove lines 415–417 (inside the DOWN branch):

```c
                        if (d.flags & DL_FLAG_IDR_REQUEST) {
                            dl_backend_enc_request_idr(be, now);
                        }
```

Remove lines 488–490 (inside the UP-gap fire):

```c
                if (apply_pending.flags & DL_FLAG_IDR_REQUEST) {
                    dl_backend_enc_request_idr(be, now_monotonic_ms());
                }
```

- [ ] **Step 3: Drop `--idr` from `dl-inject`**

Edit `drone/src/dl_inject.c`.

Remove `--idr` from the usage string (line 24):

```c
"         [--idr] [--sequence N]\n"
```

becomes:

```c
"         [--sequence N]\n"
```

Remove the longopt entry (line 58):

```c
{ "idr",       no_argument,       0, 'I' },
```

Remove the case branch (line 115):

```c
case 'I': d.flags |= DL_FLAG_IDR_REQUEST; break;
```

In the dry-run printout (line 272), remove the IDR conditional:

```c
(d.flags & DL_FLAG_IDR_REQUEST) ? " IDR" : "",
```

and clean up the surrounding format string + argument list so the printout no longer mentions IDR.

- [ ] **Step 4: Drop the flag from the C wire test**

Edit `tests/drone/test_wire.c`.

Line 9 — change:

```c
.flags = DL_FLAG_IDR_REQUEST,
```

to:

```c
.flags = 0,
```

Line 38 — delete the line:

```c
DL_ASSERT_EQ(r.flags & DL_FLAG_IDR_REQUEST, DL_FLAG_IDR_REQUEST);
```

Add a single replacement assertion that the flags byte round-trips zero:

```c
DL_ASSERT_EQ(r.flags, 0);
```

- [ ] **Step 5: Drop IDR cases from the Python wire-contract test**

Edit `tests/test_wire_contract.py`.

Line 58: remove `idr_request=False,` from the baseline fixture (the Python-side kwarg still exists — we're just not exercising it).

Lines 85–90 or so: locate the fixture case constructed with `idr_request=True` and delete that entire test case (the C side can no longer produce a flag bit, so the contract diff would fail).

- [ ] **Step 6: Build and run all drone tests**

Run: `nix-shell --run 'make -C drone all test'`
Expected: clean build, all C unit tests pass.

- [ ] **Step 7: Run the Python wire-contract test + drone e2e**

Run: `nix-shell --run 'python3 -m pytest tests/test_wire_contract.py tests/test_drone_e2e.py -v --ignore=tests/test_mavlink_status.py'`
Expected: all pass. The new PixelPilot UDP tests still work; the legacy `test_idr_throttle_drops_duplicates` and `test_idr_burst_delivers_multiple_packets` will now fail because they send `idr=True` decision packets that the drone no longer dispatches. **That is expected** — those tests will be removed in Task 6 alongside the Python-side cleanup. To unblock this task's commit:

Mark the two failing legacy tests with `@pytest.mark.skip(reason="removed in Task 6")`:

In `tests/test_drone_e2e.py` add the decorator above the affected `def test_idr_throttle_drops_duplicates` (~line 481), `async def test_idr_burst_delivers_multiple_packets` (~line 496), and `def test_osd_debug_latency_idr_throttle_not_counted` (~line 885).

Re-run the e2e suite:
Run: `nix-shell --run 'python3 -m pytest tests/test_drone_e2e.py -v --ignore=tests/test_mavlink_status.py'`
Expected: all non-skipped tests pass, three skips.

- [ ] **Step 8: Commit**

```bash
git add drone/src/dl_wire.h drone/src/dl_applier.c drone/src/dl_inject.c \
        tests/drone/test_wire.c tests/test_wire_contract.py \
        tests/test_drone_e2e.py
git commit -m "$(cat <<'EOF'
drone: remove DL_FLAG_IDR_REQUEST + dispatch sites + dl-inject flag

C side stops emitting and consuming the IDR wire bit. The three
dispatch sites in dl_applier.c (direct-apply, down-gap, up-gap)
are gone; dl-inject's --idr flag is removed; the round-trip C test
no longer sets the flag; the Python wire-contract test drops its
idr_request=True case.

Skips three legacy IDR-flag e2e tests pending their removal in the
next commit (when Decision.idr_request and the GS-side burster go
away).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Remove the Python-side wire bit, policy trigger, burster, and all `idr_request` consumers

This is the largest single commit in the plan — it is intentionally atomic because every change is coupled to the removal of `Decision.idr_request`. Touching one file without the others leaves the suite broken.

**Files:**
- Modify: `gs/dynamic_link/wire.py`
- Modify: `gs/dynamic_link/decision.py`
- Modify: `gs/dynamic_link/policy.py`
- Modify: `gs/dynamic_link/service.py`
- Modify: `gs/dynamic_link/flight_log.py`
- Delete: `gs/dynamic_link/idr_burst.py`
- Delete: `tests/test_idr_burst.py`
- Modify: `tests/test_drone_e2e.py` (delete the three skipped tests from Task 5; remove `idr=True` lines from any other test helpers)
- Modify: `tests/test_wire_contract.py` (drop the `idr_request` kwarg from `Encoder.encode()` calls and any remaining fixtures)
- Modify: `tests/test_phase2_e2e.py` (delete `test_gs_wire_idr_flag_triggers_idr_request`; drop `idr_request=False` from helper)
- Modify: `tests/test_flight_log.py` (drop `idr_request=False` at line 258)
- Modify: `tests/test_dl_review.py` (drop `idr_request` at lines 31, 38, 42)
- Modify: `tests/test_dl_replay.py` (drop `idr_request` at line 27)
- Modify: `tests/test_phase3_e2e.py` (drop `idr_request` at line 71)
- Modify: `tests/bench/per_knob_loss_bench.py` (drop `idr_request` at line 134)
- Modify: `gs/tools/dl_replay.py` (drop the line that reads `rec["idr_request"]`)
- Modify: `gs/tools/dl_report.py` (drop the `summary["idr_requests"]` block at lines 189–193)
- Modify: `conf/gs.yaml.sample` (drop any `idr_burst:` block)

- [ ] **Step 1: Remove `FLAG_IDR_REQUEST` and the kwarg from `wire.py`**

Edit `gs/dynamic_link/wire.py`.

Delete the constant (line 41):

```python
FLAG_IDR_REQUEST = 0x01
```

In `Encoder.encode` (line 78+), remove the `idr_request` parameter from the signature, remove the docstring sentence describing it (lines ~88–95), and remove the body block:

```python
        if idr_request is None:
            idr_request = bool(decision.idr_request)
        ...
        flags = FLAG_IDR_REQUEST if idr_request else 0
```

Replace the flags computation with the literal:

```python
        flags = 0
```

- [ ] **Step 2: Remove `idr_request` from `Decision`**

Edit `gs/dynamic_link/decision.py`. Delete line 27:

```python
    idr_request: bool
```

- [ ] **Step 3: Remove IDR from `TrailingLoop`**

Edit `gs/dynamic_link/policy.py`.

Change `TrailingLoop.tick()`'s return type and behavior:

- Update the docstring (lines 585–589): drop the "(depth, idr_request)" return mention; explain it returns depth only.
- In the vanilla branch (lines 597–603), remove the `idr = ...`, the reason append, and change `return 1, idr` to `return 1`.
- In the main branch, delete `idr = False` at line 622, and the entire `if had_loss: idr = True; self._reasons.append(...)` block at lines 624–633. Note `had_loss` is still needed for the depth bootstrap logic that follows; do not remove it.
- Change the final return (line 692) from `return new_depth, idr` to `return new_depth`.

In `Policy.tick()`:

- Line 896: change `new_depth, idr = self.trailing.tick(...)` to `new_depth = self.trailing.tick(...)`.
- Lines 949–950: delete the `if idr: knobs_changed.append("idr")` block.
- Wherever `Policy.tick()` constructs the outgoing `Decision`, remove `idr_request=idr` from the call. The `idr` local is now dead and can be deleted.

- [ ] **Step 4: Remove the burster from `service.py`**

Edit `gs/dynamic_link/service.py`.

Line 32: remove `from .idr_burst import IdrBurstConfig, IdrBurster`.
Line 454: remove `idr_burster: IdrBurster | None = None`.
Line 456: remove the `idr_burster = IdrBurster(...)` assignment.
Lines 574–575: remove

```python
                if decision.idr_request and idr_burster is not None:
                    idr_burster.trigger(decision)
```

If any other reference to `idr_burst` / `IdrBurstConfig` / `IdrBurster` remains in this file (`grep -n idr service.py`), delete it.

- [ ] **Step 5: Remove `idr_request` from the flight log**

Edit `gs/dynamic_link/flight_log.py`. Find the per-tick record construction (search for `"idr_request"`) and drop that key. Old log files still contain the field; the JSONL schema is forward-only by convention.

- [ ] **Step 6: Delete `idr_burst.py` and its test**

```bash
rm gs/dynamic_link/idr_burst.py tests/test_idr_burst.py
```

- [ ] **Step 7: Sweep all remaining `idr_request=` fixture references**

```bash
grep -rn 'idr_request' gs/ tests/ conf/ docs/
```

For each remaining hit, delete the `idr_request=<bool>` line from `Decision(...)` constructors:

- `tests/test_drone_e2e.py`: delete the three `@pytest.mark.skip` tests added in Task 5 (`test_idr_throttle_drops_duplicates`, `test_idr_burst_delivers_multiple_packets`, `test_osd_debug_latency_idr_throttle_not_counted`) and remove any other `idr_request=` lines (helpers, fixture builders).
- `tests/test_wire_contract.py`: remove the `idr_request=False` kwarg from any remaining `encode(...)` calls. The kwarg no longer exists.
- `tests/test_phase2_e2e.py`: delete `test_gs_wire_idr_flag_triggers_idr_request` (line 68+). Drop `idr_request=False` from the helper at line 36.
- `tests/test_flight_log.py`: drop `idr_request=False` at line 258.
- `tests/test_dl_review.py`: drop the field at lines 31, 38, 42.
- `tests/test_dl_replay.py`: drop the field at line 27.
- `tests/test_phase3_e2e.py`: drop the field at line 71.
- `tests/bench/per_knob_loss_bench.py`: drop the field at line 134.

- [ ] **Step 8: Drop the field from the GS tools**

Edit `gs/tools/dl_replay.py`. Remove line 70:

```python
        idr_request=bool(rec["idr_request"]),
```

If the surrounding `Decision(...)` constructor was multi-line, leave the closing parenthesis correctly aligned.

Edit `gs/tools/dl_report.py`. Remove lines 189–193:

```python
        summary["idr_requests"] = [
            r["seq"]
            for r in streams.verbose if r.get("idr_request")
        ]
```

(Exact slice may differ — find the `summary["idr_requests"]` assignment and delete the whole expression. No replacement needed; the key simply goes away.)

- [ ] **Step 9: Drop any `idr_burst:` block from gs.yaml.sample**

```bash
grep -n 'idr_burst\|idr_request' conf/gs.yaml.sample
```

If a `idr_burst:` block exists (count, interval_ms), delete it. If there's no such block, this step is a no-op — leave the file untouched.

- [ ] **Step 10: Run the full Python suite**

Run: `nix-shell --run 'python3 -m pytest --ignore=tests/test_mavlink_status.py -v'`
Expected: all tests pass. Any failure here is almost certainly a missed `idr_request=` reference — run `grep -rn 'idr_request\|FLAG_IDR_REQUEST\|IdrBurster\|idr_burst' gs/ tests/` and clean up.

- [ ] **Step 11: Run the C suite to confirm nothing slipped**

Run: `nix-shell --run 'make -C drone test'`
Expected: all pass.

- [ ] **Step 12: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
gs: remove dl IDR path — FLAG_IDR_REQUEST, burster, policy trigger

Atomic Python-side cleanup: deletes Decision.idr_request, the
FLAG_IDR_REQUEST wire bit + idr_request kwarg from Encoder.encode,
TrailingLoop's IDR signalling (return type now int instead of
(int, bool)), service.py's burster wiring, the idr_request field
in flight_log JSONL, and the dl_replay/dl_report consumers.
idr_burst.py and tests/test_idr_burst.py are deleted outright.

PixelPilot's UDP-token listener (added in earlier commits) is now
the sole IDR-request path to the encoder.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Documentation update

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Update the IDR section of the README**

Edit `README.md`. Find any section mentioning IDR (likely under "What dl does" or "Architecture"). Replace the description of GS-driven IDR signalling with text along these lines (write to match the surrounding tone — terse and operator-facing):

```markdown
### IDR (keyframe) requests

dynamic-link does not generate IDR requests on its own. Instead,
`dl-applier` binds a UDP socket on port **11223** (configurable via
`idr_listen_port` in `drone.conf`) and accepts the IDR-token
datagrams that PixelPilot_rk's RTP receiver sends when it detects a
sequence gap or decode stall. Any datagram on this port produces one
`GET /request/idr` call to the local encoder, rate-limited by
`min_idr_interval_ms`.

If you are running a GS video player other than PixelPilot_rk
(gstreamer, QGC, ffplay), IDR requests will not be sent.
PixelPilot's burst parameters (3 packets × 100 ms, 500 ms gap
cooldown, 700 ms decode-stall cooldown) are hardcoded in
`gstrtpreceiver.cpp:118–133` — retuning requires forking PixelPilot.
```

If the README has an architecture diagram or a "what gets installed where" table, update it to mention the new UDP port.

- [ ] **Step 2: Verify there are no stale references elsewhere in `docs/`**

```bash
grep -rn 'idr_request\|FLAG_IDR_REQUEST\|IdrBurster\|idr_burst' docs/
```

For each hit, decide:
- **Spec/plan files** under `docs/superpowers/specs/` and `docs/superpowers/plans/`: these are historical artifacts. Leave them alone — they document past decisions accurately for their date.
- **Operator-facing docs** (anything else): rewrite to match the new reality, same way as the README block above.

- [ ] **Step 3: Run both test suites one more time as a final gate**

Run in parallel:

```bash
nix-shell --run 'python3 -m pytest --ignore=tests/test_mavlink_status.py'
nix-shell --run 'make -C drone test'
```

Expected: both pass.

- [ ] **Step 4: Commit**

```bash
git add README.md docs/
git commit -m "$(cat <<'EOF'
docs: document the PixelPilot UDP IDR listener

README's IDR section now reflects reality: dl-applier listens on
UDP 11223 for PixelPilot-originated IDR tokens; no GS-driven IDR
path exists anymore. Calls out the burst parameters as hardcoded
in PixelPilot, the encoder-throttle floor, and the non-PixelPilot-
player caveat.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Verification checklist (run before declaring done)

After Task 7, run:

```bash
nix-shell --run 'python3 -m pytest --ignore=tests/test_mavlink_status.py'
nix-shell --run 'make -C drone all test'
grep -rn 'idr_request\|FLAG_IDR_REQUEST\|IdrBurster\|idr_burst' gs/ tests/ drone/src/ conf/
```

- Both test suites pass.
- The grep finds **zero** hits outside `docs/superpowers/`.
- The OSD status line on a running drone shows `I0` initially and increments each time PixelPilot triggers an IDR.
- `dl-applier --help` (or the binary's startup log) shows the new `idr_listen_port` and `idr_listen_addr` config keys.
