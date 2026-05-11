# Drop drone-side ceiling config — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the seven per-airframe ceiling knobs in `drone.conf`
(`video_k_min`, `video_k_max`, `video_n_max`, `depth_max`, `mcs_max`,
`tx_power_min_dBm`, `tx_power_max_dBm`) with `#define` constants in
`drone/src/dl_ceiling.h`. The drone applier's failsafe-4 check still
runs; only the configurability goes away. `safe_defaults` stays
configurable.

**Architecture:** `dl_ceiling_check` loses its `cfg` argument and
reads `DL_BOUND_*` constants instead. `dl_config_t` loses the seven
fields; the parser hard-fails with a deprecation message if any of the
removed keys appear in `drone.conf`. Tests, sample config, and design
docs are updated in lockstep.

**Tech Stack:** C11 (drone), pytest (Python e2e). No new
dependencies.

**Spec:** `docs/superpowers/specs/2026-05-11-drop-drone-ceiling-config-design.md`

---

## File map

**Modified:**
- `drone/src/dl_ceiling.h` — add `DL_BOUND_*` constants, change `dl_ceiling_check` signature.
- `drone/src/dl_ceiling.c` — read constants instead of `cfg`.
- `drone/src/dl_config.h` — remove 7 fields, update validate() comment.
- `drone/src/dl_config.c` — drop defaults init, parser branches, cross-field validation; add deprecation arms; simplify `dl_config_validate` to compare `safe_*` against `DL_BOUND_*`.
- `drone/src/dl_applier.c` — drop `&cfg` from `dl_ceiling_check` call.
- `tests/drone/test_ceiling.c` — rewrite against new signature + constants.
- `tests/drone/test_config.c` — replace ceiling-vs-safe tests with deprecation tests; rework `test_config_parses_good_file` to drop removed keys.
- `tests/test_drone_e2e.py` — drop the 7 keys from synthetic `drone.conf`; switch `test_ceiling_rejects_out_of_bound_mcs` and `test_ceiling_reject_emits_statustext` to inject `mcs=8` (no override needed).
- `tests/test_phase3_e2e.py` — drop the 7 keys from synthetic `drone.conf`.
- `conf/drone.conf.sample` — remove ceiling block; one-line note.
- `docs/dynamic-link-design.md` — §4/§6 ceiling references rewritten; example `drone.conf` block trimmed.
- `docs/phase1-implementation.md` — append change note.
- `README.md` — remove ceiling references if any.

**Unmodified (verified):**
- `drone/src/dl_inject.c` — wire-encoder CLI; doesn't touch ceiling.

---

## Task 1: Add `DL_BOUND_*` constants to `dl_ceiling.h`

Header-only addition. No behavior change yet. Lets later tasks
reference the constants by name.

**Files:**
- Modify: `drone/src/dl_ceiling.h`

- [ ] **Step 1: Add the constants block to the header**

Edit `drone/src/dl_ceiling.h`. After the `#include "dl_wire.h"` line
(line 9), insert:

```c
/* Hardcoded bounds for failsafe 4. Sources:
 *   DEPTH_MAX  — wfb-ng MAX_INTERLEAVE_DEPTH (src/rx.hpp:103)
 *   N_MAX      — wfb-ng src/tx.cpp:1183 (depth>1 limit; conservative
 *                for depth=1 too)
 *   MCS_MAX    — single spatial stream; design §4.1 policy
 *   K_MIN      — math sanity (k >= 1, k <= n)
 *   TX_POWER_* — sanity range; iw rejects what the radio can't do.
 */
#define DL_BOUND_K_MIN              1
#define DL_BOUND_N_MAX              32
#define DL_BOUND_DEPTH_MAX          8
#define DL_BOUND_MCS_MAX            7
#define DL_BOUND_TX_POWER_MIN_DBM (-10)
#define DL_BOUND_TX_POWER_MAX_DBM   30
```

- [ ] **Step 2: Verify the header still compiles cleanly**

Run: `make -C drone`
Expected: clean build, no warnings (the constants are unused so
far, but `#define` doesn't warn).

- [ ] **Step 3: Commit**

```bash
git add drone/src/dl_ceiling.h
git commit -m "$(cat <<'EOF'
ceiling: add DL_BOUND_* constants for hardcoded failsafe bounds

Header-only addition; not yet referenced. Followups switch
dl_ceiling_check off the per-airframe config and onto these.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Switch `dl_ceiling_check` to constants; update call site

Atomic refactor: signature change, body change, single call site,
test rewrite — all coupled.

**Files:**
- Modify: `drone/src/dl_ceiling.h`
- Modify: `drone/src/dl_ceiling.c`
- Modify: `drone/src/dl_applier.c` (single call site)
- Modify: `tests/drone/test_ceiling.c` (full rewrite)

- [ ] **Step 1: Rewrite `tests/drone/test_ceiling.c`**

Replace the entire file with:

```c
/* test_ceiling.c — failsafe 4 rejection logic against hardcoded bounds. */
#include "test_main.h"
#include "dl_ceiling.h"

static dl_decision_t default_dec(void) {
    dl_decision_t d = {
        .mcs = 5, .bandwidth = 20, .tx_power_dBm = 15,
        .k = 4, .n = 8, .depth = 1, .bitrate_kbps = 10000,
    };
    return d;
}

DL_TEST(test_ceiling_accepts_default) {
    dl_decision_t d = default_dec();
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_OK);
}

DL_TEST(test_ceiling_rejects_k_below_min) {
    dl_decision_t d = default_dec();
    d.k = DL_BOUND_K_MIN - 1;  /* 0 */
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_K_OUT_OF_RANGE);
}

DL_TEST(test_ceiling_rejects_k_above_n) {
    dl_decision_t d = default_dec();
    d.k = 9; d.n = 8;  /* k > n */
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_K_OUT_OF_RANGE);
}

DL_TEST(test_ceiling_rejects_n_above_max) {
    dl_decision_t d = default_dec();
    d.n = DL_BOUND_N_MAX + 1;  /* 33 */
    /* k must remain <= n to isolate the n check; bump k down. */
    d.k = 4;
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_N_TOO_LARGE);
}

DL_TEST(test_ceiling_rejects_depth_above_max) {
    dl_decision_t d = default_dec();
    d.depth = DL_BOUND_DEPTH_MAX + 1;  /* 9 */
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_DEPTH_TOO_LARGE);
}

DL_TEST(test_ceiling_rejects_mcs_above_cap) {
    dl_decision_t d = default_dec();
    d.mcs = DL_BOUND_MCS_MAX + 1;  /* 8 */
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_MCS_TOO_HIGH);
}

DL_TEST(test_ceiling_rejects_bandwidth_80) {
    dl_decision_t d = default_dec();
    d.bandwidth = 80;
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_BANDWIDTH_BAD);
}

DL_TEST(test_ceiling_rejects_tx_power_too_high) {
    dl_decision_t d = default_dec();
    d.tx_power_dBm = DL_BOUND_TX_POWER_MAX_DBM + 1;  /* 31 */
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_TX_POWER_OUT_OF_RANGE);
}

DL_TEST(test_ceiling_rejects_tx_power_too_low) {
    dl_decision_t d = default_dec();
    d.tx_power_dBm = DL_BOUND_TX_POWER_MIN_DBM - 1;  /* -11 */
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_TX_POWER_OUT_OF_RANGE);
}

DL_TEST(test_ceiling_accepts_at_max_bounds) {
    dl_decision_t d = default_dec();
    d.mcs = DL_BOUND_MCS_MAX;
    d.depth = DL_BOUND_DEPTH_MAX;
    d.k = 4;
    d.n = DL_BOUND_N_MAX;
    d.tx_power_dBm = DL_BOUND_TX_POWER_MAX_DBM;
    DL_ASSERT_EQ(dl_ceiling_check(&d), DL_CEILING_OK);
}
```

Note: `DL_CEILING_DEPTH_N_CONFLICT` is no longer reachable (n ≤ 32
is now flat); test for it removed.

- [ ] **Step 2: Run tests; expect compile failure**

Run: `make -C drone test`
Expected: build fails — either `dl_ceiling_check` arity mismatch
in `test_ceiling.c` (new signature not yet in header) or unresolved
references. This is the failing-test state.

- [ ] **Step 3: Update `drone/src/dl_ceiling.h` signature**

Change the function declaration. Replace lines 25–26
(`dl_ceiling_result_t dl_ceiling_check(const dl_decision_t *d, const dl_config_t *cfg);`)
with:

```c
dl_ceiling_result_t dl_ceiling_check(const dl_decision_t *d);
```

Also remove the now-unneeded include — replace the `#include "dl_config.h"`
line (line 8) with a blank line (or just delete the line; the header
no longer needs the config type).

Also remove `DL_CEILING_DEPTH_N_CONFLICT` from the enum (line 19)
and from `dl_ceiling_reason` in the .c file in the next step. The
flat `n ≤ 32` rule subsumes it.

- [ ] **Step 4: Rewrite `drone/src/dl_ceiling.c` body**

Replace the file with:

```c
/* dl_ceiling.c */
#include "dl_ceiling.h"
#include "dl_log.h"

const char *dl_ceiling_reason(dl_ceiling_result_t r) {
    switch (r) {
        case DL_CEILING_OK:                    return "ok";
        case DL_CEILING_K_OUT_OF_RANGE:        return "k_out_of_range";
        case DL_CEILING_N_TOO_LARGE:           return "n_too_large";
        case DL_CEILING_DEPTH_TOO_LARGE:       return "depth_too_large";
        case DL_CEILING_MCS_TOO_HIGH:          return "mcs_too_high";
        case DL_CEILING_BANDWIDTH_BAD:         return "bandwidth_bad";
        case DL_CEILING_TX_POWER_OUT_OF_RANGE: return "tx_power_out_of_range";
    }
    return "unknown";
}

dl_ceiling_result_t dl_ceiling_check(const dl_decision_t *d) {
    if (d->k < DL_BOUND_K_MIN || d->k > d->n) {
        dl_log_warn("ceiling: k=%u outside [%d, n=%u]",
                    d->k, DL_BOUND_K_MIN, d->n);
        return DL_CEILING_K_OUT_OF_RANGE;
    }
    if (d->n > DL_BOUND_N_MAX) {
        dl_log_warn("ceiling: n=%u > %d", d->n, DL_BOUND_N_MAX);
        return DL_CEILING_N_TOO_LARGE;
    }
    if (d->depth > DL_BOUND_DEPTH_MAX) {
        dl_log_warn("ceiling: depth=%u > %d", d->depth, DL_BOUND_DEPTH_MAX);
        return DL_CEILING_DEPTH_TOO_LARGE;
    }
    if (d->mcs > DL_BOUND_MCS_MAX) {
        dl_log_warn("ceiling: mcs=%u > %d", d->mcs, DL_BOUND_MCS_MAX);
        return DL_CEILING_MCS_TOO_HIGH;
    }
    if (d->bandwidth != 20 && d->bandwidth != 40) {
        dl_log_warn("ceiling: bandwidth=%u not in {20, 40}", d->bandwidth);
        return DL_CEILING_BANDWIDTH_BAD;
    }
    if (d->tx_power_dBm < DL_BOUND_TX_POWER_MIN_DBM ||
        d->tx_power_dBm > DL_BOUND_TX_POWER_MAX_DBM) {
        dl_log_warn("ceiling: tx_power_dBm=%d outside [%d, %d]",
                    d->tx_power_dBm,
                    DL_BOUND_TX_POWER_MIN_DBM, DL_BOUND_TX_POWER_MAX_DBM);
        return DL_CEILING_TX_POWER_OUT_OF_RANGE;
    }
    return DL_CEILING_OK;
}
```

- [ ] **Step 5: Update the single call site in `drone/src/dl_applier.c`**

Find the line containing `dl_ceiling_check(&d, &cfg)` (around line
348). Replace with `dl_ceiling_check(&d)`.

- [ ] **Step 6: Run tests; expect pass**

Run: `make -C drone test`
Expected: all C tests pass, including the rewritten ceiling
tests. Build is clean.

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_ceiling.h drone/src/dl_ceiling.c \
        drone/src/dl_applier.c tests/drone/test_ceiling.c
git commit -m "$(cat <<'EOF'
ceiling: drop cfg arg; read DL_BOUND_* constants directly

dl_ceiling_check no longer takes a dl_config_t. Bounds come from
DL_BOUND_* defines in dl_ceiling.h (wfb-ng / hardware limits).
DEPTH_N_CONFLICT result removed — flat n<=32 subsumes it.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Remove ceiling fields + add deprecation arms in `dl_config`

**Files:**
- Modify: `drone/src/dl_config.h`
- Modify: `drone/src/dl_config.c`
- Modify: `tests/drone/test_config.c`

- [ ] **Step 1: Update `tests/drone/test_config.c` ceiling tests**

Apply the following edits.

(a) In `test_config_defaults_are_sane` (lines 21–30), remove the
two ceiling assertions (`c.depth_max == 3` and `c.video_k_max == 8`)
since the fields no longer exist. Replace with one line keeping
something the operator-tunable fields should still satisfy. The
final body:

```c
DL_TEST(test_config_defaults_are_sane) {
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_STR_EQ(c.listen_addr, "0.0.0.0");
    DL_ASSERT_EQ(c.listen_port, 5800);
    DL_ASSERT_EQ(c.safe_k, 8);
    DL_ASSERT_EQ(c.health_timeout_ms, 10000);
    DL_ASSERT(dl_config_validate(&c) == 0);
}
```

(b) In `test_config_parses_good_file` (lines 32–62), drop the five
removed lines from `body` (`video_k_min`, `video_k_max`,
`video_n_max`, `depth_max`, `mcs_max`, `tx_power_max_dBm`). New
body:

```c
    const char *body =
        "# sample\n"
        "listen_addr = 127.0.0.1\n"
        "listen_port = 5900\n"
        "wfb_tx_ctrl_port = 8010\n"
        "encoder_kind = waybeam\n"
        "encoder_port = 8080\n"
        "osd_enable = false\n";
```

(c) Delete `test_config_rejects_depth_above_hw_limit`,
`test_config_rejects_depth2_with_n_over_32`,
`test_config_rejects_safe_k_above_ceiling`, and
`test_config_rejects_safe_tx_power_above_ceiling` (lines 64–93).
The first two are now compile-time invariants of the constants;
the last two get replaced by hardcoded-bound checks below.

(d) Add new tests immediately after `test_config_defaults_are_sane`:

```c
DL_TEST(test_config_rejects_safe_k_above_hardcoded_bound) {
    /* safe_k must satisfy DL_BOUND_K_MIN <= safe_k. */
    dl_config_t c;
    dl_config_defaults(&c);
    c.safe_k = 0;
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_rejects_safe_n_above_hardcoded_bound) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.safe_n = DL_BOUND_N_MAX + 1;
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_rejects_safe_depth_above_hardcoded_bound) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.safe_depth = DL_BOUND_DEPTH_MAX + 1;
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_rejects_safe_mcs_above_hardcoded_bound) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.safe_mcs = DL_BOUND_MCS_MAX + 1;
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_rejects_safe_tx_power_above_hardcoded_bound) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.safe_tx_power_dBm = DL_BOUND_TX_POWER_MAX_DBM + 1;
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_rejects_removed_ceiling_key) {
    /* Any of the seven removed keys must cause a load failure. */
    const char *body = "mcs_max = 7\n";
    char path[64];
    DL_ASSERT_EQ(write_tmp(body, path, sizeof(path)), 0);
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_load(path, &c), -1);
    unlink(path);
}
```

The new tests need the `DL_BOUND_*` constants — add
`#include "dl_ceiling.h"` at the top of `test_config.c` (after the
existing `#include "dl_config.h"` on line 3).

- [ ] **Step 2: Run tests; expect compile/test failures**

Run: `make -C drone test`
Expected: build fails. The fields `c.depth_max`, `c.video_k_max`
etc. are still present, so the deleted assertions wouldn't have
broken compile, but the *new* deprecation test would currently
**pass** the load (parser accepts `mcs_max = 7` today, returning
0). And `c.safe_k = 0` may already validate today (lo bound is 1
in the parser but `dl_config_validate` doesn't currently catch it
— it would still pass). At minimum, the deprecation test must
fail.

- [ ] **Step 3: Update `drone/src/dl_config.h`**

Remove lines 23–32 (the `Per-airframe ceiling` block and the
`Regulatory / hardware TX power bounds` block — seven `uint8_t` /
`int8_t` fields). The block to delete:

```c
    /* Per-airframe ceiling (failsafe 4). */
    uint8_t  video_k_min;
    uint8_t  video_k_max;
    uint8_t  video_n_max;
    uint8_t  depth_max;
    uint8_t  mcs_max;

    /* Regulatory / hardware TX power bounds. */
    int8_t   tx_power_min_dBm;
    int8_t   tx_power_max_dBm;
```

Update the `dl_config_validate` docstring (lines 108–111) to:

```c
/* Validate that safe_defaults satisfy the hardcoded bounds in
 * dl_ceiling.h. Returns 0 on success, -1 on violation (logged). */
int dl_config_validate(const dl_config_t *cfg);
```

- [ ] **Step 4: Update `drone/src/dl_config.c`**

(a) Add `#include "dl_ceiling.h"` near the top (after
`#include "dl_log.h"` on line 3).

(b) Delete the two `#define WFB_*` lines (12–13). They're now
encoded by `DL_BOUND_*` constants.

(c) In `dl_config_defaults` (lines 22–29), delete the seven
ceiling default assignments:

```c
    cfg->video_k_min = 2;
    cfg->video_k_max = 8;
    cfg->video_n_max = 16;
    cfg->depth_max   = 3;
    cfg->mcs_max     = 7;

    cfg->tx_power_min_dBm = 0;
    cfg->tx_power_max_dBm = 20;
```

(d) Replace the seven parser branches (lines 162–168) with
deprecation arms. Replace this block:

```c
        else if (strcmp(key, "video_k_min") == 0)        SET_INT_RANGED(video_k_min, uint8_t, 1, 32);
        else if (strcmp(key, "video_k_max") == 0)        SET_INT_RANGED(video_k_max, uint8_t, 1, 32);
        else if (strcmp(key, "video_n_max") == 0)        SET_INT_RANGED(video_n_max, uint8_t, 2, 255);
        else if (strcmp(key, "depth_max") == 0)          SET_INT_RANGED(depth_max, uint8_t, 1, 8);
        else if (strcmp(key, "mcs_max") == 0)            SET_INT_RANGED(mcs_max, uint8_t, 0, 7);
        else if (strcmp(key, "tx_power_min_dBm") == 0)   SET_INT_RANGED(tx_power_min_dBm, int8_t, -10, 30);
        else if (strcmp(key, "tx_power_max_dBm") == 0)   SET_INT_RANGED(tx_power_max_dBm, int8_t, -10, 30);
```

with:

```c
        else if (strcmp(key, "video_k_min") == 0 ||
                 strcmp(key, "video_k_max") == 0 ||
                 strcmp(key, "video_n_max") == 0 ||
                 strcmp(key, "depth_max") == 0 ||
                 strcmp(key, "mcs_max") == 0 ||
                 strcmp(key, "tx_power_min_dBm") == 0 ||
                 strcmp(key, "tx_power_max_dBm") == 0) {
            dl_log_err("%s:%d: %s is no longer supported "
                       "(removed 2026-05-11); bounds are hardcoded in "
                       "dl_ceiling.h", path, lineno, key);
            rc = -1;
            continue;
        }
```

(e) Replace the entire body of `dl_config_validate` (lines 208–260)
with:

```c
int dl_config_validate(const dl_config_t *cfg) {
    int rc = 0;

    if (cfg->safe_k < DL_BOUND_K_MIN || cfg->safe_k > cfg->safe_n) {
        dl_log_err("config: safe_k=%u outside [%d, safe_n=%u]",
                   cfg->safe_k, DL_BOUND_K_MIN, cfg->safe_n);
        rc = -1;
    }
    if (cfg->safe_n > DL_BOUND_N_MAX) {
        dl_log_err("config: safe_n=%u > %d",
                   cfg->safe_n, DL_BOUND_N_MAX);
        rc = -1;
    }
    if (cfg->safe_depth > DL_BOUND_DEPTH_MAX) {
        dl_log_err("config: safe_depth=%u > %d",
                   cfg->safe_depth, DL_BOUND_DEPTH_MAX);
        rc = -1;
    }
    if (cfg->safe_mcs > DL_BOUND_MCS_MAX) {
        dl_log_err("config: safe_mcs=%u > %d",
                   cfg->safe_mcs, DL_BOUND_MCS_MAX);
        rc = -1;
    }
    if (cfg->safe_tx_power_dBm < DL_BOUND_TX_POWER_MIN_DBM ||
        cfg->safe_tx_power_dBm > DL_BOUND_TX_POWER_MAX_DBM) {
        dl_log_err("config: safe_tx_power_dBm=%d outside [%d, %d]",
                   cfg->safe_tx_power_dBm,
                   DL_BOUND_TX_POWER_MIN_DBM, DL_BOUND_TX_POWER_MAX_DBM);
        rc = -1;
    }
    return rc;
}
```

- [ ] **Step 5: Run tests; expect pass**

Run: `make -C drone test`
Expected: clean build, all C tests pass, including the new
deprecation test and the rewritten safe_* validation tests.

- [ ] **Step 6: Commit**

```bash
git add drone/src/dl_config.h drone/src/dl_config.c tests/drone/test_config.c
git commit -m "$(cat <<'EOF'
config: remove drone-side ceiling knobs; deprecate keys

Seven per-airframe ceiling fields (video_k_min/max, video_n_max,
depth_max, mcs_max, tx_power_min/max_dBm) removed from
dl_config_t. Parser hard-fails with a deprecation message if any
of those keys appear. dl_config_validate now compares safe_*
against DL_BOUND_* constants instead of sibling ceiling fields.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Update Python e2e fixtures and ceiling tests

The synthetic `drone.conf` written by `_sandbox` and Phase 3's e2e
test fixture both contain the seven removed keys. After Task 3 the
applier will refuse to load these configs. Two ceiling-rejection
tests also need to switch from "lower the cap to 5" to "inject above
the hardcoded cap of 7".

**Files:**
- Modify: `tests/test_drone_e2e.py`
- Modify: `tests/test_phase3_e2e.py`

- [ ] **Step 1: Run pytest to confirm the current failures**

Run: `python3 -m pytest tests/test_drone_e2e.py tests/test_phase3_e2e.py -x`
Expected: drone e2e and phase3 e2e tests fail to spawn the applier
(config load returns -1 due to the deprecation arms). This is the
failing-test state.

- [ ] **Step 2: Drop the seven keys from `tests/test_drone_e2e.py`**

Find lines 246–248:

```python
        "video_k_min": 2, "video_k_max": 8, "video_n_max": 16,
        "depth_max": 3, "mcs_max": 7,
        "tx_power_min_dBm": 0, "tx_power_max_dBm": 20,
```

Delete those three lines.

- [ ] **Step 3: Switch the two ceiling-rejection tests to inject above the hardcoded cap**

In `tests/test_drone_e2e.py`, find `test_ceiling_rejects_out_of_bound_mcs`
(starts around line 375). Change `_sandbox(tmp_path, mcs_max=5)` to
`_sandbox(tmp_path)` and change `_inject(target, mcs=7, ...)` to
`_inject(target, mcs=8, ...)` so the decision exceeds the
hardcoded `DL_BOUND_MCS_MAX = 7`. Final body:

```python
def test_ceiling_rejects_out_of_bound_mcs(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=8, bandwidth=20, tx_power=18,
                k=8, n=12, depth=1, bitrate=10000)
        time.sleep(0.3)
        assert s["wfb"].received == []
        assert s["osd_path"].exists()
        content = s["osd_path"].read_text()
        assert "REJECT" in content
```

Then find `test_ceiling_reject_emits_statustext` (around line 472)
and apply the same `mcs_max=5` → no override, `mcs=7` → `mcs=8`
substitution:

```python
def test_ceiling_reject_emits_statustext(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=8, bandwidth=20, tx_power=18,
                k=8, n=12, depth=1, bitrate=10000)
        assert _wait_until(
            lambda: any("DL REJECT" in t for _, t in s["mavlink"].statustexts()),
            timeout_s=1.0,
        ), s["mavlink"].statustexts()
        texts = [t for _, t in s["mavlink"].statustexts()]
        assert any("mcs_too_high" in t for t in texts), texts
```

- [ ] **Step 4: Drop the seven keys from `tests/test_phase3_e2e.py`**

Find lines 111–113:

```python
        f"video_k_min = 2\nvideo_k_max = 8\nvideo_n_max = 16\n"
        f"depth_max = 3\nmcs_max = 7\n"
        f"tx_power_min_dBm = 0\ntx_power_max_dBm = 20\n"
```

Delete those three lines.

- [ ] **Step 5: Run pytest; expect pass**

Run: `python3 -m pytest tests/test_drone_e2e.py tests/test_phase3_e2e.py -v`
Expected: all tests in both files pass. Pay attention to the two
ceiling-rejection tests — they should still produce "REJECT" /
"mcs_too_high" because injecting `mcs=8` exceeds the hardcoded
`DL_BOUND_MCS_MAX=7`.

- [ ] **Step 6: Run the full pytest suite**

Run: `python3 -m pytest`
Expected: all tests pass. No other test should reference the
removed keys, but a clean run confirms.

- [ ] **Step 7: Commit**

```bash
git add tests/test_drone_e2e.py tests/test_phase3_e2e.py
git commit -m "$(cat <<'EOF'
tests: drop ceiling keys from synthetic drone.conf fixtures

Drone e2e and Phase 3 e2e fixtures no longer include the seven
removed ceiling keys (the applier now refuses configs containing
them). The two ceiling-rejection tests inject mcs=8 instead of
lowering mcs_max to 5, since the bound is hardcoded at 7.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Update `conf/drone.conf.sample` and docs

**Files:**
- Modify: `conf/drone.conf.sample`
- Modify: `docs/dynamic-link-design.md`
- Modify: `docs/phase1-implementation.md`
- Possibly modify: `README.md`

- [ ] **Step 1: Trim `conf/drone.conf.sample`**

Replace lines 17–31 (the `Per-airframe safety ceiling` block plus
the TX power block):

```
# ---- Per-airframe safety ceiling (failsafe 4) -----------------------
# depth_max must stay <= 8 (wfb-ng MAX_INTERLEAVE_DEPTH, src/rx.hpp:103).
# depth_max > 1 forces video_n_max <= 32 (src/tx.cpp:1183).
# Raise k_min if this airframe should never drop into survival mode.
video_k_min = 2
video_k_max = 8
video_n_max = 16
depth_max   = 3
mcs_max     = 7

# TX power hardware / regulatory ceiling. Set to the lower of:
#   (a) chipset's calibrated operating power,
#   (b) regulatory limit for the operator's country.
tx_power_min_dBm = 0
tx_power_max_dBm = 20
```

with:

```
# ---- Failsafe 4 (per-decision bounds) -------------------------------
# Bounds are hardcoded in drone/src/dl_ceiling.h (DL_BOUND_*).
# Sources: wfb-ng MAX_INTERLEAVE_DEPTH and tx.cpp:1183 (depth/n),
# single-stream MCS cap, and a wide TX-power sanity range (iw rejects
# anything the radio can't do). Per-airframe tuning is via safe_*
# below.
```

- [ ] **Step 2: Update `docs/dynamic-link-design.md`**

The spec calls out three locations:

(a) §4 failsafe 4 description (around line 699): rewrite from
"per-airframe ceiling loaded from drone.conf" to "hardcoded bounds
from wfb-ng / hardware limits; see `drone/src/dl_ceiling.h`".

Run: `grep -n "tx_power_min_dBm\|video_k_min\|mcs_max\|depth_max\|video_n_max" docs/dynamic-link-design.md`
to find every reference, then update each in turn. Specifically:

- The failsafe 4 paragraph around line 699: drop the parameter
  list `(... tx_power_min_dBm, tx_power_max_dBm)` and replace
  with "Bounds are hardcoded constants in
  `drone/src/dl_ceiling.h` (`DL_BOUND_*`) sourced from wfb-ng
  firmware limits and single-stream MCS cap."

- The §6 config-schema block (lines ~1288–1312) and the example
  `drone.conf` block (lines ~1419–1427 and 1551–1568): delete
  the seven key listings and replace with a one-line note like
  "(ceiling bounds are hardcoded in `dl_ceiling.h`; see
  failsafe 4)".

- The "Confirms `mcs_max <= 7`" line (around 1504): rewrite to
  "MCS cap is hardcoded at 7 in `DL_BOUND_MCS_MAX`".

- Comments around line 1551–1553 about the wfb-ng constraints
  can move into `dl_ceiling.h` (they're already in Task 1's
  comment block — verify and remove the doc duplication).

(b) Append a one-line note in any `change history` or
`per-airframe` section: "2026-05-11: ceiling knobs removed from
`drone.conf`; bounds hardcoded in `dl_ceiling.h`."

If the design doc is the authoritative spec and no other section
needs surgery, leave it. Read the file end-to-end before deciding
(it's long).

- [ ] **Step 3: Append a note to `docs/phase1-implementation.md`**

Read the file's ending, then append:

```markdown
## 2026-05-11: ceiling knobs removed

The seven per-airframe ceiling keys (`video_k_min`, `video_k_max`,
`video_n_max`, `depth_max`, `mcs_max`, `tx_power_min_dBm`,
`tx_power_max_dBm`) were removed from `drone.conf`. Bounds are now
hardcoded in `drone/src/dl_ceiling.h` (`DL_BOUND_*`). The applier
still enforces failsafe 4; only the configurability went away.
`safe_defaults` remain configurable. See
`docs/superpowers/specs/2026-05-11-drop-drone-ceiling-config-design.md`.
```

- [ ] **Step 4: Sweep `README.md`**

Run: `grep -n "video_k_min\|video_k_max\|video_n_max\|depth_max\|mcs_max\|tx_power_min_dBm\|tx_power_max_dBm" README.md`

If matches: edit them out — point to `dl_ceiling.h` for bounds
instead. If no matches: skip this step.

- [ ] **Step 5: Verify everything still builds and tests pass**

Run: `make -C drone && make -C drone test && python3 -m pytest`
Expected: clean build, all tests pass.

- [ ] **Step 6: Commit**

```bash
git add conf/drone.conf.sample docs/dynamic-link-design.md \
        docs/phase1-implementation.md
# Add README.md only if it was modified.
git status README.md && git add README.md || true
git commit -m "$(cat <<'EOF'
docs: drop ceiling-knob references; point to dl_ceiling.h

Sample drone.conf, design doc, and Phase 1 notes updated to reflect
that bounds are hardcoded in dl_ceiling.h. safe_defaults remain
configurable per airframe.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Final verification

No code changes. Confirms the change set is internally consistent and
the manual smoke matches the spec's Test Plan.

**Files:** none (verification only)

- [ ] **Step 1: Clean build**

Run: `make -C drone clean && make -C drone`
Expected: zero warnings, both `dl-applier` and `dl-inject` built.

- [ ] **Step 2: All C tests pass**

Run: `make -C drone test`
Expected: every `DL_TEST` registered passes; `dl_test_failed == 0`
at the end.

- [ ] **Step 3: All Python tests pass**

Run: `python3 -m pytest`
Expected: green across GS unit + Python e2e.

- [ ] **Step 4: Manual smoke — out-of-bound MCS rejection**

In one terminal, start an applier on a test conf (any minimal
conf without the removed keys):

```bash
drone/build/dl-applier --config conf/drone.conf.sample
```

In another terminal, send an out-of-bound MCS:

```bash
drone/build/dl-inject --target 127.0.0.1:5800 --mcs 8 \
    --bandwidth 20 --tx-power 18 --k 8 --n 12 --depth 1 \
    --bitrate 10000
```

Expected: applier log contains
`ceiling: mcs=8 > 7` (or similar) and `REJECT` / `mcs_too_high`.

- [ ] **Step 5: Manual smoke — deprecated key fails load**

Create a minimal conf containing one removed key:

```bash
echo "mcs_max = 7" > /tmp/dl-deprecated.conf
drone/build/dl-applier --config /tmp/dl-deprecated.conf
echo "exit=$?"
```

Expected: applier exits non-zero. Stderr contains
`mcs_max is no longer supported (removed 2026-05-11)`.

- [ ] **Step 6: Sanity check the diff stack**

Run: `git log --oneline ed4d0c4..HEAD`
Expected: 5 commits matching the task subjects (ceiling
constants, ceiling refactor, config knobs, e2e fixtures, docs).

No extra commit at this stage — verification only.
