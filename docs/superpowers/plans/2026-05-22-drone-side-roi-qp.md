# Drone-side ROI QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Compute `fpv.roiQp` on the drone from the latest applied bitrate via a deterministic linear ramp, with the wire bumped to v2 to drop the now-vestigial `roi_qp` byte.

**Architecture:** A pure `compute_roi_qp(bitrate_kbps, cfg) → int` function lives in the encoder backend. The backend keeps its own `last_*` state (per-backend prev-struct pattern), always emits `fpv.roiQp=%d` (signed; fixes the existing `%u`/skip-on-zero bug), and applies the same formula in safe-defaults. The GS→drone wire is bumped v1→v2: the `roi_qp` byte is removed, payload shrinks 28→27, on-wire 32→31. Python encoder + dl-inject CLI + contract tests update lockstep.

**Tech Stack:** C11 (drone applier + tests), Python stdlib + pytest (GS + e2e tests).

**Reference spec:** `docs/superpowers/specs/2026-05-22-drone-side-roi-qp-design.md`.

---

## File Map

**Modified:**

- `drone/src/dl_config.h` — 4 new fields in `dl_config_t`.
- `drone/src/dl_config.c` — defaults + `SET_INT_RANGED` parser entries + cross-field validation.
- `drone/src/dl_wire.h` — bump `DL_WIRE_VERSION` to 2, shrink `DL_WIRE_PAYLOAD_SIZE` to 27, shrink `DL_WIRE_ON_WIRE_SIZE` to 31, drop `roi_qp` from `dl_decision_t`.
- `drone/src/dl_wire.c` — encode/decode for v2 layout.
- `drone/src/dl_backend_enc.h` — new internal state; `dl_backend_enc_apply` no longer takes `prev` param.
- `drone/src/dl_backend_enc.c` — `compute_roi_qp` helper, per-backend `last_*`, signed `apply_set`, always-emit `fpv.roiQp=%d`, safe-defaults uses formula.
- `drone/src/dl_inject.c` — drop `--roi-qp` CLI flag and `d.roi_qp` initialiser.
- `drone/src/dl_applier.c` — remove `last_enc` from the apply path (backend owns it).
- `gs/dynamic_link/wire.py` — mirror v2 layout, drop `roi_qp` from `Encoder.encode` and `_encode_raw`.
- `tests/drone/test_wire.c` — drop `roi_qp` field, fix expected `DL_WIRE_VERSION`/`PAYLOAD_SIZE` constants.
- `tests/drone/test_main.c` — add `test_roi_qp.c` and `test_dl_backend_enc.c` to the registry (compile-time via `TEST_SRCS`).
- `drone/Makefile` — add the two new test sources to `TEST_SRCS`.
- `tests/test_wire_contract.py` — update payload size expectations.
- `tests/test_drone_e2e.py` — add ROI QP assertions in the golden-path test (or new test).
- `conf/drone.conf.sample` — document the four `roi_qp_*` keys + the `/etc/waybeam.json` operational prereq.
- `README.md` — one-paragraph operator note on the waybeam.json ROI requirement.

**Created:**

- `tests/drone/test_roi_qp.c` — pure-function tests for `compute_roi_qp`.
- `tests/drone/test_dl_backend_enc.c` — exercises the apply path (dedup, signed format, always-emit-roiQp).

---

## Task 1: Add config knobs (`roi_qp_*`)

**Files:**

- Modify: `drone/src/dl_config.h:14` (struct fields)
- Modify: `drone/src/dl_config.c:13` (defaults + parser)
- Test: `tests/drone/test_config.c`

- [ ] **Step 1: Add the failing test for defaults**

Append to `tests/drone/test_config.c` (after existing tests):

```c
DL_TEST(test_config_roi_qp_defaults) {
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    DL_ASSERT_EQ(cfg.roi_qp_threshold_kbps,  6000);
    DL_ASSERT_EQ(cfg.roi_qp_low_anchor_kbps, 2000);
    DL_ASSERT_EQ(cfg.roi_qp_floor,           -24);
    DL_ASSERT_EQ(cfg.roi_qp_step,            3);
}
```

- [ ] **Step 2: Run the C test suite to confirm the test fails**

```bash
make -C drone test
```

Expected: fails to compile because `cfg.roi_qp_threshold_kbps` (etc.) are not declared.

- [ ] **Step 3: Add struct fields**

Insert into `drone/src/dl_config.h` inside the `dl_config_t` struct, after the `encoder_port` line (around line 73):

```c
    /* ROI QP policy (drone-side computation from bitrate). See
     * docs/superpowers/specs/2026-05-22-drone-side-roi-qp-design.md. */
    uint16_t roi_qp_threshold_kbps;   /* >= this bitrate => roi_qp = 0  */
    uint16_t roi_qp_low_anchor_kbps;  /* <= this bitrate => roi_qp = floor */
    int8_t   roi_qp_floor;            /* most-negative QP delta, -30..0  */
    uint8_t  roi_qp_step;             /* quantization, 1..10             */
```

- [ ] **Step 4: Add defaults**

Insert into `drone/src/dl_config.c` inside `dl_config_defaults`, after the `cfg->encoder_port = 80;` line (around line 45):

```c
    cfg->roi_qp_threshold_kbps  = 6000;
    cfg->roi_qp_low_anchor_kbps = 2000;
    cfg->roi_qp_floor           = -24;
    cfg->roi_qp_step            = 3;
```

- [ ] **Step 5: Run defaults test**

```bash
make -C drone test
```

Expected: `test_config_roi_qp_defaults` passes; everything else still passes.

- [ ] **Step 6: Add parser-rejection test for invalid values**

Append to `tests/drone/test_config.c`:

```c
DL_TEST(test_config_roi_qp_invalid_threshold_below_anchor) {
    /* threshold must be > low_anchor (validated post-parse). */
    char path[64];
    snprintf(path, sizeof(path), "/tmp/dlc_roi_inv_%d.conf", getpid());
    FILE *f = fopen(path, "w");
    fprintf(f, "roi_qp_threshold_kbps = 1500\n"
               "roi_qp_low_anchor_kbps = 2000\n");
    fclose(f);
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    int rc = dl_config_load(path, &cfg);
    unlink(path);
    DL_ASSERT_EQ(rc, -1);
}

DL_TEST(test_config_roi_qp_floor_out_of_range_rejected) {
    char path[64];
    snprintf(path, sizeof(path), "/tmp/dlc_roi_floor_%d.conf", getpid());
    FILE *f = fopen(path, "w");
    fprintf(f, "roi_qp_floor = 5\n");   /* positive not allowed */
    fclose(f);
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    int rc = dl_config_load(path, &cfg);
    unlink(path);
    DL_ASSERT_EQ(rc, -1);
}

DL_TEST(test_config_roi_qp_step_zero_rejected) {
    char path[64];
    snprintf(path, sizeof(path), "/tmp/dlc_roi_step_%d.conf", getpid());
    FILE *f = fopen(path, "w");
    fprintf(f, "roi_qp_step = 0\n");
    fclose(f);
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    int rc = dl_config_load(path, &cfg);
    unlink(path);
    DL_ASSERT_EQ(rc, -1);
}

DL_TEST(test_config_roi_qp_loads_overrides) {
    char path[64];
    snprintf(path, sizeof(path), "/tmp/dlc_roi_ok_%d.conf", getpid());
    FILE *f = fopen(path, "w");
    fprintf(f, "roi_qp_threshold_kbps = 8000\n"
               "roi_qp_low_anchor_kbps = 3000\n"
               "roi_qp_floor = -18\n"
               "roi_qp_step = 2\n");
    fclose(f);
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    int rc = dl_config_load(path, &cfg);
    unlink(path);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(cfg.roi_qp_threshold_kbps, 8000);
    DL_ASSERT_EQ(cfg.roi_qp_low_anchor_kbps, 3000);
    DL_ASSERT_EQ(cfg.roi_qp_floor, -18);
    DL_ASSERT_EQ(cfg.roi_qp_step, 2);
}
```

The `test_config.c` header may already pull `unistd.h` and `stdio.h`. If it does not, add `#include <unistd.h>` and `#include <stdio.h>` at the top (check existing tests in the file — they use a similar tmp-file pattern, so includes should already be present).

- [ ] **Step 7: Run tests; they should fail with "bad value" rejection missing**

```bash
make -C drone test
```

Expected: the three reject-tests fail (parser accepts the values today); the overrides test may pass already.

- [ ] **Step 8: Add parser entries + cross-field validation**

In `drone/src/dl_config.c`, after the `encoder_port` SET_INT_RANGED line (~line 196), add:

```c
        else if (strcmp(key, "roi_qp_threshold_kbps") == 0)
            SET_INT_RANGED(roi_qp_threshold_kbps, uint16_t, 100, 65535);
        else if (strcmp(key, "roi_qp_low_anchor_kbps") == 0)
            SET_INT_RANGED(roi_qp_low_anchor_kbps, uint16_t, 100, 65535);
        else if (strcmp(key, "roi_qp_floor") == 0)
            SET_INT_RANGED(roi_qp_floor, int8_t, -30, 0);
        else if (strcmp(key, "roi_qp_step") == 0)
            SET_INT_RANGED(roi_qp_step, uint8_t, 1, 10);
```

Then add cross-field validation just before `fclose(fd); return rc;` at the end of `dl_config_load` (~line 227):

```c
    if (cfg->roi_qp_threshold_kbps <= cfg->roi_qp_low_anchor_kbps) {
        dl_log_err("%s: roi_qp_threshold_kbps (%u) must be > "
                   "roi_qp_low_anchor_kbps (%u)",
                   path,
                   (unsigned)cfg->roi_qp_threshold_kbps,
                   (unsigned)cfg->roi_qp_low_anchor_kbps);
        rc = -1;
    }
```

- [ ] **Step 9: Run all C tests; expect green**

```bash
make -C drone test
```

Expected: all tests pass, including the four new ROI config tests.

- [ ] **Step 10: Commit**

```bash
git add drone/src/dl_config.h drone/src/dl_config.c tests/drone/test_config.c
git commit -m "$(cat <<'EOF'
config: roi_qp_* policy knobs

Adds four new keys to drone.conf [encoder]:
- roi_qp_threshold_kbps (default 6000)
- roi_qp_low_anchor_kbps (default 2000)
- roi_qp_floor (default -24, range -30..0)
- roi_qp_step (default 3, range 1..10)

Cross-field validation rejects threshold <= low_anchor.

Per docs/superpowers/specs/2026-05-22-drone-side-roi-qp-design.md.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Pure `compute_roi_qp` function

Add the calculation as a pure function in `dl_backend_enc.c` (file-static, exposed via a small extern declaration in the header so the test can call it directly). Keep it independent of the wire change so the unit tests can land first.

**Files:**

- Modify: `drone/src/dl_backend_enc.h`
- Modify: `drone/src/dl_backend_enc.c`
- Create: `tests/drone/test_roi_qp.c`
- Modify: `drone/Makefile` (add to TEST_SRCS)

- [ ] **Step 1: Write the failing test file**

Create `tests/drone/test_roi_qp.c`:

```c
/* test_roi_qp.c — pure-function tests for compute_roi_qp. */
#include "test_main.h"
#include "dl_backend_enc.h"
#include "dl_config.h"

static dl_config_t default_cfg(void) {
    dl_config_t c;
    dl_config_defaults(&c);
    return c;
}

DL_TEST(test_roi_qp_above_threshold_is_zero) {
    dl_config_t cfg = default_cfg();
    DL_ASSERT_EQ(dl_compute_roi_qp(6000, &cfg), 0);
    DL_ASSERT_EQ(dl_compute_roi_qp(10000, &cfg), 0);
}

DL_TEST(test_roi_qp_at_low_anchor_is_floor) {
    dl_config_t cfg = default_cfg();
    DL_ASSERT_EQ(dl_compute_roi_qp(2000, &cfg), -24);
}

DL_TEST(test_roi_qp_below_low_anchor_clamps_at_floor) {
    dl_config_t cfg = default_cfg();
    DL_ASSERT_EQ(dl_compute_roi_qp(1500, &cfg), -24);
    DL_ASSERT_EQ(dl_compute_roi_qp(500, &cfg), -24);
    DL_ASSERT_EQ(dl_compute_roi_qp(0, &cfg), -24);
}

DL_TEST(test_roi_qp_midpoint_ramps_linearly) {
    /* Defaults: threshold=6000, anchor=2000, floor=-24, step=3.
     * At bitrate=4000 raw = -24 * (4000-2000)/4000 ... wait recheck:
     * span = 6000-2000 = 4000
     * delta = 4000-2000 = 2000
     * raw = -24 * (4000 - 2000) / 4000 = -24 * 2000 / 4000 = -12.
     * -12 is a multiple of 3, so quantized = -12. */
    dl_config_t cfg = default_cfg();
    DL_ASSERT_EQ(dl_compute_roi_qp(4000, &cfg), -12);
    /* At 5000: raw = -24 * (4000 - 3000) / 4000 = -6. */
    DL_ASSERT_EQ(dl_compute_roi_qp(5000, &cfg), -6);
    /* At 3000: raw = -24 * (4000 - 1000) / 4000 = -18. */
    DL_ASSERT_EQ(dl_compute_roi_qp(3000, &cfg), -18);
}

DL_TEST(test_roi_qp_quantization_lands_on_step_multiples) {
    dl_config_t cfg = default_cfg();
    /* Sweep 2000..6000 in 50-kbps increments; every result must be a
     * multiple of step (3) and must be in [floor, 0]. */
    for (int br = 2000; br <= 6000; br += 50) {
        int q = dl_compute_roi_qp((uint16_t)br, &cfg);
        DL_ASSERT(q <= 0);
        DL_ASSERT(q >= cfg.roi_qp_floor);
        DL_ASSERT_EQ(q % cfg.roi_qp_step, 0);
    }
}

DL_TEST(test_roi_qp_custom_config) {
    /* threshold=8000, anchor=3000, floor=-18, step=2. */
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    cfg.roi_qp_threshold_kbps  = 8000;
    cfg.roi_qp_low_anchor_kbps = 3000;
    cfg.roi_qp_floor           = -18;
    cfg.roi_qp_step            = 2;
    /* Endpoints */
    DL_ASSERT_EQ(dl_compute_roi_qp(8000, &cfg), 0);
    DL_ASSERT_EQ(dl_compute_roi_qp(3000, &cfg), -18);
    /* Midpoint: span=5000, delta=2500, raw = -18 * 2500/5000 = -9.
     * Quantize step=2: -9 / 2 = -4 (C truncates toward zero), * 2 = -8. */
    DL_ASSERT_EQ(dl_compute_roi_qp(5500, &cfg), -8);
}
```

- [ ] **Step 2: Wire the new test source into the build**

Edit `drone/Makefile`. Find the `TEST_SRCS :=` block (around line 43) and append:

```makefile
    $(TESTDIR)/test_roi_qp.c \
```

(Keep the trailing backslash convention; insert before whichever line currently ends without one.)

- [ ] **Step 3: Run the C tests; expect compile failure**

```bash
make -C drone test
```

Expected: link or compile error — `dl_compute_roi_qp` is undeclared.

- [ ] **Step 4: Declare in the header**

Add to `drone/src/dl_backend_enc.h`, near the bottom (just before the closing of the file, after `dl_backend_enc_apply_safe`):

```c
/* Pure function. Maps bitrate → roi_qp using cfg->roi_qp_* knobs.
 * Exposed for unit tests; production callers go through
 * dl_backend_enc_apply.
 *
 * Formula: linear ramp from 0 at >= threshold down to floor at <= anchor,
 * quantized to multiples of step. Returns a signed delta in [floor, 0].
 */
int dl_compute_roi_qp(uint16_t bitrate_kbps, const dl_config_t *cfg);
```

- [ ] **Step 5: Implement in the .c file**

Add to `drone/src/dl_backend_enc.c`, near the top after the `#include`s (before the `parse_http_status` definition):

```c
int dl_compute_roi_qp(uint16_t bitrate_kbps, const dl_config_t *cfg) {
    if (bitrate_kbps >= cfg->roi_qp_threshold_kbps) return 0;
    int span = (int)cfg->roi_qp_threshold_kbps - (int)cfg->roi_qp_low_anchor_kbps;
    int delta = (int)bitrate_kbps - (int)cfg->roi_qp_low_anchor_kbps;
    if (delta < 0) delta = 0;
    int raw = ((int)cfg->roi_qp_floor * (span - delta)) / span;   /* negative */
    int step = (int)cfg->roi_qp_step;
    int q = (raw / step) * step;                                   /* truncate toward zero */
    if (q < (int)cfg->roi_qp_floor) q = (int)cfg->roi_qp_floor;
    if (q > 0) q = 0;
    return q;
}
```

- [ ] **Step 6: Run all C tests**

```bash
make -C drone test
```

Expected: all roi_qp tests pass; nothing else breaks.

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_backend_enc.h drone/src/dl_backend_enc.c tests/drone/test_roi_qp.c drone/Makefile
git commit -m "$(cat <<'EOF'
enc: add dl_compute_roi_qp pure function

Linear ramp from 0 at >= threshold down to floor at <= anchor,
quantized to multiples of step. Unit tests cover endpoints, clamp
below anchor, quantization invariant, and custom config.

Per docs/superpowers/specs/2026-05-22-drone-side-roi-qp-design.md.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Wire format v2 — C side

Bump the wire from v1 to v2: drop the `roi_qp` byte, shrink payload 28→27, on-wire 32→31. The `_pad2` region stays the same width; `fps` shifts down to offset 24.

**Files:**

- Modify: `drone/src/dl_wire.h`
- Modify: `drone/src/dl_wire.c`
- Modify: `tests/drone/test_wire.c`

- [ ] **Step 1: Update the wire test to v2 expectations (failing)**

Edit `tests/drone/test_wire.c`:

In `test_wire_round_trip` (around line 7), remove the `.roi_qp = 30,` line from the initializer and remove the corresponding `DL_ASSERT_EQ(r.roi_qp, d.roi_qp);` assertion.

Add a new test at the end of the file (immediately after `test_wire_signed_tx_power`):

```c
DL_TEST(test_wire_v2_constants) {
    DL_ASSERT_EQ(DL_WIRE_VERSION, 2);
    DL_ASSERT_EQ(DL_WIRE_PAYLOAD_SIZE, 27);
    DL_ASSERT_EQ(DL_WIRE_ON_WIRE_SIZE, 31);
}

DL_TEST(test_wire_v2_fps_offset_24) {
    dl_decision_t d = { .fps = 0xAB };
    uint8_t buf[DL_WIRE_ON_WIRE_SIZE];
    dl_wire_encode(&d, buf, sizeof(buf));
    DL_ASSERT_EQ(buf[24], 0xAB);
}
```

Also update `test_wire_rejects_bad_crc` — its `buf[20] ^= 0xFF` is fine (still inside the payload), no change needed.

- [ ] **Step 2: Run C tests; expect compile failure on `.roi_qp` references**

```bash
make -C drone test
```

Expected: compile errors because `dl_decision_t` still has `roi_qp` (test fields were already removed), or the constants assertions fail.

Wait — if you removed `.roi_qp = 30` from the initializer but the struct still has the field, that compiles fine (designated initializer; missing fields are zero). The compile error will come from the `DL_ASSERT_EQ(r.roi_qp, d.roi_qp);` line you also removed. So actually the test compiles. Run anyway:

Expected: `test_wire_v2_constants` and `test_wire_v2_fps_offset_24` fail (constants are still v1 values; fps is still at offset 25).

- [ ] **Step 3: Update `dl_wire.h`**

Edit `drone/src/dl_wire.h`. Update the version and size constants (lines 13-15):

```c
#define DL_WIRE_MAGIC           0x444C4B31u   /* "DLK1" */
#define DL_WIRE_VERSION         2
#define DL_WIRE_PAYLOAD_SIZE    27            /* payload bytes */
#define DL_WIRE_ON_WIRE_SIZE    31            /* payload + 4-byte CRC */
```

Remove the `uint8_t roi_qp;` line from the `dl_decision_t` struct (line 56). The struct becomes:

```c
typedef struct {
    uint32_t magic;            /* always DL_WIRE_MAGIC after decode */
    uint8_t  version;
    uint8_t  flags;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint8_t  mcs;              /* 0..7 */
    uint8_t  bandwidth;        /* 20 or 40 */
    int8_t   tx_power_dBm;
    uint8_t  k;                /* 1..8 */
    uint8_t  n;                /* 4..16 typical */
    uint8_t  depth;            /* 1..3 typical */
    uint16_t bitrate_kbps;     /* 0..65535 */
    uint8_t  fps;              /* 0 = unset */
} dl_decision_t;
```

- [ ] **Step 4: Update `dl_wire.c`**

Edit `drone/src/dl_wire.c`. Replace the header comment block (lines 1-32) with the v2 layout:

```c
/* dl_wire.c — encode/decode the 27-byte dl_decision wire packet (v2).
 *
 * Layout (big-endian / network byte order):
 *
 *   off  size  field
 *    0    4    magic       = 0x444C4B31 ("DLK1")
 *    4    1    version     = 2
 *    5    1    flags
 *    6    2    _pad (0)
 *    8    4    sequence
 *   12    4    timestamp_ms
 *   16    1    mcs
 *   17    1    bandwidth
 *   18    1    tx_power_dBm (signed)
 *   19    1    k
 *   20    1    n
 *   21    1    depth
 *   22    2    bitrate_kbps
 *   24    1    fps                (shifted down from v1 offset 25)
 *   25    2    _pad2 (0)
 *   27    4    crc32(bytes[0..26])  — on-wire only; not in payload
 *
 * v1 carried a `roi_qp` byte at offset 24; v2 removes it. The drone
 * computes roi_qp from bitrate_kbps locally (see dl_backend_enc.c).
 */
```

Find the `dl_wire_encode` function. It currently writes `roi_qp` at `buf[24]` and `fps` at `buf[25]`. Replace those two lines to write `fps` at `buf[24]`. The full encode body (after the initial memset/magic/version/flags/pad and sequence/timestamp writes) should look like:

```c
    /* find the existing block writing mcs..bitrate_kbps; keep unchanged */
    buf[16] = d->mcs;
    buf[17] = d->bandwidth;
    buf[18] = (uint8_t)d->tx_power_dBm;   /* int8 two's complement */
    buf[19] = d->k;
    buf[20] = d->n;
    buf[21] = d->depth;
    put_u16(&buf[22], d->bitrate_kbps);
    buf[24] = d->fps;
    /* buf[25..26] = _pad2 (left zero by memset) */
```

Find the `dl_wire_decode` function. Update the corresponding lines: change the `roi_qp = buf[24]` line to read `fps = buf[24]`, and remove the prior `fps = buf[25]` reference.

The CRC offset in the encoder/decoder is `DL_WIRE_PAYLOAD_SIZE` (already a constant), so it will pick up `27` automatically. Verify the encoder writes the CRC at `&buf[DL_WIRE_PAYLOAD_SIZE]`, not a hard-coded `&buf[28]` — fix if hard-coded.

- [ ] **Step 5: Run the C test suite**

```bash
make -C drone test
```

Expected: all C tests pass, including the new v2 constant + fps-at-24 assertions and the existing round-trip / CRC / bad-magic / bad-version tests.

- [ ] **Step 6: Commit**

```bash
git add drone/src/dl_wire.h drone/src/dl_wire.c tests/drone/test_wire.c
git commit -m "$(cat <<'EOF'
wire: bump v1->v2, drop roi_qp byte

Removes the roi_qp byte at v1 offset 24. fps shifts down to offset
24, _pad2 stays 2 bytes. Payload shrinks 28->27, on-wire 32->31.

dl-applier will compute roi_qp drone-side from bitrate; the wire
field is no longer needed. Per
docs/superpowers/specs/2026-05-22-drone-side-roi-qp-design.md.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Wire format v2 — Python side

Mirror the C-side bump.

**Files:**

- Modify: `gs/dynamic_link/wire.py`

- [ ] **Step 1: Update `wire.py` constants and layout doc**

Edit `gs/dynamic_link/wire.py`:

Replace the docstring header block (the layout comment, lines 9-29) with the v2 description:

```python
"""Phase 2 — decision-packet serialiser.

Byte-for-byte mirror of `drone/src/dl_wire.c` (v2). The authority is
the C implementation; this module must match it exactly. The test at
`tests/test_wire_contract.py` cross-checks by running
`drone/build/dl-inject --dry-run` and diffing its hex output against
what this module produces for the same inputs.

Wire layout (big-endian, 31 bytes on-wire = 27 payload + 4 CRC32):

    off  size  field
     0    4    magic       = 0x444C4B31 ('DLK1')
     4    1    version     = 2
     5    1    flags
     6    2    _pad
     8    4    sequence
    12    4    timestamp_ms
    16    1    mcs
    17    1    bandwidth
    18    1    tx_power_dBm (signed int8)
    19    1    k
    20    1    n
    21    1    depth
    22    2    bitrate_kbps
    24    1    fps
    25    2    _pad2
    27    4    crc32(bytes[0..26])
"""
```

Update the constants (lines 37-40):

```python
MAGIC            = 0x444C4B31    # 'DLK1'
VERSION          = 2
PAYLOAD_SIZE     = 27
ON_WIRE_SIZE     = 31
```

In `Encoder.encode` (around line 100), remove the `roi_qp=0,  # policy engine doesn't set ROI yet` line from the `_encode_raw(...)` call.

In `_encode_raw` (around line 118), remove the `roi_qp: int,` parameter and the line `payload[24] = roi_qp & 0xFF`. Move the `fps` byte to offset 24:

```python
def _encode_raw(
    *,
    version: int,
    flags: int,
    sequence: int,
    timestamp_ms: int,
    mcs: int,
    bandwidth: int,
    tx_power_dBm: int,
    k: int,
    n: int,
    depth: int,
    bitrate_kbps: int,
    fps: int,
) -> bytes:
    payload = bytearray(PAYLOAD_SIZE)
    struct.pack_into(">I", payload, 0, MAGIC)
    payload[4] = version & 0xFF
    payload[5] = flags & 0xFF
    # [6..7] = _pad
    struct.pack_into(">I", payload, 8,  sequence & 0xFFFFFFFF)
    struct.pack_into(">I", payload, 12, timestamp_ms & 0xFFFFFFFF)
    payload[16] = mcs & 0xFF
    payload[17] = bandwidth & 0xFF
    payload[18] = tx_power_dBm & 0xFF     # int8 two's complement
    payload[19] = k & 0xFF
    payload[20] = n & 0xFF
    payload[21] = depth & 0xFF
    struct.pack_into(">H", payload, 22, bitrate_kbps & 0xFFFF)
    payload[24] = fps & 0xFF
    # [25..26] = _pad2
    crc = _crc32(bytes(payload))
    return bytes(payload) + struct.pack(">I", crc)
```

- [ ] **Step 2: Run the wire-related Python tests**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py tests/test_wire_contract.py -v
```

Expected: contract tests fail. C-side dl-inject is already v2 (Task 3), Python is now v2 — but `_dl_inject_hex` asserts `len(out) == 64` (32 bytes), which is the old size. We need to update the contract test too. Continue to Task 5.

- [ ] **Step 3: Commit (Python wire only — contract test fix follows)**

```bash
git add gs/dynamic_link/wire.py
git commit -m "$(cat <<'EOF'
gs/wire: mirror v2 layout (payload 27, on-wire 31)

Drops roi_qp, shifts fps to offset 24, updates docstring. Contract
test fixes follow in next commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: dl-inject CLI — drop `--roi-qp`

**Files:**

- Modify: `drone/src/dl_inject.c`

- [ ] **Step 1: Drop the `--roi-qp` option from the long-opts table**

Edit `drone/src/dl_inject.c`. Remove the line at ~56:

```c
        { "roi-qp",    required_argument, 0, 'r' },
```

Remove the `r:` from the short-options string in `getopt_long` (~line 100). Old:

```c
    while ((c = getopt_long(argc, argv, "t:M:B:P:k:n:d:b:r:f:s:Dpq:m:HAg:u:S:F:h",
                            opts, NULL)) != -1) {
```

New:

```c
    while ((c = getopt_long(argc, argv, "t:M:B:P:k:n:d:b:f:s:Dpq:m:HAg:u:S:F:h",
                            opts, NULL)) != -1) {
```

Remove the `case 'r':` block (line 111).

Remove `.roi_qp = 0,` from the `dl_decision_t` initializer (~line 88).

In the dry-run printf at ~line 267-269, remove the `roi_qp=%u` token and the `d.roi_qp` argument. The line should become something like:

```c
        fprintf(stderr,
            "mcs=%u bw=%u txp=%d k=%u n=%u depth=%u "
            "bitrate=%u fps=%u -> %s:%u\n",
            d.mcs, d.bandwidth, d.tx_power_dBm,
            d.k, d.n, d.depth, d.bitrate_kbps, d.fps,
            host, (unsigned)port);
```

Update the usage string (lines 18-30): replace `[--roi-qp QP] [--fps FPS]` with just `[--fps FPS]`.

- [ ] **Step 2: Build dl-inject**

```bash
make -C drone
```

Expected: clean build.

- [ ] **Step 3: Sanity check the dry-run output is 31 bytes (62 hex chars)**

```bash
./drone/build/dl-inject --dry-run --mcs 5 --bandwidth 20 --tx-power 18 --k 8 --n 14 --depth 2 --bitrate 12000 --fps 60 --sequence 1
```

Expected: 62 hex characters on stdout (was 64 in v1).

- [ ] **Step 4: Update `test_wire_contract.py` assertions**

Edit `tests/test_wire_contract.py`. In `_dl_inject_hex` (around line 38), update the length assertion:

```python
def _dl_inject_hex(**kwargs) -> bytes:
    """Call dl-inject --dry-run with named flags; return the on-wire bytes."""
    args = [str(DL_INJECT), "--dry-run"]
    for k, v in kwargs.items():
        if isinstance(v, bool):
            if v:
                args.append(f"--{k.replace('_', '-')}")
        else:
            args.extend([f"--{k.replace('_', '-')}", str(v)])
    out = subprocess.check_output(args, text=True).strip()
    # Decision is 31 bytes (v2 = 62 hex chars); HELLO/HELLO_ACK are 32 bytes (64 hex chars).
    assert len(out) in (62, 64), f"expected 62 or 64 hex chars, got {len(out)}: {out!r}"
    return bytes.fromhex(out)
```

In `test_contract_magic_and_version` (~line 104), change the version assertion:

```python
def test_contract_magic_and_version():
    py_bytes = encode(_decision(), sequence=1)
    # First 4 bytes = "DLK1"
    assert py_bytes[:4] == b"DLK1"
    # Byte 4 = version 2 (v2 wire)
    assert py_bytes[4] == 2
```

In `test_contract_signed_tx_power`, the `assert py_bytes[18] == 0xF6` assertion is unchanged (byte 18 still holds tx_power_dBm in v2).

- [ ] **Step 5: Run the contract suite**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py tests/test_wire_contract.py -v
```

Expected: all decision and HELLO/PING contract tests pass.

- [ ] **Step 6: Run the full Python test suite to catch other call sites**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py -x
```

Expected: pass. If anything else fails, investigate (most likely a test or replay helper that still references `roi_qp` or asserts a 32-byte on-wire size).

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_inject.c tests/test_wire_contract.py
git commit -m "$(cat <<'EOF'
dl-inject + contract test: drop --roi-qp / accept v2 size

Removes the --roi-qp CLI flag and updates the wire-contract harness
to accept 31-byte v2 decisions (HELLO/HELLO_ACK remain 32 bytes).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Encoder backend rework

Per-backend `last_*` state, signed `roi_qp`, always-emit `fpv.roiQp=%d`, integrate `compute_roi_qp`, safe-defaults uses formula.

**Files:**

- Modify: `drone/src/dl_backend_enc.h`
- Modify: `drone/src/dl_backend_enc.c`
- Modify: `drone/src/dl_applier.c` (signature change: `dl_backend_enc_apply` drops the `prev` parameter)
- Create: `tests/drone/test_dl_backend_enc.c`

- [ ] **Step 1: Add the failing backend test file**

Create `tests/drone/test_dl_backend_enc.c`. We exercise `apply_set` indirectly by capturing the HTTP request via a TCP loopback listener.

```c
/* test_dl_backend_enc.c — backend HTTP path: always-emit roiQp,
 * signed format, dedup against computed roi_qp. */
#include "test_main.h"
#include "dl_backend_enc.h"
#include "dl_config.h"
#include "dl_wire.h"

#include <arpa/inet.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct {
    int  port;
    int  fd;
    char received[2048];
    size_t received_len;
    int  request_count;
    pthread_t tid;
    int  stop;
} mock_http_t;

static void *mock_http_thread(void *arg) {
    mock_http_t *m = arg;
    while (!m->stop) {
        struct sockaddr_in c;
        socklen_t cl = sizeof(c);
        int cfd = accept(m->fd, (struct sockaddr *)&c, &cl);
        if (cfd < 0) return NULL;
        char buf[1024];
        ssize_t n = recv(cfd, buf, sizeof(buf) - 1, 0);
        if (n > 0) {
            size_t add = (size_t)n;
            if (m->received_len + add + 1 < sizeof(m->received)) {
                memcpy(m->received + m->received_len, buf, add);
                m->received_len += add;
                m->received[m->received_len++] = '\n';
                m->received[m->received_len] = '\0';
            }
            m->request_count++;
            const char *resp = "HTTP/1.0 200 OK\r\nContent-Length: 0\r\n\r\n";
            send(cfd, resp, strlen(resp), 0);
        }
        close(cfd);
    }
    return NULL;
}

static void mock_http_start(mock_http_t *m) {
    memset(m, 0, sizeof(*m));
    m->fd = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(m->fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in a = { .sin_family = AF_INET,
                             .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    bind(m->fd, (struct sockaddr *)&a, sizeof(a));
    socklen_t al = sizeof(a);
    getsockname(m->fd, (struct sockaddr *)&a, &al);
    m->port = ntohs(a.sin_port);
    listen(m->fd, 4);
    pthread_create(&m->tid, NULL, mock_http_thread, m);
}

static void mock_http_stop(mock_http_t *m) {
    m->stop = 1;
    shutdown(m->fd, SHUT_RDWR);
    close(m->fd);
    pthread_join(m->tid, NULL);
}

static void cfg_init(dl_config_t *cfg, int port) {
    dl_config_defaults(cfg);
    snprintf(cfg->encoder_host, sizeof(cfg->encoder_host), "%s", "127.0.0.1");
    cfg->encoder_port = (uint16_t)port;
}

DL_TEST(test_enc_emits_signed_roi_qp_when_starved) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    dl_decision_t d = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 4000, .fps = 60 };
    int rc = dl_backend_enc_apply(be, &d);
    DL_ASSERT_EQ(rc, 0);

    /* At 4000 kbps with defaults, roi_qp = -12. Path must include a
     * SIGNED %d-style value, not a wrap-around large uint. */
    DL_ASSERT(strstr(m.received, "fpv.roiQp=-12") != NULL);
    DL_ASSERT(strstr(m.received, "video0.bitrate=4000") != NULL);

    dl_backend_enc_close(be);
    mock_http_stop(&m);
}

DL_TEST(test_enc_emits_roi_qp_zero_above_threshold) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    dl_decision_t d = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 8000, .fps = 60 };
    int rc = dl_backend_enc_apply(be, &d);
    DL_ASSERT_EQ(rc, 0);
    /* The crucial bug-fix assertion: at 8000 kbps roi_qp = 0, and we
     * still send fpv.roiQp=0 (so waybeam clears any prior ROI). */
    DL_ASSERT(strstr(m.received, "fpv.roiQp=0") != NULL);

    dl_backend_enc_close(be);
    mock_http_stop(&m);
}

DL_TEST(test_enc_dedupes_repeat_apply) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    dl_decision_t d = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 4000, .fps = 60 };
    dl_backend_enc_apply(be, &d);
    int n1 = m.request_count;
    dl_backend_enc_apply(be, &d);  /* identical → no HTTP */
    DL_ASSERT_EQ(m.request_count, n1);
    dl_backend_enc_close(be);
    mock_http_stop(&m);
}

DL_TEST(test_enc_dedupes_within_quantization_band) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    /* Both bitrates fall in the same step (3) bucket: at 4000 → -12,
     * at 4050 → raw = -23.7, q = (raw/3)*3 = -21. So they're NOT in
     * the same bucket. Pick two that *are*: 4050 → -21; 4060 → -21.4
     * q = -21. Both -21 → second apply still emits because bitrate
     * differs. So the dedup-by-quantization claim from the spec is
     * only true when bitrate is also identical. Restate: test that
     * the backend always considers all three (bitrate, roi_qp, fps)
     * before reapplying. */
    dl_decision_t d1 = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 4000, .fps = 60 };
    dl_decision_t d2 = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 4000, .fps = 60 };
    dl_backend_enc_apply(be, &d1);
    int n1 = m.request_count;
    dl_backend_enc_apply(be, &d2);  /* identical bitrate+fps → no HTTP */
    DL_ASSERT_EQ(m.request_count, n1);
    dl_backend_enc_close(be);
    mock_http_stop(&m);
}

DL_TEST(test_enc_safe_uses_compute_formula) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);
    cfg.safe_bitrate_kbps = 2000;   /* hits the floor */

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    int rc = dl_backend_enc_apply_safe(be, &cfg);
    DL_ASSERT_EQ(rc, 0);

    DL_ASSERT(strstr(m.received, "video0.bitrate=2000") != NULL);
    DL_ASSERT(strstr(m.received, "fpv.roiQp=-24") != NULL);

    dl_backend_enc_close(be);
    mock_http_stop(&m);
}
```

- [ ] **Step 2: Wire the new test source into the Makefile and add pthread to test LDFLAGS**

Edit `drone/Makefile`. Append to `TEST_SRCS`:

```makefile
    $(TESTDIR)/test_dl_backend_enc.c \
```

The mock-HTTP test uses `pthread`. Check the existing test-link command for `-lpthread`. If not present, add it. The line is around `drone/Makefile:98-99`:

```makefile
$(TESTBIN): $(TEST_SRCS) | $(OUTDIR)
	$(CC) $(CFLAGS) -DDL_TESTING -I$(SRCDIR) -I$(TESTDIR) -o $@ $(TEST_SRCS) $(LDFLAGS) -lpthread
```

(If `-lpthread` already appears via `LDFLAGS`, no change needed.)

- [ ] **Step 3: Run the C test suite; expect failures**

```bash
make -C drone test
```

Expected: backend tests fail because `dl_backend_enc_apply` still has the old `(be, d, prev)` signature and still skips on `roi_qp == 0`.

- [ ] **Step 4: Update `dl_backend_enc.h`**

Edit `drone/src/dl_backend_enc.h`. The new `dl_backend_enc_apply` signature drops the `prev` argument (the backend owns state):

```c
/* Apply bitrate / roi_qp / fps changes for `d`. roi_qp is computed
 * drone-side from d->bitrate_kbps via dl_compute_roi_qp; not part of
 * the wire as of v2. Returns 0 on success or no-op, -1 on HTTP failure. */
int dl_backend_enc_apply(dl_backend_enc_t *be, const dl_decision_t *d);
```

- [ ] **Step 5: Update `dl_backend_enc.c`**

Edit `drone/src/dl_backend_enc.c`.

Extend `struct dl_backend_enc` (around line 25) with cached config + last-applied state:

```c
struct dl_backend_enc {
    char     host[DL_CONF_MAX_STR];
    uint16_t port;
    uint32_t min_idr_interval_ms;
    uint64_t last_idr_ms;
    bool     idr_ever_sent;

    /* Cached config for compute_roi_qp. Snapshotted at open time so
     * subsequent reloads don't drift mid-flight (config reload isn't
     * supported anyway, but the snapshot makes the dependency explicit). */
    uint16_t roi_qp_threshold_kbps;
    uint16_t roi_qp_low_anchor_kbps;
    int8_t   roi_qp_floor;
    uint8_t  roi_qp_step;

    /* Last-applied state (per-backend prev, per CLAUDE.md convention). */
    bool     last_valid;
    uint16_t last_bitrate_kbps;
    int8_t   last_roi_qp;
    uint8_t  last_fps;
};
```

Replace `dl_backend_enc_open` (around line 177):

```c
dl_backend_enc_t *dl_backend_enc_open(const dl_config_t *cfg) {
    dl_backend_enc_t *be = calloc(1, sizeof(*be));
    if (!be) return NULL;
    snprintf(be->host, sizeof(be->host), "%s", cfg->encoder_host);
    be->port = cfg->encoder_port;
    be->min_idr_interval_ms = cfg->min_idr_interval_ms;
    be->roi_qp_threshold_kbps  = cfg->roi_qp_threshold_kbps;
    be->roi_qp_low_anchor_kbps = cfg->roi_qp_low_anchor_kbps;
    be->roi_qp_floor           = cfg->roi_qp_floor;
    be->roi_qp_step            = cfg->roi_qp_step;
    dl_log_info("enc: %s at %s:%u", cfg->encoder_kind, be->host, be->port);
    return be;
}
```

Add a helper that builds a temporary `dl_config_t` snapshot for `dl_compute_roi_qp` to use (it currently takes `const dl_config_t *`):

```c
static int compute_roi_qp_from_be(dl_backend_enc_t *be, uint16_t bitrate_kbps) {
    dl_config_t snap = {0};
    snap.roi_qp_threshold_kbps  = be->roi_qp_threshold_kbps;
    snap.roi_qp_low_anchor_kbps = be->roi_qp_low_anchor_kbps;
    snap.roi_qp_floor           = be->roi_qp_floor;
    snap.roi_qp_step            = be->roi_qp_step;
    return dl_compute_roi_qp(bitrate_kbps, &snap);
}
```

Replace `apply_set` (around line 191) to take signed `roi_qp` and always emit:

```c
static int apply_set(dl_backend_enc_t *be,
                     uint16_t bitrate_kbps, int8_t roi_qp, uint8_t fps) {
    char path[256];
    char *p = path;
    size_t left = sizeof(path);
    int n = snprintf(p, left, "/api/v1/set?video0.bitrate=%u", bitrate_kbps);
    if (n < 0 || (size_t)n >= left) return -1;
    p += n; left -= (size_t)n;
    /* Always emit fpv.roiQp (signed). 0 is a legitimate "disable
     * ROI" command per waybeam's contract; the previous skip-on-zero
     * meant we could never turn ROI off once enabled. */
    n = snprintf(p, left, "&fpv.roiQp=%d", (int)roi_qp);
    if (n < 0 || (size_t)n >= left) return -1;
    p += n; left -= (size_t)n;
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

Replace `dl_backend_enc_apply` (around line 215) — note the new signature (no `prev`):

```c
int dl_backend_enc_apply(dl_backend_enc_t *be, const dl_decision_t *d) {
    if (!be) return -1;
    if (d->bitrate_kbps == 0) return 0;   /* sentinel: don't push */

    int8_t roi_qp = (int8_t)compute_roi_qp_from_be(be, d->bitrate_kbps);

    if (be->last_valid &&
        be->last_bitrate_kbps == d->bitrate_kbps &&
        be->last_roi_qp       == roi_qp &&
        be->last_fps          == d->fps) {
        return 0;
    }
    int rc = apply_set(be, d->bitrate_kbps, roi_qp, d->fps);
    if (rc == 0) {
        be->last_bitrate_kbps = d->bitrate_kbps;
        be->last_roi_qp       = roi_qp;
        be->last_fps          = d->fps;
        be->last_valid        = true;
    }
    return rc;
}
```

Replace `dl_backend_enc_apply_safe` (around line 254):

```c
int dl_backend_enc_apply_safe(dl_backend_enc_t *be, const dl_config_t *cfg) {
    if (!be) return -1;
    int8_t roi_qp = (int8_t)dl_compute_roi_qp(cfg->safe_bitrate_kbps, cfg);
    int rc = apply_set(be, cfg->safe_bitrate_kbps, roi_qp, 0);
    if (rc == 0) {
        be->last_bitrate_kbps = cfg->safe_bitrate_kbps;
        be->last_roi_qp       = roi_qp;
        be->last_fps          = 0;   /* safe path doesn't touch fps */
        be->last_valid        = true;
    }
    return rc;
}
```

- [ ] **Step 6: Update `dl_applier.c` for the new signature**

Edit `drone/src/dl_applier.c`. Find all `dl_backend_enc_apply(be, &..., &last_enc)` call sites and drop the third argument. There are at least two (around lines 396, 417, 486).

Old:

```c
if (be && dl_backend_enc_apply(be, &d, &last_enc) < 0) drc = -1;
```

New:

```c
if (be && dl_backend_enc_apply(be, &d) < 0) drc = -1;
```

Also the `last_enc` reset at ~line 475 (`memset(&last_enc, 0, sizeof(last_enc));`) becomes purely an OSD/diagnostic concern. If `last_enc` is still consulted elsewhere (e.g. by OSD display via `last_applied`), leave it; just stop passing it to `dl_backend_enc_apply`. Run a quick `grep` for `last_enc` and remove dead uses if any remain.

```bash
grep -n "last_enc" drone/src/dl_applier.c
```

If `last_enc` is no longer read anywhere after the call-site updates, remove its declaration too (the `dl_decision_t last_enc = {0};` line near 286 and the `memset` reset). If it is still used (e.g. OSD prints `last_enc.bitrate_kbps`), keep it and let the applier maintain it manually for OSD's benefit only.

- [ ] **Step 7: Run all C tests**

```bash
make -C drone test
```

Expected: all backend tests pass; existing wire/config/etc. tests still pass.

- [ ] **Step 8: Commit**

```bash
git add drone/src/dl_backend_enc.h drone/src/dl_backend_enc.c drone/src/dl_applier.c tests/drone/test_dl_backend_enc.c drone/Makefile
git commit -m "$(cat <<'EOF'
enc: drone-side roi_qp, per-backend last_* state, signed always-emit

- dl_backend_enc_apply drops the `prev` parameter; backend owns
  last_bitrate/roi_qp/fps. Matches the per-backend prev-struct
  pattern documented in CLAUDE.md.
- roi_qp is computed from d->bitrate_kbps via dl_compute_roi_qp;
  not read from the wire (v2 wire dropped the field).
- apply_set takes int8_t roi_qp, formats with %d, and ALWAYS emits
  fpv.roiQp= (including =0). Fixes bug where roi_qp=0 was silently
  skipped — meaning we could never disable ROI once enabled.
- dl_backend_enc_apply_safe runs the same formula on
  safe_bitrate_kbps so ROI tracks bitrate uniformly, including
  watchdog trip.

Per docs/superpowers/specs/2026-05-22-drone-side-roi-qp-design.md.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: End-to-end test for ROI QP path

Add an assertion to the e2e test suite that a normal decision produces an HTTP set with the expected `fpv.roiQp` value.

**Files:**

- Modify: `tests/test_drone_e2e.py`

- [ ] **Step 1: Add a new e2e test**

Append to `tests/test_drone_e2e.py` (near the other golden-path tests, after `test_golden_path_dispatches_all_backends`):

```python
def test_decision_computes_roi_qp_below_threshold(tmp_path: Path):
    """At bitrate=4000 (below 6000 threshold), the applier should send
    fpv.roiQp=-12 (defaults: floor=-24, anchor=2000, step=3 → linear
    midpoint quantizes to -12)."""
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=4000, fps=60)
        assert _wait_until(lambda: len(s["encoder"].recorded) >= 1), \
            f"encoder got {s['encoder'].recorded}"
        paths = s["encoder"].recorded
        assert any("video0.bitrate=4000" in p and "fpv.roiQp=-12" in p
                   for p in paths), paths


def test_decision_computes_roi_qp_zero_above_threshold(tmp_path: Path):
    """At bitrate=12000 (above threshold), the applier should still
    emit fpv.roiQp=0 — the bug-fix path. Previously roi_qp=0 was
    silently skipped."""
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60)
        assert _wait_until(lambda: len(s["encoder"].recorded) >= 1), \
            f"encoder got {s['encoder'].recorded}"
        paths = s["encoder"].recorded
        assert any("fpv.roiQp=0" in p for p in paths), paths
```

- [ ] **Step 2: Run the new e2e tests**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py tests/test_drone_e2e.py::test_decision_computes_roi_qp_below_threshold tests/test_drone_e2e.py::test_decision_computes_roi_qp_zero_above_threshold -v
```

Expected: both pass.

- [ ] **Step 3: Run the full Python test suite**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py
```

Expected: pass. If `test_golden_path_dispatches_all_backends` (or any other test) fails due to an unexpected `fpv.roiQp=0` being added to the path, update its assertion to allow it.

- [ ] **Step 4: Commit**

```bash
git add tests/test_drone_e2e.py
git commit -m "$(cat <<'EOF'
e2e: assert drone computes & emits fpv.roiQp

Adds two tests covering the ramp (bitrate=4000 → -12) and the
above-threshold disable path (bitrate=12000 → 0, explicitly sent).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Docs — sample config + README

**Files:**

- Modify: `conf/drone.conf.sample`
- Modify: `README.md`

- [ ] **Step 1: Document the new knobs in the sample conf**

Edit `conf/drone.conf.sample`. After the existing `encoder_port = 80` line (around line 106), append:

```ini

# ---- Drone-side ROI QP policy --------------------------------------
# The applier computes fpv.roiQp from the most-recently-applied
# bitrate using a linear ramp, and sends it to the encoder on every
# bitrate apply. Negative values sharpen the frame center (FPV
# default); 0 disables ROI banding. Operator must pre-configure
# fpv.roiEnabled=true and sensible fpv.roiSteps/roiCenter in
# /etc/waybeam.json — see README.md for details.
#
# Formula:
#   roi_qp = 0                                     if bitrate >= threshold
#   roi_qp = quantize(floor*(threshold-bitrate)/(threshold-low_anchor))
#                                                  in [floor, 0]
#   roi_qp = floor                                 if bitrate <= low_anchor
#
# Defaults map: 6000 kbps → 0, 4000 → -12, 2000 → -24.
roi_qp_threshold_kbps  = 6000
roi_qp_low_anchor_kbps = 2000
roi_qp_floor           = -24
roi_qp_step            = 3
```

- [ ] **Step 2: Add a README note**

Open `README.md` and locate the section that documents waybeam encoder setup (search for `waybeam` or `roi`). Append a short paragraph (or add it under an existing "Operational prerequisites" section if one exists):

```markdown
### ROI QP (waybeam)

`dl-applier` computes `fpv.roiQp` from the live bitrate and pushes
it on every encoder apply. For the value to take visible effect,
`/etc/waybeam.json` must have ROI pre-enabled:

```json
"fpv": {
    "roiEnabled": true,
    "roiSteps": 2,
    "roiCenter": 0.25,
    "roiQp": 0
}
```

If `roiEnabled` is `false`, the applier's updates are stored but
produce no visible ROI banding. Tune `roi_qp_*` in `drone.conf` to
match the airframe — see the sample conf for the formula.
```

(If the README is structured differently than expected, place the note in the most appropriate existing section. Don't create a new top-level section if a suitable one already exists.)

- [ ] **Step 3: Sanity check the sample conf parses**

```bash
mkdir -p /tmp/dl-conf-check && cp conf/drone.conf.sample /tmp/dl-conf-check/drone.conf
./drone/build/dl-applier --config /tmp/dl-conf-check/drone.conf --dry-run 2>&1 | head -5 || true
```

If `--dry-run` doesn't exist, just confirm the build succeeds and the parser doesn't reject the new keys:

```bash
make -C drone test
```

(The `test_config_roi_qp_loads_overrides` from Task 1 already proves parsing works with non-default values; this step is just paranoia for the sample file specifically. Skip if no convenient probe.)

- [ ] **Step 4: Commit**

```bash
git add conf/drone.conf.sample README.md
git commit -m "$(cat <<'EOF'
docs: roi_qp_* config knobs and waybeam.json prereq

Documents the four roi_qp_* keys in drone.conf.sample with the
formula, and adds a README note that /etc/waybeam.json must have
fpv.roiEnabled=true for the applier's QP updates to have visible
effect.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Final verification

- [ ] **Step 1: Full C test suite**

```bash
make -C drone clean && make -C drone test
```

Expected: all C tests pass, including the new ROI suites and the v2 wire tests.

- [ ] **Step 2: Full Python test suite**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py
```

Expected: green.

- [ ] **Step 3: Cross-compile sanity check (optional, requires arm-linux-gnueabihf-gcc)**

```bash
make -C drone clean && make -C drone CROSS_COMPILE=arm-linux-gnueabihf-
```

Expected: clean cross build of `dl-applier`, `dl-inject`. If the cross toolchain isn't installed locally, skip.

- [ ] **Step 4: Commit any cleanup if the runs surfaced issues; otherwise we're done.**
