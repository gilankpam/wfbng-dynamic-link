# Vanilla wfb-ng support — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make dynamic-link runnable against upstream (vanilla) wfb-ng, which does not implement `CMD_SET_INTERLEAVE_DEPTH`. A drone-side config flag — declared to the GS via a new bit in HELLO `flags` — switches the controller into a no-interleaver mode where depth is pinned to 1 and the GS skips its depth state machine.

**Architecture:** Single capability bit (`DL_HELLO_FLAG_VANILLA_WFB_NG` = `0x01`) lives in the existing HELLO `flags` byte (no wire-version bump). The drone sets it when `interleaving_supported = 0` in `drone.conf`; the GS reads it into `DroneConfigState.interleaving_supported` and short-circuits the trailing-loop depth logic. Drone applier gates `CMD_SET_INTERLEAVE_DEPTH` emission on the same flag. A separate parallel widening of `gs/dynamic_link/stats_client.py` accepts the older contract version and a missing `interleave_depth` key, since the JSON stats feed is consumed before HELLO arrives.

**Tech Stack:** C11 (drone applier), Python 3 + asyncio + pytest (GS).

**Spec:** `docs/superpowers/specs/2026-05-14-vanilla-wfb-ng-support-design.md`

---

## Deviation from the spec

The spec calls for a predictor change (`predictor.py`: zero out the interleave term when vanilla). This plan **does not** include that change. Reasoning: with `depth` pinned to 1 by the trailing loop in vanilla mode, the predictor's `(proposal.depth - 1) * cfg.block_duration_ms` evaluates to `0 * 12.0 = 0.0` already. The change would be a no-op and would add a parameter to `predict()` / `fit_or_degrade()` for no behavioral benefit. The plan-level invariant: as long as the trailing loop guarantees `depth == 1` in vanilla mode, the predictor needs no awareness of the capability bit.

If a future change ever lets depth move when vanilla, that invariant should be re-checked and the predictor change added then.

---

## File map

**New files:** none.

**Files modified:**

| File | What changes |
|---|---|
| `drone/src/dl_wire.h` | Add `DL_HELLO_FLAG_VANILLA_WFB_NG` macro. |
| `gs/dynamic_link/wire.py` | Add `HELLO_FLAG_VANILLA_WFB_NG` constant. |
| `drone/src/dl_config.h` | Add `bool interleaving_supported` to `dl_config_t`. |
| `drone/src/dl_config.c` | Default to `true`; parse `interleaving_supported` key. |
| `drone/src/dl_hello.c` | OR the flag bit into HELLO when `!cfg->interleaving_supported`. |
| `drone/src/dl_backend_tx.c` | Store `interleaving_supported` on the backend struct; gate `send_depth` in `apply` and `apply_safe`. |
| `drone/src/dl_inject.c` | Add `--hello-flags` CLI option so the hex-diff contract test can drive non-zero `flags`. |
| `gs/dynamic_link/stats_client.py` | Accept `contract_version ∈ {1, 2}`; default missing `interleave_depth` to 1. |
| `gs/dynamic_link/drone_config.py` | Track `interleaving_supported` (derived from HELLO flags); log the mode on transition. |
| `gs/dynamic_link/policy.py` | `TrailingLoop.tick` accepts `interleaving_supported: bool = True`; in vanilla mode pins depth=1. `Policy.tick` passes it from `self.drone_config`. |
| `conf/drone.conf.sample` | Document `interleaving_supported`. |
| `tests/drone/test_config.c` | Default + parse cases. |
| `tests/drone/test_dl_hello.c` | Flag-bit emission cases. |
| `tests/drone/fixtures/` | New `drone_vanilla.conf` fixture for the e2e test (optional — built inline). |
| `tests/test_wire_contract.py` | HELLO contract case with `flags=0x01`. |
| `tests/test_stats_client.py` | Vanilla-feed compat cases. |
| `tests/test_drone_config.py` | Flag-derived `interleaving_supported`. |
| `tests/test_policy_trailing.py` | Vanilla-mode depth-pinned cases. |
| `tests/test_drone_e2e.py` | Vanilla-config scenario asserting opcode 5 never appears. |
| `README.md` | Operator-facing note. |
| `CLAUDE.md` | Add to "operational prerequisites" list. |
| `docs/dynamic-link-design.md` | New §2 subsection. |
| `drone/src/vendored/README.md` | Clarify vanilla forward-compat. |

---

## Implementation order

The order is chosen so each task lands a small, self-test-able piece. Wire format → drone config + applier → GS-side compat. Tests are written first per task.

### Task 1: Define the HELLO vanilla flag bit (C + Python constants)

**Files:**
- Modify: `drone/src/dl_wire.h` (add macro near `DL_FLAG_IDR_REQUEST`, around line 36)
- Modify: `gs/dynamic_link/wire.py` (add constant near `FLAG_IDR_REQUEST`, around line 41)
- Test: `tests/drone/test_wire.c` (new test asserting macro value)

- [ ] **Step 1: Write the failing C test**

Append to `tests/drone/test_wire.c`:

```c
DL_TEST(hello_flag_vanilla_macro_value) {
    /* Bit 0 of the HELLO flags byte. Wire-compatible because old drones
     * always set flags = 0, which means "capable" — matching today's
     * behavior. New vanilla drones OR this bit to declare themselves. */
    DL_ASSERT_EQ(DL_HELLO_FLAG_VANILLA_WFB_NG, 0x01u);
}
```

- [ ] **Step 2: Run the failing C test**

Run: `make -C drone test`
Expected: compile error — `DL_HELLO_FLAG_VANILLA_WFB_NG` undefined.

- [ ] **Step 3: Add the macro to `drone/src/dl_wire.h`**

Locate the existing `DL_FLAG_IDR_REQUEST` block (around line 35–36) and add the new macro directly after it:

```c
/* flag bits (decision packet) */
#define DL_FLAG_IDR_REQUEST 0x01u

/* HELLO flag bits. Bit 0 = "vanilla wfb-ng" — when set the drone is
 * running upstream wfb-ng (no CMD_SET_INTERLEAVE_DEPTH). Bit clear
 * (today's default) = the feat/interleaving_uep branch. */
#define DL_HELLO_FLAG_VANILLA_WFB_NG 0x01u
```

- [ ] **Step 4: Run C test to verify it passes**

Run: `make -C drone test`
Expected: all tests pass, including `hello_flag_vanilla_macro_value`.

- [ ] **Step 5: Write the failing Python test**

Append to `tests/test_wire_contract.py` (it's the closest existing home for HELLO encoding fixtures; the constant assertion belongs in a wire unit test, but the codebase doesn't currently have a dedicated `test_wire.py`, so we put the new symbol test inline here):

```python
def test_hello_flag_vanilla_constant():
    from dynamic_link.wire import HELLO_FLAG_VANILLA_WFB_NG
    assert HELLO_FLAG_VANILLA_WFB_NG == 0x01
```

- [ ] **Step 6: Run Python test to verify it fails**

Run: `python3 -m pytest tests/test_wire_contract.py::test_hello_flag_vanilla_constant -v --ignore=tests/test_mavlink_status.py`
Expected: ImportError on `HELLO_FLAG_VANILLA_WFB_NG`.

- [ ] **Step 7: Add the Python constant**

In `gs/dynamic_link/wire.py`, locate the `FLAG_IDR_REQUEST = 0x01` line (around line 41). Add directly after it:

```python
FLAG_IDR_REQUEST = 0x01

# HELLO flag bits — mirrors drone/src/dl_wire.h.
# Bit 0 = "vanilla wfb-ng" (no CMD_SET_INTERLEAVE_DEPTH). Bit clear =
# the feat/interleaving_uep branch (today's default).
HELLO_FLAG_VANILLA_WFB_NG = 0x01
```

- [ ] **Step 8: Run Python test to verify it passes**

Run: `python3 -m pytest tests/test_wire_contract.py::test_hello_flag_vanilla_constant -v --ignore=tests/test_mavlink_status.py`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add drone/src/dl_wire.h gs/dynamic_link/wire.py tests/drone/test_wire.c tests/test_wire_contract.py
git commit -m "$(cat <<'EOF'
wire: define DL_HELLO_FLAG_VANILLA_WFB_NG (HELLO flag bit 0)

C + Python constants for the new HELLO capability bit. No struct
changes — the flags byte at HELLO offset 5 already exists and was
always emitted as 0. Bit semantics: set = vanilla wfb-ng, clear =
custom branch (today's behavior preserved).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: Extend `dl-inject` so the contract test can set HELLO flags

**Files:**
- Modify: `drone/src/dl_inject.c` (add `--hello-flags` option)

The hex-diff contract test in `tests/test_wire_contract.py` calls `dl-inject --hello --dry-run …` and diffs the C bytes against Python's `encode_hello`. To exercise the new flag bit through that contract path we need a way to pass non-zero `hello.flags` on the CLI. The existing `--idr` flag wires only into the decision packet, not HELLO.

- [ ] **Step 1: Inspect existing option wiring**

Open `drone/src/dl_inject.c`. The pattern for adding an option:
- new entry in `static struct option opts[]` (~line 47)
- new character in the short-opts string (~line 100)
- new `case` in the switch (~line 100–138)

- [ ] **Step 2: Add the option (no test yet — wired up in Task 3's contract test)**

In `drone/src/dl_inject.c`, locate the `opts[]` table (around line 47–71) and add a new entry just after the `"build-sha"` line:

```c
        { "build-sha", required_argument, 0, 'S' },
        { "hello-flags", required_argument, 0, 'F' },
        { "help",      no_argument,       0, 'h' },
```

Update the `getopt_long` short-opts string (around line 100) — add `F:` after `S:`:

```c
    while ((c = getopt_long(argc, argv, "t:M:B:P:k:n:d:b:r:f:Is:Dpq:m:HAg:u:S:F:h",
                            opts, NULL)) != -1) {
```

Add the case handler after the `'S'` case (around line 130–134):

```c
            case 'S': {
                unsigned long v = strtoul(optarg, NULL, 0);
                hello.applier_build_sha = (uint32_t)v;
                break;
            }
            case 'F': {
                unsigned long v = strtoul(optarg, NULL, 0);
                hello.flags = (uint8_t)v;
                break;
            }
            case 'h': usage(argv[0]); return 0;
```

Also update the usage string (around line 26):

```c
        "       %s --hello --gen-id N --mtu N --fps N [--build-sha N] [--hello-flags N] --dry-run\n"
```

- [ ] **Step 3: Build to confirm it compiles**

Run: `make -C drone`
Expected: clean build, no warnings.

- [ ] **Step 4: Smoke-check the new flag**

Run: `drone/build/dl-inject --hello --dry-run --gen-id 0xCAFEBABE --mtu 3994 --fps 60 --build-sha 0xDEADBEEF --hello-flags 0x01`
Expected: 64-character hex string whose byte 5 is `01` (i.e. the 11th and 12th hex chars are `01`).

You can confirm by running it without the flag and diffing — bytes 4 and 5 should be `01 00` without `--hello-flags`, `01 01` with `--hello-flags 0x01` (byte 4 is the wire version = 1, byte 5 is the flags byte).

- [ ] **Step 5: Commit**

```bash
git add drone/src/dl_inject.c
git commit -m "$(cat <<'EOF'
dl-inject: add --hello-flags option

Lets the hex-diff contract test exercise non-zero hello.flags. Wired
the same way as existing flags; no behavior change for callers that
don't pass it.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: HELLO wire-contract test covering flags

**Files:**
- Test: `tests/test_wire_contract.py` (add one case)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_wire_contract.py`:

```python
def test_contract_hello_vanilla_flag():
    """Round-trip a HELLO with the vanilla bit set; C and Python encoders
    must produce byte-identical output."""
    from dynamic_link.wire import HELLO_FLAG_VANILLA_WFB_NG
    c_bytes = _dl_inject_hex(
        hello=True,
        gen_id="0xcafebabe",
        mtu=3994,
        fps=60,
        build_sha="0xdeadbeef",
        hello_flags=f"0x{HELLO_FLAG_VANILLA_WFB_NG:02x}",
    )
    py_bytes = encode_hello(
        Hello(generation_id=0xCAFEBABE,
              mtu_bytes=3994,
              fps=60,
              applier_build_sha=0xDEADBEEF,
              flags=HELLO_FLAG_VANILLA_WFB_NG)
    )
    assert c_bytes == py_bytes, (
        f"hello-vanilla mismatch:\n  C : {c_bytes.hex()}\n  Py: {py_bytes.hex()}"
    )
    # Byte 5 is the flags byte.
    assert py_bytes[5] == HELLO_FLAG_VANILLA_WFB_NG
```

- [ ] **Step 2: Run to verify it passes**

The C side already supports HELLO encoding with arbitrary `flags` (Task 2 wired the CLI option), and the Python side has supported `Hello(flags=...)` since Phase 4a. So this test should pass immediately — it's a regression guard, not a new behavior driver.

Run: `python3 -m pytest tests/test_wire_contract.py::test_contract_hello_vanilla_flag -v --ignore=tests/test_mavlink_status.py`
Expected: PASS.

If it fails, double-check that Task 2's `dl-inject --hello-flags` correctly stamps byte 5 of the output.

- [ ] **Step 3: Commit**

```bash
git add tests/test_wire_contract.py
git commit -m "$(cat <<'EOF'
test: HELLO wire-contract case for vanilla flag

Hex-diff a HELLO with flags=0x01 between dl-inject --dry-run and the
Python encoder. Anchors the contract that the new capability bit
travels on byte 5 with no wire-format bump.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: `dl_config_t.interleaving_supported` field

**Files:**
- Modify: `drone/src/dl_config.h` (add field)
- Modify: `drone/src/dl_config.c` (default + parse)
- Test: `tests/drone/test_config.c` (new test cases)

- [ ] **Step 1: Inspect existing test fixture style**

Open `tests/drone/test_config.c` and look for an existing test that asserts both a default value and a parsed override; reuse its pattern. Look in particular for tests like `test_config_defaults_*` and `test_config_parse_*`.

- [ ] **Step 2: Write the failing C tests**

Append to `tests/drone/test_config.c`:

```c
DL_TEST(config_interleaving_supported_default_true) {
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    DL_ASSERT_EQ(cfg.interleaving_supported, true);
}

DL_TEST(config_interleaving_supported_parses_zero) {
    /* Write a one-liner conf to a tmp file, load it, assert the
     * vanilla override took effect. */
    char path[] = "/tmp/dl_test_conf_XXXXXX";
    int fd = mkstemp(path);
    DL_ASSERT(fd >= 0);
    const char *content = "interleaving_supported = 0\n";
    DL_ASSERT_EQ((int)write(fd, content, strlen(content)), (int)strlen(content));
    close(fd);

    dl_config_t cfg;
    dl_config_defaults(&cfg);
    int rc = dl_config_load(path, &cfg);
    unlink(path);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(cfg.interleaving_supported, false);
}

DL_TEST(config_interleaving_supported_parses_one) {
    char path[] = "/tmp/dl_test_conf_XXXXXX";
    int fd = mkstemp(path);
    DL_ASSERT(fd >= 0);
    const char *content = "interleaving_supported = 1\n";
    DL_ASSERT_EQ((int)write(fd, content, strlen(content)), (int)strlen(content));
    close(fd);

    dl_config_t cfg;
    dl_config_defaults(&cfg);
    /* Flip default to false first so the parse must actually run to
     * leave us at true. */
    cfg.interleaving_supported = false;
    int rc = dl_config_load(path, &cfg);
    unlink(path);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(cfg.interleaving_supported, true);
}
```

If `test_config.c` doesn't already include `<unistd.h>` and `<stdlib.h>` at the top, add them.

- [ ] **Step 3: Run to verify failure**

Run: `make -C drone test`
Expected: compile error — `dl_config_t` has no `interleaving_supported` field.

- [ ] **Step 4: Add the field to `dl_config.h`**

In `drone/src/dl_config.h`, locate the section after the `safe_bitrate_kbps` field and before the `wlan_dev` field (around lines 47–58). Add a new section:

```c
    /* GS-link watchdog + safe_defaults. */
    uint32_t health_timeout_ms;
    uint8_t  safe_k;
    uint8_t  safe_n;
    uint8_t  safe_depth;
    uint8_t  safe_mcs;
    uint8_t  safe_bandwidth;
    int8_t   safe_tx_power_dBm;
    uint16_t safe_bitrate_kbps;

    /* Whether the underlying wfb_tx supports CMD_SET_INTERLEAVE_DEPTH.
     * True (default) on the feat/interleaving_uep branch. Set to false
     * for upstream/vanilla wfb-ng builds — the applier then never
     * emits opcode 5, and HELLO declares the capability to the GS via
     * DL_HELLO_FLAG_VANILLA_WFB_NG. */
    bool     interleaving_supported;

    /* Backends. */
    char     wlan_dev[DL_CONF_MAX_STR];
```

- [ ] **Step 5: Add the default + parse rule in `dl_config.c`**

In `drone/src/dl_config.c`, locate the `dl_config_defaults` block where `safe_bitrate_kbps` is set (around line 37). Add the default right after it:

```c
    cfg->safe_bitrate_kbps = 2000;
    cfg->interleaving_supported = true;
```

In the parse switch (around line 185, between `safe_bitrate_kbps` and `wlan_dev`), add:

```c
        else if (strcmp(key, "safe_bitrate_kbps") == 0)  SET_INT_RANGED(safe_bitrate_kbps, uint16_t, 100, 65535);
        else if (strcmp(key, "interleaving_supported") == 0) SET_BOOL(interleaving_supported);
        else if (strcmp(key, "wlan_dev") == 0)           SET_STR(wlan_dev);
```

- [ ] **Step 6: Run to verify it passes**

Run: `make -C drone test`
Expected: all C tests pass, including the three new cases.

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_config.h drone/src/dl_config.c tests/drone/test_config.c
git commit -m "$(cat <<'EOF'
drone: add interleaving_supported config field

New bool in dl_config_t, default true to preserve current behavior.
Parser accepts the standard 0/1/true/false/on/off bool forms. Subsequent
commits wire this through dl_hello (sets the HELLO flag bit) and
dl_backend_tx (gates send_depth).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: Drone HELLO declares the capability bit

**Files:**
- Modify: `drone/src/dl_hello.c` (set flag based on cfg)
- Test: `tests/drone/test_dl_hello.c`

- [ ] **Step 1: Inspect the existing hello-announce test**

Open `tests/drone/test_dl_hello.c`. The `hello_announcing_uses_initial_ms_for_first_retries` test (around line 47) shows how to drive `dl_hello_build_announce` to a buffer. We'll write a test that asserts the encoded HELLO's flags byte.

- [ ] **Step 2: Write the failing tests**

Append to `tests/drone/test_dl_hello.c`:

```c
DL_TEST(hello_announce_flags_zero_when_interleaving_supported) {
    dl_config_t cfg; setup_cfg(&cfg);
    cfg.interleaving_supported = true;
    dl_hello_sm_t h;
    DL_ASSERT_EQ(dl_hello_init(&h, &cfg), 0);
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    size_t n = dl_hello_build_announce(&h, buf, sizeof(buf));
    DL_ASSERT_EQ((int)n, DL_HELLO_ON_WIRE_SIZE);
    /* Byte 5 is the flags field. Capable drone → zero. */
    DL_ASSERT_EQ(buf[5], 0x00);
}

DL_TEST(hello_announce_flags_sets_vanilla_bit_when_unsupported) {
    dl_config_t cfg; setup_cfg(&cfg);
    cfg.interleaving_supported = false;
    dl_hello_sm_t h;
    DL_ASSERT_EQ(dl_hello_init(&h, &cfg), 0);
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    size_t n = dl_hello_build_announce(&h, buf, sizeof(buf));
    DL_ASSERT_EQ((int)n, DL_HELLO_ON_WIRE_SIZE);
    DL_ASSERT_EQ(buf[5] & DL_HELLO_FLAG_VANILLA_WFB_NG, DL_HELLO_FLAG_VANILLA_WFB_NG);
}
```

- [ ] **Step 3: Run to verify failure**

Run: `make -C drone test`
Expected: `hello_announce_flags_sets_vanilla_bit_when_unsupported` fails — currently `dl_hello.c` hard-codes `.flags = 0`.

- [ ] **Step 4: Wire `cfg->interleaving_supported` into the HELLO flags**

In `drone/src/dl_hello.c::dl_hello_build_announce` (around line 73), replace the hard-coded `.flags = 0` line. Current code:

```c
    dl_hello_t pkt = {
        .version = DL_WIRE_VERSION,
        .flags = 0,
        .generation_id = h->generation_id,
        ...
    };
```

Change to:

```c
    uint8_t flags = 0;
    if (h->cfg && !h->cfg->interleaving_supported) {
        flags |= DL_HELLO_FLAG_VANILLA_WFB_NG;
    }
    dl_hello_t pkt = {
        .version = DL_WIRE_VERSION,
        .flags = flags,
        .generation_id = h->generation_id,
        ...
    };
```

(Keep the existing `generation_id`, `mtu_bytes`, `fps`, `applier_build_sha` fields exactly as they were.)

The `dl_hello_sm_t` struct already holds `cfg` (see `dl_hello_init`), so no struct changes are needed.

- [ ] **Step 5: Run to verify pass**

Run: `make -C drone test`
Expected: all C tests pass.

- [ ] **Step 6: Log the mode at init**

Still in `drone/src/dl_hello.c::dl_hello_init`, the existing INFO log (around line 68) is:

```c
    dl_log_info("dl_hello: ANNOUNCING gen=0x%08x mtu=%u fps=%u",
                h->generation_id, h->mtu_bytes, h->fps);
```

Extend it to surface the mode so operators see it once at boot:

```c
    dl_log_info("dl_hello: ANNOUNCING gen=0x%08x mtu=%u fps=%u interleaver=%s",
                h->generation_id, h->mtu_bytes, h->fps,
                cfg->interleaving_supported ? "enabled" : "vanilla");
```

- [ ] **Step 7: Run C tests once more**

Run: `make -C drone test`
Expected: all tests still pass (log content isn't asserted).

- [ ] **Step 8: Commit**

```bash
git add drone/src/dl_hello.c tests/drone/test_dl_hello.c
git commit -m "$(cat <<'EOF'
hello: declare vanilla capability via flags byte

dl_hello_build_announce OR's DL_HELLO_FLAG_VANILLA_WFB_NG into the
flags byte when cfg->interleaving_supported is false. INFO log at
ANNOUNCING gains an interleaver=enabled|vanilla tag so the mode is
visible in flight logs without grepping config.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 6: Drone applier gates `CMD_SET_INTERLEAVE_DEPTH`

**Files:**
- Modify: `drone/src/dl_backend_tx.c` (store flag in struct; gate emits)
- Test: covered by Task 8's e2e test (a focused unit test against `dl_backend_tx` would require new mock plumbing; the e2e covers the same ground with the real binary and is already part of the suite).

- [ ] **Step 1: Store `interleaving_supported` on the backend struct**

In `drone/src/dl_backend_tx.c`, locate the `struct dl_backend_tx` definition (around lines 18–25):

```c
struct dl_backend_tx {
    int fd;
    struct sockaddr_in dst;
    useconds_t pace_us;
};
```

Add a new field:

```c
struct dl_backend_tx {
    int fd;
    struct sockaddr_in dst;
    useconds_t pace_us;
    bool interleaving_supported;
};
```

- [ ] **Step 2: Populate the field at open**

In `dl_backend_tx_open` (around line 119–162), after the existing `bt->pace_us = ...` line (around line 158):

```c
    bt->fd = fd;
    bt->dst = dst;
    bt->pace_us = (useconds_t)cfg->apply_sub_pace_ms * 1000u;
    bt->interleaving_supported = cfg->interleaving_supported;
    dl_log_info("tx_cmd: connected to %s:%u (sub_pace=%u ms, interleaver=%s)",
                cfg->wfb_tx_ctrl_addr, cfg->wfb_tx_ctrl_port,
                (unsigned)cfg->apply_sub_pace_ms,
                cfg->interleaving_supported ? "enabled" : "vanilla");
```

(Replace the existing `dl_log_info("tx_cmd: connected ...")` line so the mode shows up alongside the port + pace.)

- [ ] **Step 3: Gate `send_depth` in `dl_backend_tx_apply`**

In `dl_backend_tx_apply` (around line 218–241), the existing depth block is:

```c
    if (first || prev->depth != d->depth) {
        if (emitted) pace(bt);
        if (send_depth(bt, d->depth) < 0) rc = -1;
        emitted = true;
    }
```

Wrap it in the capability gate:

```c
    if (bt->interleaving_supported &&
        (first || prev->depth != d->depth)) {
        if (emitted) pace(bt);
        if (send_depth(bt, d->depth) < 0) rc = -1;
        emitted = true;
    }
```

- [ ] **Step 4: Gate `send_depth` in `dl_backend_tx_apply_safe`**

In `dl_backend_tx_apply_safe` (around line 243–251):

```c
int dl_backend_tx_apply_safe(dl_backend_tx_t *bt, const dl_config_t *cfg) {
    int rc = 0;
    if (send_fec(bt, cfg->safe_k, cfg->safe_n) < 0) rc = -1;
    pace(bt);
    if (send_depth(bt, cfg->safe_depth) < 0) rc = -1;
    pace(bt);
    if (send_radio(bt, cfg->safe_mcs, cfg->safe_bandwidth) < 0) rc = -1;
    return rc;
}
```

Change to:

```c
int dl_backend_tx_apply_safe(dl_backend_tx_t *bt, const dl_config_t *cfg) {
    int rc = 0;
    if (send_fec(bt, cfg->safe_k, cfg->safe_n) < 0) rc = -1;
    if (bt->interleaving_supported) {
        pace(bt);
        if (send_depth(bt, cfg->safe_depth) < 0) rc = -1;
    }
    pace(bt);
    if (send_radio(bt, cfg->safe_mcs, cfg->safe_bandwidth) < 0) rc = -1;
    return rc;
}
```

(The pace before `send_radio` is preserved regardless — only the depth-related pace is conditional.)

- [ ] **Step 5: Update the `apply_sub_pace_ms` comment in dl_config.h**

The existing comment in `drone/src/dl_config.h` (around lines 30–38) describes the "FEC/DEPTH/RADIO triplet". With this change, DEPTH is conditional. Update:

```c
    /* Pacing between sub-commands within a single apply phase: the
     * CMD_SET_* sub-calls inside dl_backend_tx_apply (FEC, optionally
     * DEPTH, RADIO) and the tx<->radio handoff inside the applier.
     * Both CMD_SET_FEC and CMD_SET_INTERLEAVE_DEPTH trigger wfb-ng's
     * refresh_session() which closes the open block, flushes the
     * interleaver and re-broadcasts SESSION packets; back-to-back
     * issues overlap those bursts and discard more in-flight video
     * than necessary. DEPTH is skipped when interleaving_supported
     * is false (vanilla wfb-ng). 0 disables pacing (legacy). */
    uint32_t apply_sub_pace_ms;
```

- [ ] **Step 6: Build and run the existing test suite to confirm no regression**

Run: `make -C drone && make -C drone test && python3 -m pytest tests/test_drone_e2e.py -v --ignore=tests/test_mavlink_status.py`
Expected: all existing tests pass (default `interleaving_supported=true` preserves behavior; the existing e2e test in `test_golden_path_dispatches_all_backends` still sees opcode 5 because the default config has `interleaving_supported=1`).

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_backend_tx.c drone/src/dl_config.h
git commit -m "$(cat <<'EOF'
backend_tx: gate CMD_SET_INTERLEAVE_DEPTH on interleaving_supported

dl_backend_tx_t now caches the capability from cfg at open time and
skips send_depth() in both apply() and apply_safe() when the underlying
wfb_tx doesn't support opcode 5. Apply log line surfaces the mode.
Default behavior unchanged (interleaving_supported defaults to true).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 7: Document the new key in `drone.conf.sample`

**Files:**
- Modify: `conf/drone.conf.sample`

- [ ] **Step 1: Add the documentation block**

In `conf/drone.conf.sample`, between the existing `# short_gi always false ...` comment (around line 69) and the `# ---- Backends ----` section (around line 71), insert:

```ini
# short_gi always false — pinned by the applier regardless of GS input.

# ---- Interleaver capability ----------------------------------------
# Whether the underlying wfb-ng supports the runtime
# CMD_SET_INTERLEAVE_DEPTH opcode. True only on the
# feat/interleaving_uep branch (vendored at drone/src/vendored/tx_cmd.h
# from upstream SHA 208ec1d).
#
# When 0:
#   - dl-applier never sends CMD_SET_INTERLEAVE_DEPTH.
#   - The drone reports DL_HELLO_FLAG_VANILLA_WFB_NG (bit 0 of the
#     HELLO flags byte); the GS pins depth=1 and short-circuits its
#     depth state machine.
#   - safe_depth above is harmless (never reaches wfb_tx).
#
# Mismatch warning: setting this to 1 against vanilla wfb_tx produces
# per-tick TX_APPLY_FAIL events on opcode 5 (visible via the debug
# log when debug_enable = 1).
interleaving_supported = 1

# ---- Backends -------------------------------------------------------
```

- [ ] **Step 2: Commit**

```bash
git add conf/drone.conf.sample
git commit -m "$(cat <<'EOF'
conf: document interleaving_supported in drone.conf.sample

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 8: End-to-end test — vanilla applier never emits opcode 5

**Files:**
- Test: `tests/test_drone_e2e.py` (new test)

- [ ] **Step 1: Add the test**

Append to `tests/test_drone_e2e.py` (place near `test_golden_path_dispatches_all_backends` so it sits with the other backend-coverage tests):

```python
def test_vanilla_mode_never_emits_set_interleave_depth(tmp_path: Path):
    """With interleaving_supported=0, the applier must NOT send opcode
    5 to wfb_tx — even when decisions arrive with depth != prev.depth.
    """
    with _sandbox(tmp_path, interleaving_supported=0) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"

        # First decision: depth=2 (would normally provoke opcode 5
        # since prev.depth starts at 1).
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60, sequence=1)
        # Second decision: depth=3 (would also provoke opcode 5 on
        # the diff path).
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=3,
                bitrate=12000, fps=60, sequence=2)

        # Wait for at least the FEC + RADIO commands to land (2 per
        # apply tick × 2 decisions = 4 expected; we wait for ≥4 so
        # the applier has had time to process both decisions).
        assert _wait_until(lambda: len(s["wfb"].received) >= 4), \
            f"wfb_tx got {s['wfb'].received}"

        cmd_ids = {r["cmd_id"] for r in s["wfb"].received}
        assert CMD_SET_FEC in cmd_ids
        assert CMD_SET_RADIO in cmd_ids
        assert CMD_SET_INTERLEAVE_DEPTH not in cmd_ids, \
            f"vanilla applier should never emit opcode 5, got {s['wfb'].received}"


def test_vanilla_mode_hello_sets_vanilla_flag(tmp_path: Path):
    """The HELLO emitted by a vanilla-config applier must have the
    vanilla bit set in byte 5."""
    from dynamic_link.wire import HELLO_FLAG_VANILLA_WFB_NG
    with _sandbox(tmp_path, interleaving_supported=0) as s:
        hello = s.recv_one_hello(timeout=3.0)
        assert hello is not None, "no HELLO arrived within 3 s"
        # Byte 5 is the flags byte.
        assert hello[5] & HELLO_FLAG_VANILLA_WFB_NG, \
            f"expected vanilla bit set, got byte5=0x{hello[5]:02x}"
```

- [ ] **Step 2: Run the new tests**

Run: `python3 -m pytest tests/test_drone_e2e.py::test_vanilla_mode_never_emits_set_interleave_depth tests/test_drone_e2e.py::test_vanilla_mode_hello_sets_vanilla_flag -v --ignore=tests/test_mavlink_status.py`
Expected: both PASS.

- [ ] **Step 3: Run the full drone e2e suite to confirm no regression**

Run: `python3 -m pytest tests/test_drone_e2e.py -v --ignore=tests/test_mavlink_status.py`
Expected: all pass.

- [ ] **Step 4: Commit**

```bash
git add tests/test_drone_e2e.py
git commit -m "$(cat <<'EOF'
test(e2e): vanilla applier omits CMD_SET_INTERLEAVE_DEPTH

Two new e2e cases:
  - interleaving_supported=0 → opcode 5 never reaches the mock wfb_tx,
    even when decisions carry depth != 1.
  - HELLO emitted by a vanilla applier has bit 0 set in the flags byte.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 9: GS stats client tolerates vanilla wfb-ng's stats feed

**Files:**
- Modify: `gs/dynamic_link/stats_client.py`
- Test: `tests/test_stats_client.py`

- [ ] **Step 1: Inspect existing stats-client test style**

Open `tests/test_stats_client.py`. Note two things:

- The public parser is `parse_record(raw: dict) -> Event | None` (imported in the existing test).
- There's an existing test `test_parse_rx_rejects_bad_contract_version` that asserts `parse_record(_rx_record(contract_version=1))` raises `ContractVersionError`. We are deliberately reversing that contract — `contract_version=1` must now be accepted. The existing test must be deleted/replaced as part of this task.

- [ ] **Step 2: Write the failing tests + remove the now-wrong existing test**

In `tests/test_stats_client.py`:

(a) **Delete** the existing `test_parse_rx_rejects_bad_contract_version` test (around lines 71–73). Its assertion is now wrong.

(b) **Add** new tests covering the widened behavior. Append:

```python
def test_parse_rx_accepts_contract_version_1():
    """Vanilla wfb-ng emits contract_version=1; we must accept it."""
    ev = parse_record(_rx_record(contract_version=1))
    assert isinstance(ev, RxEvent)
    assert ev.session is not None
    assert ev.session.contract_version == 1


def test_parse_rx_rejects_unknown_contract_version():
    """Versions outside {1, 2} still raise."""
    with pytest.raises(ContractVersionError):
        parse_record(_rx_record(contract_version=99))


def test_parse_session_defaults_interleave_depth_when_missing():
    """Vanilla wfb-ng's session record omits interleave_depth — default
    it to 1 rather than raising KeyError."""
    raw = {
        "type": "rx",
        "timestamp": 1.0,
        "id": "video rx",
        "tx_wlan": 0,
        "packets": {"all": [10, 100]},
        "rx_ant_stats": [],
        "session": {
            "fec_type": "VDM_RS",
            "fec_k": 8, "fec_n": 12, "epoch": 1,
            "contract_version": 1,
            # interleave_depth deliberately absent
        },
    }
    ev = parse_record(raw)
    assert isinstance(ev, RxEvent)
    assert ev.session is not None
    assert ev.session.interleave_depth == 1
```

Note: the existing `test_parse_new_session_rejects_bad_contract_version` (uses version 99) is still correct — leave it alone.

- [ ] **Step 3: Run to verify failure**

Run: `python3 -m pytest tests/test_stats_client.py -v --ignore=tests/test_mavlink_status.py`
Expected: `test_parse_rx_accepts_contract_version_1` fails (ContractVersionError), `test_parse_session_defaults_interleave_depth_when_missing` fails (KeyError on `interleave_depth`).

- [ ] **Step 4: Widen the version check and default `interleave_depth`**

In `gs/dynamic_link/stats_client.py`:

(a) Replace the single-value constant with a set. Locate (around line 19):

```python
CONTRACT_VERSION_EXPECTED = 2
```

Change to:

```python
# wfb-ng's contract_version field — bumped by upstream on
# session-record schema changes. Vanilla wfb-ng emits 1; the
# feat/interleaving_uep branch emits 2. We accept both because we
# decode the same minimal subset (fec_*, epoch) regardless.
CONTRACT_VERSIONS_SUPPORTED = frozenset({1, 2})
```

(b) Replace the strict equality at both call sites (around line 144 and line 165). Find:

```python
            if session.contract_version != CONTRACT_VERSION_EXPECTED:
                raise ContractVersionError(
                    f"contract_version={session.contract_version}, "
                    f"expected {CONTRACT_VERSION_EXPECTED}"
                )
```

Change to:

```python
            if session.contract_version not in CONTRACT_VERSIONS_SUPPORTED:
                raise ContractVersionError(
                    f"contract_version={session.contract_version}, "
                    f"supported {sorted(CONTRACT_VERSIONS_SUPPORTED)}"
                )
```

Make the same change at the second call site (the `new_session` branch around line 165).

(c) Also update the log message at line ~261 (`stats_client: contract_version mismatch; aborting`) to reference the supported set if it inlines the constant — otherwise leave it alone.

(d) In `_parse_session` (around line 101–109), make `interleave_depth` optional. Find:

```python
def _parse_session(d: dict) -> SessionInfo:
    return SessionInfo(
        fec_type=str(d.get("fec_type", "")),
        fec_k=int(d["fec_k"]),
        fec_n=int(d["fec_n"]),
        epoch=int(d["epoch"]),
        interleave_depth=int(d["interleave_depth"]),
        contract_version=int(d["contract_version"]),
    )
```

Change to:

```python
def _parse_session(d: dict) -> SessionInfo:
    return SessionInfo(
        fec_type=str(d.get("fec_type", "")),
        fec_k=int(d["fec_k"]),
        fec_n=int(d["fec_n"]),
        epoch=int(d["epoch"]),
        # Vanilla wfb-ng omits this key. Default to 1 (no interleaver).
        interleave_depth=int(d.get("interleave_depth", 1)),
        contract_version=int(d["contract_version"]),
    )
```

- [ ] **Step 5: Run new tests to confirm pass**

Run: `python3 -m pytest tests/test_stats_client.py -v --ignore=tests/test_mavlink_status.py`
Expected: all stats_client tests pass.

- [ ] **Step 6: Run the full pytest suite to confirm no other test relied on the strict version check**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py`
Expected: full pass. If any existing tests asserted on `CONTRACT_VERSION_EXPECTED` by name, update those references (search: `grep -rn CONTRACT_VERSION_EXPECTED tests/ gs/`) to use `CONTRACT_VERSIONS_SUPPORTED` instead.

- [ ] **Step 7: Commit**

```bash
git add gs/dynamic_link/stats_client.py tests/test_stats_client.py
git commit -m "$(cat <<'EOF'
stats_client: accept vanilla wfb-ng feeds

contract_version is now validated against {1, 2} instead of strict
==2. interleave_depth is optional on the session record (defaults to
1 when absent — vanilla wfb-ng doesn't emit it). The parser stays a
pure decoder; the controller's vanilla awareness comes from the HELLO
flag bit (see drone_config in the next task).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 10: GS `DroneConfigState` tracks `interleaving_supported`

**Files:**
- Modify: `gs/dynamic_link/drone_config.py`
- Test: `tests/test_drone_config.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_drone_config.py`:

```python
def test_default_interleaving_supported_is_true():
    """Before any HELLO arrives, assume capable (today's default)."""
    s = DroneConfigState()
    assert s.interleaving_supported is True


def test_hello_without_vanilla_flag_keeps_interleaving_supported_true():
    from dynamic_link.wire import Hello
    s = DroneConfigState()
    s.on_hello(Hello(generation_id=0xCAFEBABE,
                     mtu_bytes=3994, fps=60,
                     applier_build_sha=0xDEADBEEF,
                     flags=0))
    assert s.interleaving_supported is True


def test_hello_with_vanilla_flag_sets_interleaving_supported_false():
    from dynamic_link.wire import Hello, HELLO_FLAG_VANILLA_WFB_NG
    s = DroneConfigState()
    s.on_hello(Hello(generation_id=0xCAFEBABE,
                     mtu_bytes=3994, fps=60,
                     applier_build_sha=0xDEADBEEF,
                     flags=HELLO_FLAG_VANILLA_WFB_NG))
    assert s.interleaving_supported is False


def test_hello_flips_back_to_true_on_reboot_into_capable_build():
    """A drone rebuilt with the custom branch (new generation_id, flags=0)
    must clear the vanilla flag."""
    from dynamic_link.wire import Hello, HELLO_FLAG_VANILLA_WFB_NG
    s = DroneConfigState()
    s.on_hello(Hello(generation_id=0x1111, mtu_bytes=3994, fps=60,
                     flags=HELLO_FLAG_VANILLA_WFB_NG))
    assert s.interleaving_supported is False
    s.on_hello(Hello(generation_id=0x2222, mtu_bytes=3994, fps=60,
                     flags=0))
    assert s.interleaving_supported is True
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_drone_config.py -v --ignore=tests/test_mavlink_status.py`
Expected: all four new tests fail — attribute doesn't exist.

- [ ] **Step 3: Add the field + adoption logic**

In `gs/dynamic_link/drone_config.py`, locate the `DroneConfigState` dataclass (around lines 32–39). Add the field with a True default:

```python
@dataclass
class DroneConfigState:
    state: State = State.AWAITING
    generation_id: int | None = None
    mtu_bytes: int | None = None
    fps: int | None = None
    applier_build_sha: int | None = None
    # Derived from HELLO flags. True until a HELLO clears it; once a
    # vanilla-flagged HELLO arrives, stays False until a non-vanilla
    # HELLO replaces it (e.g. drone rebuilt with the custom branch and
    # rebooted → new generation_id, flags cleared).
    interleaving_supported: bool = True
```

Update the `_adopt` method (around lines 71–76) to read the flag:

```python
    def _adopt(self, h: Hello) -> None:
        from .wire import HELLO_FLAG_VANILLA_WFB_NG
        self.state = State.SYNCED
        self.generation_id = h.generation_id
        self.mtu_bytes = h.mtu_bytes
        self.fps = h.fps
        self.applier_build_sha = h.applier_build_sha
        self.interleaving_supported = (
            (h.flags & HELLO_FLAG_VANILLA_WFB_NG) == 0
        )
```

Update both log messages in `on_hello` (around lines 48–50 and 56–60) to include the interleaver mode:

```python
        if self.state is State.AWAITING:
            self._adopt(h)
            log.info(
                "drone_config sync gen=0x%08x mtu=%d fps=%d sha=0x%08x interleaver=%s",
                h.generation_id, h.mtu_bytes, h.fps, h.applier_build_sha,
                "enabled" if self.interleaving_supported else "vanilla",
            )
            return DroneConfigEvent.SYNCED
        # SYNCED
        if h.generation_id == self.generation_id:
            return DroneConfigEvent.ALREADY_SYNCED
        log.warning(
            "drone_reboot_detected old_gen=0x%08x new_gen=0x%08x "
            "new_mtu=%d new_fps=%d new_interleaver=%s",
            self.generation_id, h.generation_id, h.mtu_bytes, h.fps,
            "enabled" if (h.flags & HELLO_FLAG_VANILLA_WFB_NG) == 0 else "vanilla",
        )
```

(The second log emits the *incoming* HELLO's mode — we can't call `self.interleaving_supported` before `_adopt` runs.)

You'll need to import `HELLO_FLAG_VANILLA_WFB_NG` at the top of the file (or locally — already imported inline in `_adopt` above). Either is fine; prefer module-level import for the log message to avoid re-importing:

At the top of `drone_config.py` (around line 15):

```python
from .wire import Hello, HelloAck, HELLO_FLAG_VANILLA_WFB_NG
```

Then drop the local import from `_adopt`.

- [ ] **Step 4: Run new tests**

Run: `python3 -m pytest tests/test_drone_config.py -v --ignore=tests/test_mavlink_status.py`
Expected: all new tests pass; existing tests still pass.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/drone_config.py tests/test_drone_config.py
git commit -m "$(cat <<'EOF'
drone_config: track interleaving_supported from HELLO flags

DroneConfigState carries an interleaving_supported bool (default True,
flipped to False when HELLO byte 5 has the vanilla bit set). The
trailing loop will consume this in the next commit. Log lines on
sync / reboot now surface the mode.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 11: GS trailing loop pins depth in vanilla mode

**Files:**
- Modify: `gs/dynamic_link/policy.py` (TrailingLoop.tick + Policy.tick)
- Test: `tests/test_policy_trailing.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_policy_trailing.py`:

```python
def test_vanilla_mode_pins_depth_at_one_even_under_sustained_loss():
    """When the drone is vanilla wfb-ng, TrailingLoop must NOT raise depth
    even when both bootstrap and refine triggers fire."""
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    # First feed several windows so sustained_loss() trips, then a tick
    # with strong burst/holdoff signals. Without the vanilla guard this
    # would raise depth to 2.
    for i in range(cfg.sustained_loss_windows):
        tl.tick(
            _sigs(residual_loss_w=0.05, fec_work=0.20, ts=i * 0.1),
            current_depth=1, ts_ms=i * 100.0,
            interleaving_supported=False,
        )
    depth, idr = tl.tick(
        _sigs(residual_loss_w=0.05, fec_work=0.20,
              burst_rate=5.0, holdoff_rate=2.0, ts=1.0),
        current_depth=1, ts_ms=1000.0,
        interleaving_supported=False,
    )
    assert depth == 1, "vanilla mode must pin depth at 1"
    # IDR signalling still works in vanilla mode.
    assert idr is True


def test_vanilla_mode_does_not_step_down_below_one():
    """With current_depth=1 and vanilla, depth stays at 1 across a
    clean window streak."""
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    for i in range(cfg.clean_windows_for_depth_stepdown + 5):
        depth, _ = tl.tick(
            _idle_sigs(i * 100.0),
            current_depth=1, ts_ms=i * 100.0,
            interleaving_supported=False,
        )
        assert depth == 1


def test_capable_mode_still_raises_depth_after_change():
    """Regression: default (interleaving_supported=True kwarg or omitted)
    keeps the existing burst+holdoff bootstrap firing."""
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    depth, _ = tl.tick(
        _sigs(residual_loss_w=0.02, burst_rate=5.0,
              holdoff_rate=2.0, ts=0.3),
        current_depth=1, ts_ms=300.0,
        interleaving_supported=True,
    )
    assert depth == 2
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_policy_trailing.py -v --ignore=tests/test_mavlink_status.py -k vanilla`
Expected: TypeError — `tick()` got an unexpected keyword argument `interleaving_supported`.

- [ ] **Step 3: Add the kwarg + short-circuit to `TrailingLoop.tick`**

In `gs/dynamic_link/policy.py::TrailingLoop.tick` (around line 564–664), change the signature:

```python
    def tick(
        self,
        signals: Signals,
        current_depth: int,
        ts_ms: float,
        *,
        interleaving_supported: bool = True,
    ) -> tuple[int, bool]:
```

Insert the short-circuit at the very top of the method body, immediately after `self._reasons = []` (around line 576):

```python
        self._reasons = []

        if not interleaving_supported:
            # Vanilla wfb-ng: depth is structurally 1 (no interleaver
            # exists). Skip the entire depth state machine — bootstrap,
            # refine, and step-down all become no-ops. IDR signalling
            # still runs because it's independent of the interleaver.
            idr = signals.residual_loss_w > 0.0
            if idr:
                self._reasons.append(
                    f"residual_loss={signals.residual_loss_w:.3f} +IDR (vanilla)"
                )
            return 1, idr

        # ... existing logic continues unchanged ...
```

(Keep the existing body — `st = self.state`, `had_loss = ...`, all the way through to `return new_depth, idr`. Don't delete or reorder it.)

- [ ] **Step 4: Plumb it through `Policy.tick`**

In `gs/dynamic_link/policy.py::Policy.tick` (around line 858), locate:

```python
        new_depth, idr = self.trailing.tick(
            signals, self.state.depth, ts_ms,
        )
```

Change to:

```python
        new_depth, idr = self.trailing.tick(
            signals, self.state.depth, ts_ms,
            interleaving_supported=(
                self.drone_config.interleaving_supported
                if self.drone_config is not None
                else True
            ),
        )
```

(Default to True when no drone_config is wired — preserves existing test-suite behavior where many tests construct Policy without a drone_config.)

- [ ] **Step 5: Run new tests**

Run: `python3 -m pytest tests/test_policy_trailing.py -v --ignore=tests/test_mavlink_status.py`
Expected: all pass, including the three new vanilla cases.

- [ ] **Step 6: Run the full Python suite**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py`
Expected: full pass. Any test that constructs `TrailingLoop` directly will still work because `interleaving_supported` defaults to True.

- [ ] **Step 7: Commit**

```bash
git add gs/dynamic_link/policy.py tests/test_policy_trailing.py
git commit -m "$(cat <<'EOF'
policy: pin depth=1 when drone is vanilla wfb-ng

TrailingLoop.tick gains a keyword-only `interleaving_supported` param
(default True for backward compat). When false: skip the entire depth
state machine and return (1, idr) — IDR signalling is independent of
the interleaver so it still fires on residual loss.

Policy.tick reads the bit from self.drone_config and threads it through.
With no drone_config (test convenience), behavior is unchanged.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 12: Documentation updates

**Files:**
- Modify: `README.md`
- Modify: `CLAUDE.md`
- Modify: `docs/dynamic-link-design.md`
- Modify: `drone/src/vendored/README.md`

- [ ] **Step 1: README — operator-facing note**

Open `README.md`. Find the prerequisites section (look for the section listing `[common] log_interval = 100`, `control_port = 8000`, `wireless.mlink`, `video0.fps`). Add a fifth bullet:

```
5. **wfb-ng build**: if your drone is running upstream/vanilla wfb-ng
   (not the `feat/interleaving_uep` branch), set
   `interleaving_supported = 0` in `drone.conf`. The applier will then
   never emit `CMD_SET_INTERLEAVE_DEPTH` and the GS will pin
   `depth = 1`. See `docs/superpowers/specs/2026-05-14-vanilla-wfb-ng-support-design.md`
   for the full rationale.
```

(Phrasing/numbering: match whatever style the existing list uses — if it's a Markdown ordered list, use the same; if it's prose bullets, match that.)

- [ ] **Step 2: CLAUDE.md — operational prerequisites**

Open `CLAUDE.md`. Find the "Operational prerequisites (wfb-ng master.cfg changes)" section. Add a new item to the existing list (currently 4 items at the time of this plan):

```
5. `drone.conf` must set `interleaving_supported = 0` if the drone is
   running upstream/vanilla wfb-ng. Default is 1 (custom
   `feat/interleaving_uep` branch). Mismatch produces per-tick
   `TX_APPLY_FAIL` events on `set_interleave_depth` when set wrong on
   vanilla; degraded loss-recovery when set wrong on the custom branch.
```

- [ ] **Step 3: dynamic-link-design.md — new compatibility subsection**

Open `docs/dynamic-link-design.md`. Find §2 ("Why the asymmetry" or wherever wfb-ng coupling is discussed). Add a new subsection at an appropriate location near the end of §2 or beginning of §5:

```markdown
### Compatibility with vanilla wfb-ng

The controller can be run against upstream wfb-ng (without the
`feat/interleaving_uep` branch) by setting `interleaving_supported = 0`
in `drone.conf`. In this mode:

- The applier never emits `CMD_SET_INTERLEAVE_DEPTH` (opcode 5).
- The drone reports its mode to the GS via bit 0 of the HELLO `flags`
  byte (`DL_HELLO_FLAG_VANILLA_WFB_NG`); no wire-version bump.
- The GS pins `depth = 1` in every decision and short-circuits the
  trailing loop's depth state machine (§4.2 is a no-op in vanilla).
- The GS stats parser tolerates `contract_version = 1` and a missing
  `interleave_depth` key on the session record.

Loss recovery in vanilla mode relies on FEC + MCS adaptation alone —
the interleaver dimension is structurally unavailable.
```

- [ ] **Step 4: vendored/README.md — vanilla forward-compat note**

Open `drone/src/vendored/README.md`. After the "Local modifications" section, add (or extend an existing "compatibility" note if one exists):

```markdown
## Vanilla wfb-ng compatibility

The vendored header supports both wfb-ng builds:

- The `cmd_req_t` layouts for opcodes 1 (`CMD_SET_FEC`) and 2
  (`CMD_SET_RADIO`) are byte-identical between vanilla wfb-ng and the
  `feat/interleaving_uep` branch.
- Opcode 5 (`CMD_SET_INTERLEAVE_DEPTH`) is custom-branch only. The
  applier gates emission on `dl_config_t.interleaving_supported`, so
  vanilla deployments never put opcode 5 on the wire.

`WFB_IPC_CONTRACT_VERSION = 2` in this vendored copy reflects the
custom branch's value. Vanilla wfb-ng's JSON stats feed advertises
`contract_version = 1`; the GS accepts both (see
`gs/dynamic_link/stats_client.py::CONTRACT_VERSIONS_SUPPORTED`).
```

- [ ] **Step 5: Commit**

```bash
git add README.md CLAUDE.md docs/dynamic-link-design.md drone/src/vendored/README.md
git commit -m "$(cat <<'EOF'
docs: vanilla wfb-ng support

Operator-facing note in README + prerequisites entry in CLAUDE.md.
New compatibility subsection in the design doc plus a vanilla note in
the vendored tx_cmd.h README explaining which opcodes are
forward-compatible.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 13: Final full-suite verification

**Files:** none.

- [ ] **Step 1: Run the full Python test suite**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py`
Expected: all pass.

- [ ] **Step 2: Run the C test suite**

Run: `make -C drone test`
Expected: all pass.

- [ ] **Step 3: Cross-compile sanity check**

Run: `make -C drone clean && make -C drone CROSS_COMPILE=arm-linux-gnueabihf-`
Expected: clean build, no warnings. (Skip if the cross toolchain isn't installed on the dev host.)

- [ ] **Step 4: Smoke `dl-inject --dry-run` with vanilla flag**

Run: `drone/build/dl-inject --hello --dry-run --gen-id 0xCAFEBABE --mtu 3994 --fps 60 --hello-flags 0x01`
Expected: 64 hex chars; byte 5 (chars 10–11) is `01`.

- [ ] **Step 5: If any deploy/integration smoke step exists in this repo, run it**

Check for a deploy helper script (e.g. `deploy/*.sh`). If one exists and is appropriate for a non-functional change like this, no action needed — the spec is a pure code change and shouldn't disturb deploys.

If everything is green, the plan is complete. No additional commit (Task 12 closed out the docs).

---

## Self-review checklist

After implementing all tasks, verify:

- [ ] `grep -n "CONTRACT_VERSION_EXPECTED" gs/ tests/` returns no live references (only stale ones in comments are OK).
- [ ] `grep -n "interleaving_supported" drone/src/ gs/dynamic_link/ conf/ tests/` returns hits in: dl_config.h, dl_config.c, dl_hello.c (via cfg), dl_backend_tx.c, drone.conf.sample, drone_config.py, policy.py, test files.
- [ ] `grep -n "DL_HELLO_FLAG_VANILLA_WFB_NG" drone/src/ tests/drone/` shows the C macro in dl_wire.h and is referenced from tests + dl_hello.c.
- [ ] `grep -n "HELLO_FLAG_VANILLA_WFB_NG" gs/dynamic_link/ tests/` shows the Python constant in wire.py and is referenced from drone_config.py + tests.
- [ ] The existing `test_golden_path_dispatches_all_backends` still passes (capable default still emits opcode 5).
- [ ] No `--no-verify`, no `git config` changes, no force-push.
