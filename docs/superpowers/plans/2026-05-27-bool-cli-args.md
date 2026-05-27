# dl-applier Boolean CLI Flags Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let operators disable boolean fields from the dl-applier CLI (e.g. `--interleaving-supported=false`) without editing `drone.conf`, while keeping the existing bare `--flag` form working.

**Architecture:** Switch the bool entries in dl-applier's `getopt_long` table from `no_argument` to `optional_argument`. When `optarg == NULL`, set the field to `true` (back-compat); otherwise call the existing `dl_parse_bool` helper from `dl_config.c` (which already accepts `true/false/1/0/yes/no/on/off` case-insensitively). The helper must be unstaticed and prototyped in `dl_config.h`.

**Tech Stack:** C11, POSIX `getopt_long`, pytest end-to-end harness (`tests/test_drone_e2e.py::_sandbox`).

**Spec:** `docs/superpowers/specs/2026-05-27-bool-cli-args-design.md`

---

## File Structure

**Modified:**
- `drone/src/dl_config.h` — add prototype for `dl_parse_bool`.
- `drone/src/dl_config.c` — drop `static` qualifier on `dl_parse_bool`.
- `drone/src/dl_applier.c` — change bool option entries to `optional_argument`, update the bool case in the getopt loop, update help text.
- `tests/test_drone_cli_overrides.py` — add four new end-to-end tests.
- `README.md` — update the "Boolean fields are on-only switches" paragraph.
- `docs/superpowers/specs/2026-05-27-drone-cli-flags-design.md` — update §3 ("Booleans") to reflect the new surface.

**No new files.**

---

## Task 1: Expose `dl_parse_bool` from `dl_config.c`

`dl_parse_bool` already does exactly the value parsing we need
(`drone/src/dl_config.c:102-114`, accepts `true/false/1/0/yes/no/on/off`
case-insensitively). It is currently `static`. Unstatic it and add a
prototype so `dl_applier.c` can call it. Pure refactor — no behavior
change — so it is tested only via the existing `make -C drone test`
suite continuing to pass.

**Files:**
- Modify: `drone/src/dl_config.c:102`
- Modify: `drone/src/dl_config.h:153` (insert prototype after the
  existing field-table accessors)

- [ ] **Step 1: Drop the `static` qualifier on `dl_parse_bool`**

Edit `drone/src/dl_config.c`, change line 102 from:

```c
static int dl_parse_bool(const char *val, bool *out) {
```

to:

```c
int dl_parse_bool(const char *val, bool *out) {
```

- [ ] **Step 2: Add prototype in `dl_config.h`**

Edit `drone/src/dl_config.h`. After the line:

```c
const dl_str_field_t  *dl_config_str_fields (size_t *n_out);
```

(currently line 154), add a blank line and these lines:

```c
/* Parse a value string into a bool. Accepts true/false/1/0/yes/no/on/off
 * case-insensitively. Returns 0 on success, -1 on any other input.
 * Used by both the conf-file parser and the CLI override path. */
int dl_parse_bool(const char *val, bool *out);
```

- [ ] **Step 3: Build to confirm no breakage**

Run: `nix-shell --run 'make -C drone'`
Expected: clean build of `drone/build/dl-applier` and
`drone/build/dl-inject`, no new warnings.

- [ ] **Step 4: Run the C unit tests**

Run: `nix-shell --run 'make -C drone test'`
Expected: all tests pass (this change is invisible to existing
tests; they just confirm we didn't break the conf-file parser).

- [ ] **Step 5: Commit**

```bash
git add drone/src/dl_config.c drone/src/dl_config.h
git commit -m "$(cat <<'EOF'
refactor(dl_config): expose dl_parse_bool for CLI reuse

Drops the static qualifier and adds a prototype so dl_applier.c can
parse bool CLI override values with the same accepting set as the
conf-file parser. No behavior change.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Failing end-to-end test for `--flag=false`

Drive the design from a test first. Pick `--interleaving-supported=false`
as the canonical case: the existing
`tests/test_drone_e2e.py::test_vanilla_mode_never_emits_set_interleave_depth`
shows exactly how to assert "no opcode-5 emitted." That test gets
`interleaving_supported=0` via `_sandbox(...)` (which writes the
conf file). We mirror it but pass the flag through the CLI override
layer instead.

**Files:**
- Test: `tests/test_drone_cli_overrides.py` (append at end of file)

- [ ] **Step 1: Add the failing test**

Open `tests/test_drone_cli_overrides.py`. Add this imports line near
the top, merging it into the existing `from tests.test_drone_e2e import (...)` block:

```python
from tests.test_drone_e2e import (
    APPLIER,
    CMD_SET_FEC,
    CMD_SET_INTERLEAVE_DEPTH,
    CMD_SET_RADIO,
    _sandbox,
    _inject,
    _wait_until,
    build_drone,  # noqa: F401 — autouse session fixture
)
```

(Add `CMD_SET_FEC` and `CMD_SET_INTERLEAVE_DEPTH` to the existing
imports — the other names are already imported.)

Then append this test at the end of the file:

```python
# --------------------------------------------------------------------
# Boolean CLI overrides: --flag=true|false|1|0 (P-bool-cli-args).
# --------------------------------------------------------------------

def test_interleaving_supported_false_via_cli(tmp_path):
    """--interleaving-supported=false from the CLI must put the
    applier into vanilla mode: no CMD_SET_INTERLEAVE_DEPTH (opcode 5)
    ever emitted, even when the depth in a decision changes."""
    with _sandbox(
        tmp_path,
        cli_args=["--interleaving-supported=false"],
    ) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"

        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 2), \
            f"wfb_tx got {s['wfb'].received}"

        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=3,
                bitrate=12000, fps=60, sequence=2)
        time.sleep(0.25)

        cmd_ids = {r["cmd_id"] for r in s["wfb"].received}
        assert CMD_SET_FEC in cmd_ids
        assert CMD_SET_RADIO in cmd_ids
        assert CMD_SET_INTERLEAVE_DEPTH not in cmd_ids, \
            f"--interleaving-supported=false should suppress opcode 5; " \
            f"got {s['wfb'].received}"
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```
nix-shell --run 'python3 -m pytest tests/test_drone_cli_overrides.py::test_interleaving_supported_false_via_cli -v --ignore=tests/test_mavlink_status.py'
```

Expected: FAIL. The current applier registers `--interleaving-supported`
as `no_argument`, so `--interleaving-supported=false` triggers getopt
to emit `option '--interleaving-supported' doesn't allow an argument`
to stderr and exit with `parse_args -> -1`. The applier process exits
non-zero; `_sandbox` raises a timeout / connection error or the test
fails on `wfb_tx got []`.

- [ ] **Step 3: Do NOT commit yet** — failing tests should land in the same commit as the implementation. Move to Task 3.

---

## Task 3: Implement `optional_argument` + `dl_parse_bool` in the bool case

The two-line core change. Plus updating help text. Plus including the
new header.

**Files:**
- Modify: `drone/src/dl_applier.c:291` (option-table bool entry)
- Modify: `drone/src/dl_applier.c:317-326` (getopt bool case)
- Modify: `drone/src/dl_applier.c:245-251` (usage help text)
- Modify: `drone/src/dl_applier.c` includes (confirm dl_config.h
  already included — it is, via the `dl_config_*` calls in this file)

- [ ] **Step 1: Change bool option-table entry to `optional_argument`**

In `drone/src/dl_applier.c`, at line 291, change:

```c
    for (size_t i = 0; i < n_bool; i++)
        opts[k++] = (struct option){ xstrdup_kebab(tb[i].name), no_argument,       0, (int)(OPT_BOOL_BASE + i) };
```

to:

```c
    for (size_t i = 0; i < n_bool; i++)
        opts[k++] = (struct option){ xstrdup_kebab(tb[i].name), optional_argument, 0, (int)(OPT_BOOL_BASE + i) };
```

- [ ] **Step 2: Update bool case in the getopt loop**

In `drone/src/dl_applier.c`, replace the bool branch at lines 317-326:

```c
        else if (c >= OPT_BOOL_BASE && c < OPT_BOOL_BASE + (int)n_bool) {
            size_t i = (size_t)(c - OPT_BOOL_BASE);
            if (dl_config_set_bool_by_name(&out->overrides, tb[i].name, true) != 0) {
                fprintf(stderr, "--");
                print_kebab(tb[i].name);
                fprintf(stderr, ": internal error setting bool\n");
                free(opts); return -1;
            }
            bit_set(out->set_bool, i);
        }
```

with:

```c
        else if (c >= OPT_BOOL_BASE && c < OPT_BOOL_BASE + (int)n_bool) {
            size_t i = (size_t)(c - OPT_BOOL_BASE);
            bool v = true;
            if (optarg != NULL && dl_parse_bool(optarg, &v) != 0) {
                fprintf(stderr, "--");
                print_kebab(tb[i].name);
                fprintf(stderr, ": bad value '%s' (expected true|false|1|0)\n", optarg);
                free(opts); return -1;
            }
            if (dl_config_set_bool_by_name(&out->overrides, tb[i].name, v) != 0) {
                fprintf(stderr, "--");
                print_kebab(tb[i].name);
                fprintf(stderr, ": internal error setting bool\n");
                free(opts); return -1;
            }
            bit_set(out->set_bool, i);
        }
```

- [ ] **Step 3: Update usage help text**

In `drone/src/dl_applier.c`, replace the bool help block at lines
245-251:

```c
    const dl_bool_field_t *tb = dl_config_bool_fields(&n);
    fprintf(stderr, "\n  Boolean switches (set to true when passed):\n");
    for (size_t i = 0; i < n; i++) {
        fprintf(stderr, "    --");
        print_kebab(tb[i].name);
        fputc('\n', stderr);
    }
```

with:

```c
    const dl_bool_field_t *tb = dl_config_bool_fields(&n);
    fprintf(stderr, "\n  Boolean fields (--name or --name=true|false; default = true when no value):\n");
    for (size_t i = 0; i < n; i++) {
        fprintf(stderr, "    --");
        print_kebab(tb[i].name);
        fputc('\n', stderr);
    }
```

- [ ] **Step 4: Build**

Run: `nix-shell --run 'make -C drone'`
Expected: clean build, no warnings.

- [ ] **Step 5: Run the new test — expect PASS now**

Run:
```
nix-shell --run 'python3 -m pytest tests/test_drone_cli_overrides.py::test_interleaving_supported_false_via_cli -v --ignore=tests/test_mavlink_status.py'
```
Expected: PASS.

- [ ] **Step 6: Run the full CLI overrides test file — expect no regressions**

Run:
```
nix-shell --run 'python3 -m pytest tests/test_drone_cli_overrides.py -v --ignore=tests/test_mavlink_status.py'
```
Expected: all tests pass. In particular `test_dl_applier_boots_without_config`
(which uses bare `--mavlink-enable`) must still pass — that test
proves bare-flag back-compat.

- [ ] **Step 7: Run the C unit tests**

Run: `nix-shell --run 'make -C drone test'`
Expected: all pass.

- [ ] **Step 8: Commit (code + new test together)**

```bash
git add drone/src/dl_applier.c tests/test_drone_cli_overrides.py
git commit -m "$(cat <<'EOF'
feat(dl-applier): --flag=true|false|1|0 for bool CLI overrides

Bool CLI flags become optional_argument: bare --flag still sets the
field to true (back-compat), while --flag=false (or =0, =no, =off)
disables it. Reuses dl_parse_bool from dl_config.c so CLI and conf
file accept the same value spellings.

Primary use case: --interleaving-supported=false to flip a drone
into vanilla wfb-ng mode without editing drone.conf. Caveat documented
in the spec: only the = form works (--flag value would treat 'value'
as a positional — a getopt_long optional_argument limitation).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Coverage tests for `=0`, bare-flag back-compat, error path

Three more focused tests, each pinning one promise from the spec.

**Files:**
- Modify: `tests/test_drone_cli_overrides.py` (append below the test
  added in Task 2)

- [ ] **Step 1: Add the `=0` alias test**

Append to `tests/test_drone_cli_overrides.py`:

```python
def test_interleaving_supported_zero_via_cli(tmp_path):
    """--interleaving-supported=0 must be equivalent to =false."""
    with _sandbox(
        tmp_path,
        cli_args=["--interleaving-supported=0"],
    ) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"

        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 2), \
            f"wfb_tx got {s['wfb'].received}"

        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=3,
                bitrate=12000, fps=60, sequence=2)
        time.sleep(0.25)

        cmd_ids = {r["cmd_id"] for r in s["wfb"].received}
        assert CMD_SET_INTERLEAVE_DEPTH not in cmd_ids, \
            f"--interleaving-supported=0 should suppress opcode 5; " \
            f"got {s['wfb'].received}"
```

- [ ] **Step 2: Add the bare-flag back-compat test**

Append:

```python
def test_bare_flag_still_means_true(tmp_path):
    """Bare --osd-enable (no value) must set the field to true,
    matching the pre-change behavior. We assert by running the
    full _sandbox lifecycle and confirming the applier boots and
    a CMD_SET_RADIO (safe_defaults push) arrives — proxy for
    'parse_args returned 0 and the watchdog is running'."""
    with _sandbox(
        tmp_path,
        cli_args=["--osd-enable"],
    ) as s:
        _wait_for_safe_radio(s["wfb"])
```

- [ ] **Step 3: Add the bad-value error test**

Append:

```python
def test_bool_bad_value_exits_nonzero(tmp_path):
    """--osd-enable=garbage must exit non-zero with a clear error
    on stderr. We invoke dl-applier directly (no _sandbox) since
    the process must crash before binding anything."""
    cfg = tmp_path / "drone.conf"
    cfg.write_text("listen_addr = 127.0.0.1\nlisten_port = 0\n")

    out = subprocess.run(
        [str(APPLIER), "--config", str(cfg), "--osd-enable=garbage"],
        capture_output=True, text=True, timeout=5,
    )
    assert out.returncode != 0, (
        f"expected non-zero exit, got rc={out.returncode}, "
        f"stderr={out.stderr!r}"
    )
    assert "--osd-enable" in out.stderr
    assert "bad value" in out.stderr
    assert "garbage" in out.stderr
```

- [ ] **Step 4: Run the three new tests**

Run:
```
nix-shell --run 'python3 -m pytest tests/test_drone_cli_overrides.py::test_interleaving_supported_zero_via_cli tests/test_drone_cli_overrides.py::test_bare_flag_still_means_true tests/test_drone_cli_overrides.py::test_bool_bad_value_exits_nonzero -v --ignore=tests/test_mavlink_status.py'
```
Expected: all three PASS.

- [ ] **Step 5: Run the full pytest suite**

Run:
```
nix-shell --run 'python3 -m pytest --ignore=tests/test_mavlink_status.py'
```
Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add tests/test_drone_cli_overrides.py
git commit -m "$(cat <<'EOF'
test(dl-applier): coverage for bool CLI =0, bare flag, bad value

Three focused tests pinning the spec promises beyond the canonical
=false case: =0 alias works, bare --flag still means true (back-compat),
and --flag=garbage exits non-zero with the documented error.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Update user-facing docs

Two doc surfaces describe the old "on-only" behavior. Both need to be
updated to describe the new form and to call out the `=`-only caveat.

**Files:**
- Modify: `README.md:187-189`
- Modify: `docs/superpowers/specs/2026-05-27-drone-cli-flags-design.md`
  (§3 "Booleans")

- [ ] **Step 1: Update README**

Open `README.md`. Replace lines 187-189:

```
Boolean fields are on-only switches: `--mavlink-enable` sets the
field to `true`. There is no `--no-mavlink-enable`; to force a
default-true field off, set it in the conf file.
```

with:

```
Boolean fields take an optional value: `--mavlink-enable` (bare) sets
the field to `true`; `--mavlink-enable=false` (or `=0`, `=no`, `=off`,
case-insensitive) sets it to `false`. The `=` form is required — a
space-separated value (`--mavlink-enable false`) does not work, because
`getopt_long` treats it as a positional argument. The most common
use is `--interleaving-supported=false` to flip a drone running
vanilla wfb-ng without editing `drone.conf`.
```

- [ ] **Step 2: Update the earlier CLI-flags spec**

Open `docs/superpowers/specs/2026-05-27-drone-cli-flags-design.md`.
Find §3 ("Booleans") — currently around lines 46-48 — which reads:

```
3. **Booleans:** on-only switches. `--osd-debug-latency`,
   `--mavlink-enable`, etc. all flip their target to `true` when
   passed. ...
```

Replace that bullet's body (the "on-only switches..." sentence and
anything in the same bullet describing the disable workaround) with:

```
3. **Booleans:** value-taking via `--name` (bare, = true) or
   `--name=true|false|1|0` (case-insensitive; `yes/no/on/off` also
   accepted). The `=` form is required for explicit values; a
   space-separated value is parsed as a positional. Updated
   2026-05-27 — see `docs/superpowers/specs/2026-05-27-bool-cli-args-design.md`.
```

(If §3's text differs slightly from the snippet above, preserve the
surrounding structure and only swap the description of how bools
work. The previous wording explicitly noted "there is no `--no-flag`"
and "to disable a default-true field, set it in the conf file" — both
of those statements are now stale and should be removed.)

- [ ] **Step 3: Verify nothing else in the docs still says "on-only"**

Run:
```
grep -rn "on-only\|on-only switches\|no --no-\|no \`--no-" README.md docs/
```
Expected: no surviving references to the old behavior. If anything
turns up, update it.

- [ ] **Step 4: Commit**

```bash
git add README.md docs/superpowers/specs/2026-05-27-drone-cli-flags-design.md
git commit -m "$(cat <<'EOF'
docs: --flag=true|false for dl-applier bool CLI overrides

Updates the README and the earlier per-field CLI-flags spec to
describe the new bool surface. The on-only switch language is
gone; the = form requirement and the canonical use case
(--interleaving-supported=false) are called out.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Final Verification

- [ ] **Step 1: Full pytest suite**

Run:
```
nix-shell --run 'python3 -m pytest --ignore=tests/test_mavlink_status.py'
```
Expected: all pass.

- [ ] **Step 2: C unit tests**

Run: `nix-shell --run 'make -C drone test'`
Expected: all pass.

- [ ] **Step 3: Clean build (no warnings)**

Run: `nix-shell --run 'make -C drone clean && make -C drone'`
Expected: clean build of `dl-applier` and `dl-inject`, zero warnings.

- [ ] **Step 4: Manual smoke — bare flag prints OK**

Run: `nix-shell --run 'drone/build/dl-applier --help 2>&1 | grep -A2 "Boolean fields"'`
Expected: help text shows the new "Boolean fields (--name or
--name=true|false; default = true when no value):" header.

- [ ] **Step 5: Manual smoke — bad value produces the documented error**

Run: `nix-shell --run 'drone/build/dl-applier --osd-enable=garbage 2>&1'`
Expected output contains: `--osd-enable: bad value 'garbage' (expected true|false|1|0)`
Exit code: non-zero.

- [ ] **Step 6: Confirm commit graph**

Run: `git log --oneline -6`
Expected (top to bottom):
1. docs: --flag=true|false for dl-applier bool CLI overrides
2. test(dl-applier): coverage for bool CLI =0, bare flag, bad value
3. feat(dl-applier): --flag=true|false|1|0 for bool CLI overrides
4. refactor(dl_config): expose dl_parse_bool for CLI reuse
5. docs: spec for dl-applier --flag=true|false bool CLI args (already landed)
6. ... (older history)
