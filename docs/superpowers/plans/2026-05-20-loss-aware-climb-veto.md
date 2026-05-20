# Loss-aware climb veto — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent the leading-loop MCS selector from climbing while
any residual loss has been observed, by gating the existing
`up_confidence_count` accumulator on `signals.residual_loss_w == 0`,
and unifying the MCS=0 fallback recovery path with the normal climb
path so the same veto applies uniformly.

**Architecture:** Two surgical changes to `gs/dynamic_link/policy.py`:
(1) add a residual-loss reset to **both** confidence-counter
increment sites (the main upgrade branch at `LeadingSelector.select`
and the secondary `_try_confidence_feed` helper); (2) delete the
`elif cur == self.profile.mcs_min:` branch so MCS=0→1 climbs use the
same gate as every other step. Mark the now-unused
`hold_fallback_mode_ms` config field deprecated for back-compat.

**Tech Stack:** Python 3 (stdlib + dataclasses); pytest for unit
tests; PyYAML for config parsing.

**Reference spec:** `docs/superpowers/specs/2026-05-20-loss-aware-climb-veto-design.md`.

---

## Task 1: Residual-loss veto on the climb confidence counter

**Files:**
- Modify: `gs/dynamic_link/policy.py` (two sites: `_try_confidence_feed` at 308-323 and the upgrade branch at 465-481)
- Test: `tests/test_policy_leading.py`

- [ ] **Step 1: Write failing test for the main upgrade branch**

Append to `tests/test_policy_leading.py`:

```python
def test_climb_blocked_by_residual_loss():
    """Any nonzero residual loss must reset the climb confidence
    counter and clear the target candidate."""
    s = _selector(hysteresis_up_db=2.5, snr_safety_margin=3.0,
                  upward_confidence_loops=4, hold_modes_down_ms=0)
    # 3 clean ticks at SNR=20 (well above MCS=2 threshold).
    # Counter builds toward upward_confidence_loops=4.
    for i in range(3):
        _, _, changed = _select(s, snr=20.0, ts_ms=i * 100.0)
        assert not changed, f"climb fired prematurely at tick {i}"
    assert s.state.up_confidence_count == 3
    assert s.state.up_target_mcs == 2
    # Loss tick at the same SNR: counter resets, target clears.
    _, _, changed = _select(s, snr=20.0, loss=0.05, ts_ms=300.0)
    assert not changed
    assert s.state.up_confidence_count == 0
    assert s.state.up_target_mcs == -1
    assert any("climb_blocked" in r for r in s.reasons), \
        f"reasons={s.reasons}"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_policy_leading.py::test_climb_blocked_by_residual_loss -v`

Expected: FAIL — counter does not reset on loss (current code increments unconditionally), so `up_confidence_count == 4` and `changed=True`.

- [ ] **Step 3: Write failing test for the `_try_confidence_feed` shadow path**

Append to `tests/test_policy_leading.py`:

```python
def test_try_confidence_feed_blocked_by_residual_loss():
    """The _try_confidence_feed shadow-path (used when hysteresis blocks
    but candidate is still set) must also veto on loss."""
    s = _selector(hysteresis_up_db=10.0, snr_safety_margin=3.0,
                  upward_confidence_loops=4, hold_modes_down_ms=0)
    # SNR=20 puts candidate at MCS=2 (floor 14 → margin 6 dB), but
    # hysteresis_up_db=10 blocks the actual climb. _try_confidence_feed
    # still bumps the counter on clean ticks.
    for i in range(3):
        _select(s, snr=20.0, ts_ms=i * 100.0)
    assert s.state.up_confidence_count == 3
    assert s.state.up_target_mcs == 2
    # Loss tick: even on the shadow path, counter must reset.
    _select(s, snr=20.0, loss=0.05, ts_ms=300.0)
    assert s.state.up_confidence_count == 0
    assert s.state.up_target_mcs == -1
```

- [ ] **Step 4: Run both tests to verify they fail**

Run: `python3 -m pytest tests/test_policy_leading.py::test_climb_blocked_by_residual_loss tests/test_policy_leading.py::test_try_confidence_feed_blocked_by_residual_loss -v`

Expected: both FAIL.

- [ ] **Step 5: Implement the loss veto in the main upgrade branch**

In `gs/dynamic_link/policy.py`, locate the `else:` branch at line 465 (the comment is `# Upgrade: confidence-loop gating + hold timer.`). Replace lines 465-481 with:

```python
        else:
            # Upgrade: confidence-loop + hold timer + loss veto.
            if signals.residual_loss_w > 0:
                if st.up_confidence_count > 0:
                    self._reasons.append(
                        f"climb_blocked residual_loss="
                        f"{signals.residual_loss_w:.3f}"
                    )
                st.up_confidence_count = 0
                st.up_target_mcs = -1
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            if candidate != st.up_target_mcs:
                st.up_target_mcs = candidate
                st.up_confidence_count = 1
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            st.up_confidence_count += 1
            if st.up_confidence_count < self.sel.upward_confidence_loops:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            if elapsed_since_mcs_ms < self.sel.hold_modes_down_ms:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
```

**Note:** the `select(...)` method receives `loss_rate` as a parameter and builds an internal `signals` dataclass before reaching this code path. Verify by reading `_select_inner` / `select` around lines 327-400 — the field is referenced as a local in this function. If `signals` isn't a local at this point, use whatever local variable carries `loss_rate` (likely the `loss_rate` parameter directly). Search for `loss_rate` references in the function before editing.

**Concretely:** the existing code at line 357 reads `emergency = self._emergency_active(loss_rate, fec_pressure, link_starved)`. So `loss_rate` is the local. Use `loss_rate` in the patch, not `signals.residual_loss_w`:

```python
        else:
            # Upgrade: confidence-loop + hold timer + loss veto.
            if loss_rate > 0:
                if st.up_confidence_count > 0:
                    self._reasons.append(
                        f"climb_blocked residual_loss={loss_rate:.3f}"
                    )
                st.up_confidence_count = 0
                st.up_target_mcs = -1
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            if candidate != st.up_target_mcs:
                st.up_target_mcs = candidate
                st.up_confidence_count = 1
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            st.up_confidence_count += 1
            if st.up_confidence_count < self.sel.upward_confidence_loops:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            if elapsed_since_mcs_ms < self.sel.hold_modes_down_ms:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
```

- [ ] **Step 6: Implement the loss veto in `_try_confidence_feed`**

In `gs/dynamic_link/policy.py`, locate `_try_confidence_feed` at line 308. The caller passes only `target`, not `loss_rate`. Two options: (a) pass `loss_rate` as a new parameter; (b) read state from `self`. Option (a) is cleaner. Change the method signature and the single call site at line 413.

Replace the method (lines 308-323) with:

```python
    def _try_confidence_feed(self, target: int, loss_rate: float) -> None:
        """Bump the upward-confidence counter even when hysteresis blocks,
        so when conditions improve the upgrade fires quickly. Any
        nonzero loss vetoes the increment and resets the target — see
        docs/superpowers/specs/2026-05-20-loss-aware-climb-veto-design.md."""
        cur = self.state.current_mcs
        if cur < 0 or target <= cur:
            return
        # Cap by max_mcs_step_up.
        if self.gate.max_mcs_step_up > 0:
            target = min(target, cur + self.gate.max_mcs_step_up)
        if target == cur:
            return
        if loss_rate > 0:
            if self.state.up_confidence_count > 0:
                self._reasons.append(
                    f"climb_blocked residual_loss={loss_rate:.3f}"
                )
            self.state.up_confidence_count = 0
            self.state.up_target_mcs = -1
            return
        if target != self.state.up_target_mcs:
            self.state.up_target_mcs = target
            self.state.up_confidence_count = 1
        else:
            self.state.up_confidence_count += 1
```

Update the call site at line 413. Find it (the only call to `_try_confidence_feed`):

```python
                if (tgt_margin < self.gate.hysteresis_up_db
                        or predicted < 0):
                    # Block the upgrade — keep confidence bumping toward it.
                    self._try_confidence_feed(candidate)
```

Change to:

```python
                if (tgt_margin < self.gate.hysteresis_up_db
                        or predicted < 0):
                    # Block the upgrade — keep confidence bumping toward it.
                    self._try_confidence_feed(candidate, loss_rate)
```

- [ ] **Step 7: Run both new tests to verify they pass**

Run: `python3 -m pytest tests/test_policy_leading.py::test_climb_blocked_by_residual_loss tests/test_policy_leading.py::test_try_confidence_feed_blocked_by_residual_loss -v`

Expected: both PASS.

- [ ] **Step 8: Write positive test for clean-window climb**

Append to `tests/test_policy_leading.py`:

```python
def test_climb_proceeds_after_clean_window():
    """After a loss-induced reset, the climb fires once
    upward_confidence_loops consecutive clean ticks accumulate."""
    s = _selector(hysteresis_up_db=2.5, snr_safety_margin=3.0,
                  upward_confidence_loops=4, hold_modes_down_ms=0)
    # 2 clean ticks (counter → 2), 1 loss tick (counter → 0).
    _select(s, snr=20.0, ts_ms=0.0)
    _select(s, snr=20.0, ts_ms=100.0)
    assert s.state.up_confidence_count == 2
    _select(s, snr=20.0, loss=0.05, ts_ms=200.0)
    assert s.state.up_confidence_count == 0
    # Now 4 consecutive clean ticks → climb fires on the 4th.
    for i in range(3):
        _, _, changed = _select(s, snr=20.0, ts_ms=300.0 + i * 100.0)
        assert not changed, f"premature climb at tick {i}"
    _, _, changed = _select(s, snr=20.0, ts_ms=600.0)
    assert changed
    assert s.state.current_mcs == 2
```

- [ ] **Step 9: Run the positive test to verify it passes**

Run: `python3 -m pytest tests/test_policy_leading.py::test_climb_proceeds_after_clean_window -v`

Expected: PASS.

- [ ] **Step 10: Run the full leading-loop test file to check for regressions**

Run: `python3 -m pytest tests/test_policy_leading.py -v`

Expected: all tests pass *except* `test_hold_fallback_mode_ms_when_at_mcs_zero` (handled in Task 2). Note any other failures — if existing tests broke, the loss-veto logic might have a hole; investigate before continuing.

- [ ] **Step 11: Commit**

```bash
git add gs/dynamic_link/policy.py tests/test_policy_leading.py
git commit -m "$(cat <<'EOF'
policy: residual-loss veto on climb confidence counter

Gates both up_confidence_count increment sites
(LeadingSelector upgrade branch + _try_confidence_feed shadow path)
on residual_loss_w == 0. Any nonzero loss resets the counter and
clears the target candidate, with a "climb_blocked" reason string
appended for gs.jsonl visibility.

Spec: docs/superpowers/specs/2026-05-20-loss-aware-climb-veto-design.md
Motivation: flight-3/4 fast-IDR root-cause was climb-to-edge, not
apply-gap (confirmed by per-knob bench).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Unify MCS=0 path with the normal climb branch

**Files:**
- Modify: `gs/dynamic_link/policy.py:459-464` (delete the `elif cur == self.profile.mcs_min:` block)
- Test: `tests/test_policy_leading.py:359` (delete `test_hold_fallback_mode_ms_when_at_mcs_zero`)

- [ ] **Step 1: Write failing test for unified MCS=0 climb**

Append to `tests/test_policy_leading.py`:

```python
def test_climb_from_mcs_0_uses_confidence_loop():
    """After unification: MCS=0 → MCS=1 climbs go through the same
    confidence-loop gate as every other step. Specifically, the climb
    requires upward_confidence_loops clean ticks AND respects
    hold_modes_down_ms, NOT the legacy hold_fallback_mode_ms."""
    # hold_modes_down_ms=0 isolates the confidence-loop behavior;
    # hold_fallback_mode_ms is set high to prove it's ignored.
    s = _selector(hold_modes_down_ms=0, hold_fallback_mode_ms=999999,
                  upward_confidence_loops=4, max_mcs=5)
    # Force MCS to 0 via emergency starvation.
    ts = 0.0
    while s.state.current_mcs > 0:
        _select(s, snr=40.0, link_starved=True, ts_ms=ts)
        ts += 100.0
    # Settle and clear emergency flag.
    _select(s, snr=40.0, ts_ms=ts + 100.0)
    # First 3 clean ticks: confidence builds, no climb.
    for i in range(3):
        _, _, changed = _select(s, snr=40.0,
                                ts_ms=ts + 200.0 + i * 100.0)
        assert not changed, f"premature climb from MCS=0 at tick {i}"
    # 4th clean tick: climb fires.
    _, _, changed = _select(s, snr=40.0, ts_ms=ts + 500.0)
    assert changed
    assert s.state.current_mcs == 1
```

- [ ] **Step 2: Write failing test for MCS=0 climb blocked by loss**

Append to `tests/test_policy_leading.py`:

```python
def test_climb_from_mcs_0_blocked_by_loss():
    """The loss veto also gates MCS=0 → MCS=1 climbs."""
    s = _selector(hold_modes_down_ms=0, upward_confidence_loops=1,
                  max_mcs=5)
    # Force to MCS=0.
    ts = 0.0
    while s.state.current_mcs > 0:
        _select(s, snr=40.0, link_starved=True, ts_ms=ts)
        ts += 100.0
    # Clear emergency, then try to climb with nonzero loss.
    _select(s, snr=40.0, ts_ms=ts + 100.0)
    _, _, changed = _select(s, snr=40.0, loss=0.05,
                            ts_ms=ts + 1000.0)
    assert not changed
    assert s.state.current_mcs == 0
```

- [ ] **Step 3: Run new tests to verify they fail**

Run: `python3 -m pytest tests/test_policy_leading.py::test_climb_from_mcs_0_uses_confidence_loop tests/test_policy_leading.py::test_climb_from_mcs_0_blocked_by_loss -v`

Expected: both FAIL — current MCS=0 path skips the confidence loop and climbs after `hold_fallback_mode_ms`.

- [ ] **Step 4: Delete the MCS=0 special branch**

In `gs/dynamic_link/policy.py`, delete lines 459-464 (the entire `elif cur == self.profile.mcs_min:` block):

```python
        elif cur == self.profile.mcs_min:
            # Climbing out of fallback — extra-conservative hold.
            if elapsed_since_mcs_ms < self.sel.hold_fallback_mode_ms:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
```

The control flow now falls from the `elif is_downgrade:` block straight into the unified `else:` upgrade branch from Task 1.

- [ ] **Step 5: Delete the obsolete legacy test**

Delete `test_hold_fallback_mode_ms_when_at_mcs_zero` from `tests/test_policy_leading.py` (lines 359-373). This test explicitly validates the deleted behavior and would now fail.

- [ ] **Step 6: Run the new MCS=0 tests to verify they pass**

Run: `python3 -m pytest tests/test_policy_leading.py::test_climb_from_mcs_0_uses_confidence_loop tests/test_policy_leading.py::test_climb_from_mcs_0_blocked_by_loss -v`

Expected: both PASS.

- [ ] **Step 7: Run the full leading-loop test file**

Run: `python3 -m pytest tests/test_policy_leading.py -v`

Expected: all tests pass.

- [ ] **Step 8: Commit**

```bash
git add gs/dynamic_link/policy.py tests/test_policy_leading.py
git commit -m "$(cat <<'EOF'
policy: unify MCS=0 climb path with normal upgrade branch

Deletes the elif cur == mcs_min: special case. MCS=0 → 1 climbs now
use the same confidence-loop + hold_modes_down_ms gate (and the new
loss veto from the previous commit) as every other step.

Nominal MCS=0 recovery slows from ~1 s to ~2 s, but flight-3 data
showed the fast recovery was producing thrash (16 climbs + 15
demotions between 0 and 1 in one flight), not stable recovery.

Spec: docs/superpowers/specs/2026-05-20-loss-aware-climb-veto-design.md

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Deprecate `hold_fallback_mode_ms` config field

**Files:**
- Modify: `gs/dynamic_link/service.py:150-162` (YAML parser for `ProfileSelectionConfig`)
- Test: `tests/test_policy_leading.py`

- [ ] **Step 1: Write failing test for deprecation warning**

Append to `tests/test_policy_leading.py`:

```python
def test_deprecated_hold_fallback_mode_ms_parses(caplog):
    """Old gs.yaml files with hold_fallback_mode_ms still parse;
    a deprecation warning is logged so operators see the knob is
    no-op now."""
    import logging
    from dynamic_link.service import _build_policy_config  # adjust to actual exported helper

    raw = {
        "profile_selection": {
            "hold_fallback_mode_ms": 1500,
            "hold_modes_down_ms": 2000,
            "upward_confidence_loops": 4,
        },
        # Minimal required sections for the parser to not blow up.
        "fec": {}, "gate": {}, "smoothing": {}, "cooldown": {},
        "policy": {}, "safe_defaults": {},
        "leading_loop": {
            "radio_profile": "m8812eu2",
            "radio_profiles_dir": str(PACKAGED_DIR),
            "bandwidth": 20,
            "tx_power_min_dBm": 18,
            "tx_power_max_dBm": 28,
        },
    }
    with caplog.at_level(logging.WARNING, logger="dynamic_link"):
        cfg = _build_policy_config(raw)
    assert cfg.selection.hold_fallback_mode_ms == 1500  # still parsed
    assert any("hold_fallback_mode_ms" in r.message
               and "deprecated" in r.message.lower()
               for r in caplog.records), \
        f"no deprecation warning emitted; records={[r.message for r in caplog.records]}"
```

**Note:** the exact import path may need adjustment — verify by checking how `service.py` exposes its config builder. If there is no public `_build_policy_config`, factor one out as part of this task, or invoke whatever the existing public function is (likely `service.load_config` or similar). Read `gs/dynamic_link/service.py` lines 130-200 to confirm before writing the test.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_policy_leading.py::test_deprecated_hold_fallback_mode_ms_parses -v`

Expected: FAIL — no deprecation warning emitted today.

- [ ] **Step 3: Add deprecation warning in the YAML parser**

In `gs/dynamic_link/service.py`, locate the `ProfileSelectionConfig` construction (around lines 150-162). Add a deprecation check immediately before the dataclass construction. Insert the following lines after `selection_raw = …` (wherever the raw dict is fetched — read the surrounding code first):

```python
    if "hold_fallback_mode_ms" in selection_raw:
        log.warning(
            "profile_selection.hold_fallback_mode_ms is deprecated and no "
            "longer affects controller behavior — MCS=0 → 1 climbs now use "
            "the unified confidence-loop gate. Remove the key from gs.yaml."
        )
```

The `log` logger is already imported elsewhere in `service.py` (verify by grepping `^log = logging`). If not, add `log = logging.getLogger(__name__)` at the top of the file.

Leave the `ProfileSelectionConfig` dataclass field intact (still parsed, still stored on the config object — keeps `_selector()` test helpers and any in-flight YAML files working unchanged). Just stop using the value in the selector code (already done in Task 2).

- [ ] **Step 4: Run the deprecation test to verify it passes**

Run: `python3 -m pytest tests/test_policy_leading.py::test_deprecated_hold_fallback_mode_ms_parses -v`

Expected: PASS.

- [ ] **Step 5: Run the full pytest suite (excluding the wfb-ng-mavlink test as per CLAUDE.md)**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -q`

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add gs/dynamic_link/service.py tests/test_policy_leading.py
git commit -m "$(cat <<'EOF'
service: deprecate hold_fallback_mode_ms config knob

The knob is now a no-op since the MCS=0 climb path was unified with
the normal upgrade branch in the previous commit. Emit a deprecation
warning when present in gs.yaml so operators know to remove it. The
dataclass field stays parseable for backward compatibility with
in-flight YAML files.

Spec: docs/superpowers/specs/2026-05-20-loss-aware-climb-veto-design.md

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Remove `hold_fallback_mode_ms` from sample and deploy YAML

**Files:**
- Modify: `conf/gs.yaml.sample:94` (delete the `hold_fallback_mode_ms` line)
- Modify: `deploy/gs/gs.yaml` (delete the same line — line number may differ)

- [ ] **Step 1: Remove the key from `conf/gs.yaml.sample`**

In `conf/gs.yaml.sample`, delete this line (currently line 94):

```yaml
  hold_fallback_mode_ms: 1000       # extra hold when climbing out of MCS 0
```

The surrounding `profile_selection:` block keeps the remaining keys (`hold_modes_down_ms`, `min_between_changes_ms`, `fast_downgrade`, `upward_confidence_loops`).

- [ ] **Step 2: Remove the key from `deploy/gs/gs.yaml` if present**

Run: `grep -n "hold_fallback_mode_ms" deploy/gs/gs.yaml`

If a line is reported, delete it with the same edit pattern as Step 1. If nothing is reported, that file already does not include the key (it's untracked in git — see CLAUDE.md note about `deploy/` being gitignored — so its state on a given workstation may vary). Skip Step 2 in that case.

- [ ] **Step 3: Run the full pytest suite**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -q`

Expected: all tests pass.

- [ ] **Step 4: Run the leading-loop tests with `-v` for visibility**

Run: `python3 -m pytest tests/test_policy_leading.py -v`

Expected: every test in the file passes, including all four new tests from Tasks 1-3.

- [ ] **Step 5: Commit**

```bash
git add conf/gs.yaml.sample
# If deploy/gs/gs.yaml was modified AND is tracked (it usually isn't),
# add it too; otherwise the deploy/ change is local to the operator.
git commit -m "$(cat <<'EOF'
conf: remove deprecated hold_fallback_mode_ms from sample YAML

The knob is no-op as of the MCS=0 climb-path unification.
gs.yaml.sample reflects the canonical post-patch config surface.
Existing operator YAMLs with the key still parse (warning logged).

Spec: docs/superpowers/specs/2026-05-20-loss-aware-climb-veto-design.md

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Final verification sweep

**Files:**
- No code changes.

- [ ] **Step 1: Run the full Python suite**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -q`

Expected: every test passes, including:
- `test_climb_blocked_by_residual_loss` (Task 1)
- `test_try_confidence_feed_blocked_by_residual_loss` (Task 1)
- `test_climb_proceeds_after_clean_window` (Task 1)
- `test_climb_from_mcs_0_uses_confidence_loop` (Task 2)
- `test_climb_from_mcs_0_blocked_by_loss` (Task 2)
- `test_deprecated_hold_fallback_mode_ms_parses` (Task 3)

And no remaining reference to `test_hold_fallback_mode_ms_when_at_mcs_zero` (deleted in Task 2).

- [ ] **Step 2: Run the C drone-side suite (sanity — this patch is GS-only but the suite should still pass)**

Run: `make -C drone test`

Expected: all C tests pass. If they don't, the failures are unrelated to this patch (we touched no C code).

- [ ] **Step 3: Verify the wire format contract is unchanged**

Run: `python3 -m pytest tests/test_wire_contract.py -v`

Expected: PASS. We made no wire-format changes, so this is a sanity check that we didn't accidentally break it.

- [ ] **Step 4: Inspect the final git log for this branch**

Run: `git log --oneline -5`

Expected: 4 new commits on top of the design-spec commit (one per task that touched code; Task 5 has no commit). Commit subjects should be the ones from Tasks 1-4.

- [ ] **Step 5: No final commit needed**

If everything in Steps 1-4 passed, the implementation is complete. No additional commit.

---
