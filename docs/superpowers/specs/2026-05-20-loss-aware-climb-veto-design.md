# Loss-aware climb veto

Status: design
Owner: Gilang
Date: 2026-05-20

## Summary

Make the leading-loop MCS selector refuse to climb when **any
residual loss has been observed**, by gating the existing
`up_confidence_count` accumulator on `signals.residual_loss_w == 0`.
Also unify the MCS=0 fallback recovery path with the normal upgrade
path so the same veto applies uniformly to every climb step.

Goals:

- Eliminate the dominant production failure mode where the controller
  climbs to a too-aggressive MCS at marginal SNR and the first
  post-change stats window registers real residual loss, triggering
  an IDR and an emergency demotion (the "climb to edge" pattern).
- Apply the veto uniformly across all MCS levels including 0→1,
  removing the historical special case for fallback recovery.
- Add zero new config knobs. Reuse the existing
  `upward_confidence_loops` cadence.
- Surface vetoes in `gs.jsonl` so operators can see the gate firing.

Non-goals:

- Suppressing IDR requests after MCS changes (apply-gap mask). The
  per-knob and cascade bench showed apply transitions do not lose
  fragments; previous-versions of this fix targeted a problem that
  does not exist.
- Climbing slower for the sake of stability. The fix is loss-driven,
  not generic conservatism — climb timing in clean conditions is
  unchanged (still `upward_confidence_loops` × 100 ms = 1.2 s).
- Tuning `loss_margin_weight` or `fec_margin_weight`. The existing
  SNR-margin stress mechanism stays as-is and composes with the
  veto.

## Background

Production flight logs (`debug/flight-0003`, `flight-0004`) showed
40-50 % of non-emergency IDR requests firing within 200 ms of an
MCS-up change, and ~80 % of those after a climb (not a demotion).
Hypothesis at the time: "apply-gap" — the brief TX pause during a
wfb_tx MCS retune loses queued fragments.

A bench harness
(`tests/bench/per_knob_loss_bench.py`,
docs at `docs/per-knob-loss-bench.md`) was built to isolate
which backend (mcs / bitrate / tx_power / FEC) causes the
post-change loss. Three runs at increasingly marginal RSSI
(tx_power=22 → 10, RSSI −45 dBm → −82 dBm) produced **0 lost
fragments across 271 trials**. A subsequent cascade variant
(`tests/bench/cascade_bench.py` style — not committed) confirmed
that the production-pattern "full retune" cascade (MCS + bitrate
+ tx_power + FEC moving together per profile row) also produces
near-zero loss (3 lost packets / 30 cascade changes), but a
deliberately-mismatched "MCS-only" cascade (MCS jumps to 5 while
FEC stays at k=4 / n=6) produces ~10 lost fragments per trial.

Conclusion: the apply transition does not cost fragments in any
realistic configuration. The production fast-IDR pattern must
therefore be **real loss on the newly-selected MCS** — the climb
chose a modulation that the current SNR cannot reliably decode,
and the first post-change stats window catches the genuine fallout.
The fix must prevent the climb decision, not mask the resulting
IDR.

## Design

### Current behavior

`policy.py`, in the leading-loop selector
(`LeadingLoop._select_target_mcs` / `_apply_mcs_change` path,
lines 444-510), has three climb branches:

1. **Same-MCS** (`candidate == cur`): clears `up_confidence_count`,
   returns no change.
2. **Downgrade**: either fast-path (emergency or `fast_downgrade`)
   or slow-path with `hold_modes_down_ms` timer. Loss-veto
   irrelevant here.
3. **Climb out of MCS=0** (`cur == mcs_min`): waits
   `hold_fallback_mode_ms` (1 s), then climbs unconditionally on
   the next tick. **No confidence loop, no loss check.**
4. **Normal climb** (every other up-direction): builds
   `up_confidence_count` on consecutive ticks where the same
   candidate stays eligible by SNR margin; emits the climb when
   the count reaches `upward_confidence_loops`.

The leading loop already uses `loss_margin_weight: 20` and
`fec_margin_weight: 5` to *widen* the SNR margin under observed
stress. But these are soft: they make the climb harder, not
impossible. At high SNR (the typical marginal-RSSI-but-decent-SNR
case), the climb proceeds regardless of recent loss.

### Patched behavior

Two changes, in the same code site:

**1. Unify the MCS=0 path with the normal climb branch.**

Delete the `elif cur == self.profile.mcs_min:` block (lines 459-464).
MCS=0→1 climbs now go through the same `else:` branch as every other
upgrade — confidence loop + `hold_modes_down_ms` dwell.

Trade-off: nominal MCS=0 recovery time goes from ~1 s to ~2 s
(the `hold_modes_down_ms = 2 s` timer becomes binding). The
historical optimization was justified as "minimize time at
degraded video", but flight-3 data showed 16 climbs from 0→1 and
15 demotions back in a single flight — the fast recovery was
producing thrash, not stable recovery. Slower-but-correct beats
faster-but-bouncing.

**2. Add a residual-loss veto on the climb confidence counter.**

In the unified upgrade branch, add a check before incrementing the
counter:

```python
else:
    # Upgrade: confidence-loop + hold timer + loss veto.
    if candidate != st.up_target_mcs:
        st.up_target_mcs = candidate
        st.up_confidence_count = 1 if signals.residual_loss_w == 0 else 0
        return cur, tx, False

    if signals.residual_loss_w > 0:
        if st.up_confidence_count > 0:
            self._reasons.append(
                f"climb_blocked residual_loss={signals.residual_loss_w:.3f}"
            )
        st.up_confidence_count = 0
        st.up_target_mcs = -1
        return cur, tx, False

    st.up_confidence_count += 1
    if st.up_confidence_count < self.sel.upward_confidence_loops:
        return cur, tx, False
    if elapsed_since_mcs_ms < self.sel.hold_modes_down_ms:
        return cur, tx, False
    # fall through to the apply-change path
```

Semantics:

- On any tick with `residual_loss_w > 0` and an in-progress climb
  (`up_confidence_count > 0`), reset the counter to 0, clear the
  target, log `climb_blocked residual_loss=X.XXX` to the decision
  reasons.
- The counter must therefore see `upward_confidence_loops` (12)
  *consecutive* clean ticks before a climb can fire. At 10 Hz
  controller cadence, that's 1.2 s of zero-loss eligibility.
- The candidate-change reset (`up_confidence_count = 1`) is also
  loss-gated: if loss is happening at the moment a new candidate
  appears, the counter starts at 0 rather than 1.

### Deprecated config

`profile_selection.hold_fallback_mode_ms` becomes unused. To keep
backward compatibility with operator YAML files in the field:

- The dataclass field stays parseable. A `__post_init__` warning is
  logged if a non-default value is read (so the operator sees the
  knob is no-op now).
- `gs.yaml.sample` removes the key.
- `deploy/gs/gs.yaml` removes the key as part of this change.

### Composition with existing mechanisms

- **`loss_margin_weight` / `fec_margin_weight`**: still active. They
  widen SNR margin under stress, making `up_target_mcs` itself
  harder to nominate. The veto adds a hard block on top. Defense
  in depth.
- **Emergency channel (Channel B)**: completely unaffected. Loss
  rate ≥ `emergency_loss_rate` (0.10) still triggers immediate
  one-step demotion regardless of any of this.
- **`hold_modes_down_ms`**: unchanged. Still gates the climb at the
  tail end (after confidence reaches threshold).

## Tests

New unit tests in `tests/test_policy_leading.py`:

1. **`test_climb_blocked_by_residual_loss`** — at MCS=2 with SNR
   margin clearly above the climb threshold, build
   `up_confidence_count` for several clean ticks, inject one tick
   with `residual_loss_w = 0.05`. Assert counter resets to 0,
   `up_target_mcs = -1`, no MCS change emitted, decision reason
   includes `climb_blocked`.

2. **`test_climb_proceeds_after_clean_window`** — same setup, after
   the loss tick run `upward_confidence_loops` consecutive clean
   ticks. Assert climb emits exactly on the Nth clean tick.

3. **`test_climb_from_mcs_0_uses_confidence_loop`** — start at
   MCS=0, SNR margin met, no loss. Assert climb fires only after
   `upward_confidence_loops` clean ticks AND
   `hold_modes_down_ms` elapsed since the last MCS change (not
   the old 1 s `hold_fallback_mode_ms`). Validates the unification.

4. **`test_climb_from_mcs_0_blocked_by_loss`** — at MCS=0 with
   nonzero residual loss in current tick, confirm no climb
   regardless of SNR margin or elapsed time since last MCS change.

5. **`test_deprecated_hold_fallback_mode_ms_parses`** — load a
   config with `hold_fallback_mode_ms: 1000`; assert dataclass
   parses without error, the field is present, and a deprecation
   warning was logged.

Existing tests to update:

- **`test_hold_fallback_mode_ms_when_at_mcs_zero`**
  (`tests/test_policy_leading.py:359`) — this test explicitly
  validates the old 1-s `hold_fallback_mode_ms` semantics. Delete
  it; its replacement is the new `test_climb_from_mcs_0_*` cases
  above.
- The `_selector(...)` helper at `test_policy_leading.py:51`
  accepts `hold_fallback_mode_ms` as a parameter. Keep the
  parameter (for back-compat parsing of old `gs.yaml`) but stop
  consuming it in the selector itself.

## Migration

- **Operator GUI / yaml**: change is invisible. Old YAML still
  parses (with a deprecation warning); new YAML omits the key.
  No config flag-day.
- **Drone applier**: unaffected. This is a GS-only patch.
- **Wire format**: unaffected. No new fields, no version bump.
- **Replays**: a captured `flight-NNNN/gs.jsonl` may include
  `climb_blocked` reason strings after this lands. Tooling that
  parses reasons should treat unknown strings as opaque already.

## Observability

The new `climb_blocked residual_loss=X.XXX` reason string is
emitted on every veto where the counter was already accumulating.
Operators can grep `gs.jsonl` for `climb_blocked` to see how often
the gate fires per flight.

Optional follow-up (not in this change): include
`up_confidence_count` in the per-decision `signals_snapshot` for
deeper offline diagnosis. ~3 lines of code; defer until someone
needs it.

## Rollout

1. Land the policy.py change + tests + sample-config edit on
   `master`.
2. Run the deploy script against the live GS
   (`./deploy/gs/deploy-gs.sh`).
3. Re-fly. Compare flight-NNNN against flight-0003/0004:
   - MCS-up transitions per flight should drop substantially.
   - Non-emergency IDR count should drop in step with #1.
   - Time at MCS 3-5 may decrease (more conservative climbs).
4. If goodput at the new tune is acceptable, the patch is done.
   If goodput is unacceptably low, the remaining lever is
   tuning down `upward_confidence_loops` (12 → 8 perhaps), not
   weakening the veto itself.
