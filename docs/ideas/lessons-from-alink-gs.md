# Lessons from `alink_gs` (gilankpam/adaptive-link)

Notes from reading the alternate ground-station controller at
[gilankpam/adaptive-link/ground-station/alink_gs](https://github.com/gilankpam/adaptive-link/blob/main/ground-station/alink_gs)
during the Phase 2 post-crash review (April 2026).

The crashes we saw against `dynamic-link` weren't a TX-power-overdrive
issue — manual `iw set txpower 3000` (30 dBm) was reproduced as fine,
so the chip isn't desensing. The actual fault was the controller
**climbing into MCS levels the real link couldn't sustain**, on a
schedule driven by the post-drop inhibit window. `alink_gs` solves
the same family of problems differently and there are 3-4 ideas
worth importing.

## Architectural shape

| Aspect | alink_gs | dynamic-link |
|---|---|---|
| Decision unit | One **profile** per tick (MCS+GI+FEC+bitrate+power+GOP coupled atomically) | Independent loops: leading (MCS), trailing (FEC ladder), predictor (latency budget) |
| TX power | **Open-loop function of MCS**: `max − (mcs/max_mcs)·(max−min)` — low MCS ⇒ high power, instant transition | Closed loop on RSSI vs `rssi_target_dBm`, ramps at `step_max_db` per cooldown |
| Climb gate | Stress-aware margin + slope prediction + N-tick confidence count | Fixed `snr_margin_db` + hold-time hysteresis |
| Loss handling | Channel-B emergency bypass when `loss_rate ≥ emergency` → forced step-down | `forced_mcs_drop` after 3-of-3 sustained loss + post-drop inhibit |
| Wire format | Text UDP `P:idx:gi:mcs:k:n:br:gop:power:bw:ts:rtt`, drone delta-dedupes | Binary + CRC32, drone applies via `tx_cmd.h` / `iw` / encoder HTTP |
| Latency model | None | `predict()` + `fit_or_degrade()` budget gate (50 ms) |
| Depth (interleaver) | Not exposed | First-class knob with bootstrap + step-down |
| Telemetry | JSONL with per-change outcome labels (good/marginal/bad over next 10 ticks) | JSONL of decisions only |
| RTT | Clock-offset-cancelling handshake (gs_recv − t1) − (t3 − t2) | None |

Both designs work; they're different points on the design surface.
`dynamic-link` is more layered and instrumented; `alink_gs` is more
atomic and reactive.

## Four ideas worth borrowing, prioritised

### A. Stress-aware margin widening (highest priority — tier 1)

Their `_margin()`:
```python
stress_margin = (self.snr_safety_margin
                 + self._current_loss_rate    * self.loss_margin_weight
                 + self._current_fec_pressure * self.fec_margin_weight)
margin = self.snr_ema - MCS_SNR_THRESHOLDS[mcs] - stress_margin
```

Effective SNR threshold rises with active loss / FEC pressure. With
`loss_rate=0.95`, `loss_margin_weight≈10` adds 9.5 dB to the
threshold — survivor-biased SNR=24 no longer clears MCS 4's
now-effectively-31-dB requirement. **This directly addresses the
climb-fail loop in our crash logs.**

In our codebase the change is ~10 LoC in `LeadingLoop._mcs_up_target`,
plus plumbing `signals.residual_loss_w` and `signals.fec_work` into
the leading loop (currently they're owned by the trailing loop).

### B. SNR slope prediction (tier 1)

```python
predicted = tgt_margin + self._snr_slope * self.snr_predict_horizon_ticks
if tgt_margin < self.hysteresis_up_db or predicted < 0:
    return False  # refuse upgrade
```

EMA the per-tick Δsnr, project forward `horizon` ticks; refuse the
climb if projected margin goes negative even if current snapshot
clears. Our `snr_up_hold_ms = 2000` does some of this passively (need
2 s of sustained good SNR to climb), but slope prediction catches
"currently good but trending down fast" earlier.

### C. Gate the controller on `receiving_video` (tier 1, trivial)

```python
if receiving_video:
    calculate_link()
```

`alink_gs` skips the controller tick entirely when no video frames
are arriving. We tick `Policy.tick()` on every RxEvent — even
zero-packet ones with stale RSSI/SNR. That's the
**post-crash stuck oscillation** we logged: signals freeze at
last-good values, leading loop sees "great SNR, climb!", starvation
drops, repeat forever. Gating the policy on `signals.packet_rate_w >
threshold` (or equivalently `not link_starved_w`) makes it stop.

Cosmetic for the post-crash case (drone is on the ground), but the
same pathology can happen mid-flight on a brief total-blackout. Worth
doing.

### D. Inverse MCS↔TX-power coupling (tier 3 — needs design pass)

Their TX power is a function of MCS, not a closed loop on RSSI:
- MCS 0 → max_power instantly
- MCS 5 → ~min_power
- Profile changes are atomic; power moves with MCS in the same tick.

This is the ONE that most directly explains *why* our crashes
unfold the way they do. Bench tuning drove our TX power to 5 dBm
at MCS 5. When the drone took off and forced-dropped to MCS 0, our
closed loop needed `(28−5)/3 × 1 s ≈ 8 s` to ramp power back up.
For 8 seconds the drone was TXing at 5 dBm outside the link envelope.

In `alink_gs`, an MCS-down event also drops the operating point of
TX power simultaneously — instant. No ramp delay.

**Tradeoff:**
- Our closed loop: precise (drives toward an absolute RSSI setpoint).
  Power floors are configurable. But slow recovery when MCS drops.
- Their open loop: less precise (always at MCS-coupled power), but no
  ramp-up delay.

For a safety-critical FPV link the open-loop coupling is probably the
right tradeoff. But it's a real architectural change — the
RSSI-target sub-loop disappears entirely — so it deserves a written
design pass before we commit.

## Things `dynamic-link` has that they don't

- **Latency-budget predictor** — they don't model interleaver latency
  at all. Our `predict()` + `fit_or_degrade()` is a real value-add.
- **Depth (interleaver) control with bootstrap + step-down** — they
  don't expose depth.
- **Trailing-loop FEC ladder with band/step semantics** — theirs is a
  single static formula `n = ceil(k / (1 − redundancy_ratio))` with
  one stress-tweak. Ours adapts to actual loss patterns.
- **Per-MCS preferred_k from a calibrated radio profile** — theirs
  uses canonical 802.11n thresholds.
- **Drone-side watchdog + safe-defaults push** on GS silence.
- **Binary wire + CRC32 + vendored tx_cmd.h contract test** — more
  robust on a noisy link than text UDP, and the contract anchor
  catches schema drift.

## Things they have that are independently worth a look

These don't address the crash but are nice-to-haves we should track:

- **RTT measurement.** Their handshake nets a clock-offset-cancelling
  RTT estimate `(gs_recv − t1) − (t3 − t2)`. We have no RTT signal at
  all today. Useful as an additional stress indicator (queue
  buildup) and for diagnostics.
- **Outcome-labeled telemetry.** They tag each profile change with a
  good/marginal/bad label based on `loss_rate` over the next ~10
  ticks. Useful both for offline tuning of the stress weights from (A)
  and for any future ML-driven controller.
- **Drone session detection via handshake `session_id`.** A clean
  alternative to the `wfb_tx -e $(od urandom)` trick we considered
  for per-flight log rotation in
  `docs/ideas/per-flight-jsonl-rotation.md` — and it doesn't need any
  drone-side launch-script edit if the drone exposes its own session
  identifier in the handshake reply.

## What I would NOT borrow

- **Coupling FEC k/n to MCS via a static formula.** Our trailing loop
  adapts FEC to actual loss patterns; theirs is a fixed
  `n = ceil(k / (1 − redundancy_ratio))`. We'd lose granularity.
- **Their text wire format.** Our binary + CRC + vendored schema is a
  system worth keeping; churning it for visual readability is a step
  back.
- **Their fixed depth.** We've gotten value from depth bootstrap +
  step-down; deleting it would be a regression.
- **Stress weights as the *only* loss handling.** We already have
  `forced_mcs_drop` + starvation; stress-margin should be **additive**
  to those, not a replacement. The two complement each other —
  stress-margin prevents bad climbs, forced-drop catches when we got
  it wrong anyway.

## Plan-of-record going forward

When picking this work back up:

1. **Tier 1 first** (A + B + C bundled): one focused change.
   - Plumb `residual_loss_w` and `fec_work` into the leading loop.
   - Replace the constant `snr_margin_db` use in
     `_mcs_down_target` / `_mcs_up_target` with `_stress_margin()`.
   - Add `_snr_slope` EMA to `SignalAggregator`; predictive gate in
     `_mcs_up_target`.
   - Gate `Policy.tick()` on `not link_starved_w` (or a stricter
     packet-rate check).
   - New gs.yaml knobs: `loss_margin_weight`, `fec_margin_weight`,
     `snr_predict_horizon_ticks`, `snr_slope_alpha`.
   - Tests + live antenna-cover repro of the climb-fail loop, verify
     it doesn't repeat.

2. **Tier 3 separately**: written design pass for inverse MCS↔TX-power
   coupling. Decide whether to keep the closed-loop alongside as a
   fallback or replace entirely.

3. **Independent**: RTT measurement (small), outcome-labeled telemetry
   (small), drone session_id detection (might supersede the
   per-flight-rotation `-e` trick).
