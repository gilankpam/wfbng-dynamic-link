# Bitrate-aware FEC: making encoder rate respond to n_escalation

## Problem

At MCS 0 (or any low PHY rate) near the edge of range, the link can enter a
death spiral:

1. Loss appears as SNR degrades.
2. `NEscalator` ramps `n_escalation` from 0 toward `max_n_escalation` (default
   6) over ~2 seconds, growing FEC redundancy from 1.4× to as much as 4×.
3. The encoder bitrate is **not** adjusted as `n` grows. With the bitrate
   sized for `wire = bitrate × 1.4`, the actual wire rate `bitrate × n/k`
   silently inflates past the effective PHY rate.
4. wfb_tx drops packets locally because the encoder feeds faster than the
   radio can carry. Latency grows in the buffer between encoder and wfb_tx.
5. The dropped packets manifest at the GS as more loss, which the
   `NEscalator` interprets as worse link conditions — escalating `n` further.
6. Video stalls, then mavlink starves (shared radio), pilot loses telemetry,
   crash.

The mechanism is purely on the GS side: `compute_bitrate_kbps` assumes a
fixed `k/n = 1/(1 + base_redundancy_ratio) = 0.714` regardless of what the
`NEscalator` is doing at the same tick. Worked example with the current
`deploy/gs/gs.yaml` at MCS 0 (`utilization_factor=0.6`,
`base_redundancy_ratio=0.4`, `max_n_escalation=6`, `mtu=1500`, `fps=60`):

| escalation | n  | wire (bitrate × n/3) | vs 5950 PHY |
|------------|----|----------------------|-------------|
| 0          | 5  | 4250                 | +1700 ✓     |
| 1          | 6  | 5100                 | +850  ✓     |
| 2          | 7  | 5950                 | 0 (edge)    |
| 3          | 8  | 6800                 | **−850 saturated** |
| 6          | 11 | 9350                 | −3400        |

(Encoder bitrate is held at 2550 kbps throughout; only the wire rate
inflates.)

## Solution

Anchor every per-tick computation on a single new quantity,
`wire_target_kbps = eff_phy_kbps × utilization_factor`, and derive encoder
bitrate from the *live* `(k, n)` rather than a fixed assumed ratio. As FEC
escalates, encoder bitrate shrinks so total wire stays at the target.

### Per-tick dataflow

```
1. wire_target_kbps = eff_phy_kbps(mcs, bw, mtu) × utilization_factor
2. k_candidate = packets_per_frame(wire_target_kbps, mtu, fps),
                 clamped to [k_min, k_max]
3. escalation  = NEscalator.update(residual_loss)
4. n_candidate = ceil(k × (1+base_ratio)) + escalation
5. n_candidate = clamp_n_for_bitrate_floor(
       n_candidate, k_candidate, wire_target_kbps, min_bitrate_kbps)
6. EmitGate decides whether (k_candidate, n_candidate) ride this tick;
   emitted (k, n) may equal the last emitted values if held by debounce
7. bitrate_kbps = wire_target_kbps × emitted_k / emitted_n,
                  clamped to [min_bitrate_kbps, max_bitrate_kbps]
8. Decision{ mcs, k=emitted_k, n=emitted_n, bitrate=bitrate_kbps, ... }
```

### Invariants enforced every tick

- **Wire safety**: `bitrate × n / k ≤ wire_target_kbps ≤ eff_phy × util`.
  The death-spiral mechanism cannot ignite.
- **Bitrate floor**: `bitrate ≥ min_bitrate_kbps`. Honored by construction
  via `clamp_n_for_bitrate_floor`.
- **(k, n)-bitrate coherence**: emitted bitrate is computed from the same
  (k, n) the EmitGate decided to emit. Never out of sync.

### What stays the same

- `NEscalator` loss/recovery logic — same 3-tick-up / 10-tick-recover
  hysteresis, same `max_n_escalation` cap.
- `EmitGate` behavior on (k, n) — always emits on MCS change; debounces
  solo (k, n) changes by 2 ticks.
- Wire format. The decision packet still carries a single
  `bitrate_kbps` field; only the value changes under loss.
- IDR-burst behavior on MCS change.
- Drone-side applier and backends — bitrate changes already drive the
  encoder backend via HTTP; no drone-side code touched.

### Behavior diff vs current code

| Scenario | Current | New |
|---|---|---|
| Clean link, any MCS | bitrate = 2550 kbps (MCS 0) | same (within rounding) |
| Light loss, escalation=1–2 | bitrate static, wire grows | bitrate steps down 5–15%, wire constant |
| Sustained loss, escalation=6 | bitrate static, wire 50%+ over PHY | bitrate at ~⅓ of baseline, wire at PHY |
| Loss past floor (extreme) | death spiral | floor honored, FEC growth capped via `clamp_n_for_bitrate_floor` |

The only "new" behavior under load is encoder retune cadence during a loss
event (~7 retunes during 1.8 s escalation, ~6 during 6 s recovery). The
retune count is bounded by `NEscalator`'s hysteresis — escalator can't bump
faster than every 3 ticks.

## Design choices considered and rejected

**Constant wire-rate target (option `a`).** Keep `compute_k` as
`packets_per_frame(bitrate, ...)` and iterate / two-pass to converge on a
self-consistent `(bitrate, k, n)`. Rejected for the closed-form approach
because: (1) it preserves the implicit feedback loop between bitrate and k
that makes the system harder to reason about, (2) it introduces a 1-tick lag
where k was sized for the previous bitrate, (3) the closed-form expression
`wire_target × k / n` is more honest about what FEC is actually doing.

**Threshold-based wire clamp (option `b`).** Add an assertion `bitrate × n/k
≤ eff_phy × safety` and back off only when violated. Rejected because: it
keeps the underlying inconsistency (assumed 1.4× vs actual n/k) and only
catches the worst case at the edge. Doesn't fix the small-k `ceil()`
over-allocation that exists even at escalation=0.

**Dedicated bitrate cooldown (option `γ` symmetric).** Add
`min_change_interval_ms_bitrate`. Rejected because it breaks the
wire-safety invariant during the cooldown window — exactly the mechanism we
are spending this design to fix. The asymmetric variant (down-only is
immediate, up is throttled) preserves the invariant but adds a config knob,
a branch, and a new oscillation mode for marginal retune savings. Kept as
an escape hatch if encoder retune cost ever turns out to be measurable.

**Hard clamp on bitrate at the floor, let n keep growing (option `i` for
Q4).** Rejected because at the floor, `bitrate = min_bitrate` while n
inflates → wire grows past target. Same death-spiral mechanism, just
triggered at the floor instead of mid-escalation.

## Code changes

### `gs/dynamic_link/bitrate.py`

Split the API into two pure functions:

```python
def compute_wire_target_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    mtu_bytes: int,
    utilization_factor: float,
) -> float

def compute_bitrate_kbps(
    wire_target_kbps: float,
    k: int,
    n: int,
    min_bitrate_kbps: int,
    max_bitrate_kbps: int,
) -> int
```

`BitrateConfig` field changes:

- **Remove**: `base_redundancy_ratio` — no longer in the bitrate formula.
- **Keep**: `utilization_factor`, `min_bitrate_kbps`, `max_bitrate_kbps`.

### `gs/dynamic_link/dynamic_fec.py`

`compute_k` argument rename — `bitrate_kbps` → `wire_target_kbps`. Math is
unchanged; the input is now the worst-case wire rate (full utilization)
rather than the live encoder rate. Result: `k` is a pure function of
`(MCS, mtu, fps, utilization_factor)` with no feedback loop.

`compute_n` unchanged.

New helper:

```python
def clamp_n_for_bitrate_floor(
    n_candidate: int,
    k: int,
    wire_target_kbps: float,
    min_bitrate_kbps: int,
) -> int:
    n_max_phy = int(wire_target_kbps * k / min_bitrate_kbps)
    return max(k, min(n_candidate, n_max_phy))
```

The `max(k, ...)` guards the pathological case where `min_bitrate_kbps >
wire_target_kbps` (link literally can't carry minimum video). At that
point we cap n at k (degenerate: no parity) rather than dipping below; the
wire-safety invariant slightly bends here, but only when link is failing
beyond what this layer can fix.

### `gs/dynamic_link/policy.py`

Rewrite the per-tick chunk around the new dataflow (existing lines
~814–855). Net diff: ~30 lines changed, no new state, no new debounce
primitives. The new bitrate computation moves AFTER the EmitGate so that
emitted `(k, n)` and emitted bitrate stay coherent.

### `gs/dynamic_link/service.py`

Drop the line parsing `policy.bitrate.base_redundancy_ratio`. Add a
one-time deprecation log line when the key is present:
`WARN: policy.bitrate.base_redundancy_ratio is deprecated and ignored;
fec.base_redundancy_ratio is now authoritative`. The warning comes out
in a follow-up PR once operator configs have been updated.

### Config samples

- `conf/gs.yaml.sample` — remove `policy.bitrate.base_redundancy_ratio` line
  and its "should match fec.base_redundancy_ratio" comment.
- `deploy/gs/gs.yaml` — same.

### `CLAUDE.md`

Update the "Dynamic FEC (P4b)" section. The current text inverts:

Replace:
> *"Bitrate uses a FIXED `k/n = 1/(1 + base_redundancy_ratio)`, not the
> live (k, n), so encoder allocation stays steady when n_escalation
> moves."*

With:
> *"Bitrate is derived from the live `(k, n)`: `bitrate = wire_target × k /
> n` where `wire_target = eff_phy × util`. As FEC escalates, encoder
> bitrate shrinks proportionally so total wire stays under PHY. The bitrate
> floor (`min_bitrate_kbps`) is enforced by `clamp_n_for_bitrate_floor`,
> which caps FEC growth before bitrate would drop below the floor."*

## Tests

### Updates to existing tests

- `tests/test_bitrate.py` — adapt to the split API; drop assertions tied to
  `base_redundancy_ratio`.
- `tests/test_dynamic_fec.py` — `compute_k` now takes `wire_target_kbps`.
  The formula is unchanged, so existing tests pass after a parameter
  rename, but the semantic interpretation of each case shifts: where a
  test previously read "at encoder bitrate 5000 kbps, k=...", it now
  reads "with wire-target 5000 kbps, k=...". Test case comments updated
  alongside the rename.
- `tests/test_policy_bitrate.py` — exercise the new dataflow; assert
  bitrate moves with escalation.
- `tests/test_policy_dynamic_fec_e2e.py` — add wire-invariant assertions to
  existing scenarios.

### New tests

1. **`compute_k` is independent of bitrate**: same `(MCS, mtu, fps,
   util)` always yields the same `k` regardless of escalation/loss. Catches
   regressions where someone reintroduces the bitrate→k coupling.

2. **`clamp_n_for_bitrate_floor` behavior**:
   - Headroom case: `n_candidate < n_max_phy` → returns `n_candidate`
     unchanged.
   - Cap case: `n_candidate > n_max_phy` → returns `n_max_phy`, ensures
     `bitrate(k, returned_n) ≥ min_bitrate_kbps`.
   - Degenerate case: `n_max_phy < k` (link failing) → returns `k`,
     documents the wire-safety bend.

3. **Wire-safety invariant (parametric)**: for every `(mcs ∈
   profile.mcs_min..mcs_max, escalation ∈ 0..max_n_escalation)`
   combination, assert `bitrate × n / k ≤ wire_target_kbps`. Lock-in for
   the bug we're fixing.

4. **Bitrate-floor invariant (parametric)**: same matrix, assert `bitrate
   ≥ min_bitrate_kbps`.

5. **Death-spiral scenario (regression test for the bug)**:
   - Stimulus: simulated trace at MCS 0, `residual_loss = 5%` sustained for
     30 ticks, then `0%` for 100 ticks.
   - Assert escalation ramps 0→6 over ~18 ticks, bitrate steps down
     monotonically with each escalation step, wire bitrate stays
     `≤ wire_target` across the whole trace.
   - Assert recovery: bitrate climbs back to baseline as escalation decays.

6. **EmitGate alignment**: when EmitGate holds (k, n) at the previous
   values, the emitted bitrate uses the previous (k, n), not the
   candidate. Locks in the (β) emission decision.

### E2E

`tests/test_drone_e2e.py` gets one new scenario function:
`test_loss_episode_does_not_oversubscribe_wire`. Drive the GS service with
a synthetic stats trace containing a loss burst; verify the bitrate
trajectory and that wfb_tx invocation arguments are well-formed
throughout.

## Migration

**Config breaking change**: `policy.bitrate.base_redundancy_ratio` is
obsolete. Operators with the key set get a one-time deprecation warning
at startup; the value is ignored.

**Wire format**: unchanged. Decision packet still carries a single
`bitrate_kbps` field.

**Runtime behavior diff**: clean conditions identical to today; under
sustained loss, encoder bitrate now shrinks (was constant). The drone-side
encoder backend already handles bitrate changes via the existing
MCS-change path. No drone-side code touched.

**Rollout risk**: bounded. The retune cadence (~13 events per loss
episode) is gated by `NEscalator` hysteresis and cannot exceed that
ceiling. If waybeam retune cost turns out to matter, escape hatch is the
asymmetric bitrate cooldown described in "Design choices considered and
rejected"; that's an additive change that doesn't require revisiting this
design.

## Validation plan

1. `python3 -m pytest --ignore=tests/test_mavlink_status.py` from repo root.
2. `make -C drone test`.
3. GS-only smoke against a recorded `capture.jsonl`; compare bitrate
   trajectory side-by-side with current-master baseline.
4. Bench flight with `enabled: true`. Walk drone to far corner, spin to
   engage encoder; verify video doesn't lag during the high-loss segment.

## References

- Issue surfaced during 2026-05-22 bench session — hand-injecting
  `bitrate=5000` at MCS 0 with `k=8,n=9` confirmed GS receive at 5.04 Mbps;
  GS autopilot's `compute_k` at the same bitrate would pick `k=6,n=9`
  (`n/k=1.5`), wire=7500 kbps > 5952 PHY → infeasible.
- Operator failure mode: drone at edge → MCS forced to 0 → latency
  spike → video lag → crash.
- Related: `docs/superpowers/specs/2026-05-11-drone-config-handshake-and-dynamic-fec-design.md`
  (introduced the current `NEscalator` and `compute_k`/`compute_n` model).
- Related: `docs/superpowers/specs/2026-05-20-mtu-aware-bitrate-design.md`
  (introduced the `compute_bitrate_kbps` model that this design replaces).
