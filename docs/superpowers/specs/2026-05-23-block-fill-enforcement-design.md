# Block-fill latency enforcement: bring back `blocks_per_frame`

## Problem

The previous spec (`2026-05-22-bitrate-aware-fec-design.md`) made FEC
bitrate-aware: encoder bitrate now shrinks as `n_escalation` grows, so
total wire rate stays at `wire_target = eff_phy × utilization_factor`.
That closes the death-spiral mechanism. It also moved `compute_k`'s input
from the encoder bitrate to `wire_target_kbps`, so `k` is now sized off
the *wire* rate rather than the *encoder* rate.

The unintended consequence: **block_fill latency now exceeds one video
frame period**. The block_fill term in the latency predictor is

```
block_fill_ms = k × inter_packet_interval = k × mtu_bits / encoder_kbps
```

On the current branch, with `encoder_kbps = wire_target × k / n`, this
simplifies to

```
block_fill_ms = n × mtu_bits / wire_target_kbps
```

`k` cancels: block_fill scales linearly with `n`. With the branch sizing
`k ≈ wire_pps_per_frame`, the redundancy ratio leaks straight into
latency. Measured worst case at the deploy config (mtu=1500, fps=60,
util=0.7):

| MCS | block_fill @ n_base | block_fill @ n_max | × frame_period |
|---:|---:|---:|---:|
| 0 | 20.2 ms | 28.8 ms | 1.21× → 1.73× |
| 1 | 21.9 ms | 31.2 ms | 1.31× → 1.87× |
| 5 | 23.5 ms | 26.9 ms | 1.41× → 1.61× |

Frame period at 60 fps is 16.67 ms; block_fill should be `≤ frame_period`
so FEC blocks line up with video frames. The branch silently broke that.

The constraint was originally encoded in `fec.blocks_per_frame: 1` in
`gs.yaml`, but `grep` shows nothing reads that key. It is documentation-
only. The historical knob `fec_block_fill_ms_target` was removed in the
static-table era with the rationale *"block-fill is now bounded by
k_bounds.max"* (`service.py:210`) — but `k_max=50` is far above the
frame-period limit, so the ceiling is decorative.

A previous WIP attempt at this fix exists as `stash@{0}` on
`feat/fec-blocks-per-frame`. It introduces `blocks_per_frame: float` as
a divisor inside `compute_k` against an encoder-anchored input. That
stash predates bitrate-aware-FEC; the divisor mechanism still applies
but the input quantity needs to be re-derived because `compute_k` on
this branch is fed `wire_target_kbps`, not encoder bitrate.

## Goal

Restore the invariant `block_fill_ms ≤ frame_period_ms` at all loss
levels, including maximum `n_escalation`, while preserving the
bitrate-aware-FEC wire-safety invariant.

## Approach

Resize `k` so that at the *maximum* `n` the controller will ever emit
(`n_max = ceil(k × (1 + max_redundancy_ratio))`), the block_fill bound
still holds.

Anchor `compute_k` on the **encoder bitrate at `n_base`**
(`anchor_encoder_kbps = wire_target / (1 + base_redundancy_ratio)`) and
divide by a new field `blocks_per_frame`:

```
anchor_encoder_kbps = wire_target_kbps / (1 + base_redundancy_ratio)
packets_per_frame   = anchor_encoder_kbps × 1000 / (fps × mtu × 8)
k                   = floor(packets_per_frame / blocks_per_frame),
                      clamped to [k_min, k_max]
```

Choose `blocks_per_frame ≥ 1 + max_redundancy_ratio` and the bound holds
by construction:

```
block_fill_at_n_max
  = n_max × mtu_bits / wire_target
  = ceil(k × (1 + max_red)) × mtu_bits / wire_target
  ≤ (1 + max_red) / ((1 + base_red) × bpf) × frame_period
  ≤ frame_period         (since bpf ≥ 1 + max_red ≥ 1 + base_red)
```

Default `blocks_per_frame = 1 + max_redundancy_ratio` at config-load
time. With the deploy config (`max_red=1.0`), the resolved default is
`2.0`, giving hard-bound block_fill at all loss levels. Operators may
override:

- `bpf > 1 + max_red` → tighter (faster recovery, smaller blocks, less
  burst-loss spreading)
- `bpf < 1 + max_red` → looser, block_fill > frame_period possible under
  sustained loss; service logs a warning at startup

### Per-tick dataflow

Only step 3's internals change. The pipeline shape is identical to
`2026-05-22-bitrate-aware-fec-design.md`.

```
1. (mcs, tx_power, mcs_changed) = leading.select(...)
2. wire_target_kbps = compute_wire_target_kbps(mcs, bw, mtu, util)
3. k = compute_k(wire_target_kbps, mtu, fps, cfg)
        # CHANGED: derives anchor_encoder = wire/(1+base_red) internally,
        # then divides packets_per_frame by cfg.blocks_per_frame
4. escalation = NEscalator.update(residual_loss_w)
5. n_unclamped = compute_n(k, escalation, cfg)
6. n           = clamp_n_for_bitrate_floor(n_unclamped, k,
                                           wire_target_kbps,
                                           min_bitrate_kbps)
7. (new_k, new_n) = EmitGate.decide(k, n, mcs_changed)
8. bitrate     = compute_bitrate_kbps(wire_target_kbps,
                                      new_k, new_n, min, max)
9. depth       = trailing.tick(...)
10. (new_k, new_n, depth) = fit_or_degrade(Proposal(...),
                                           max_latency_ms, predictor_cfg)
11. bitrate    = compute_bitrate_kbps(...)   # recompute after budget gate
12. Commit state, emit Decision
```

### Worked example — MCS 5, deploy config

Constants: `wire_target = 20 960 kbps`, `mtu = 1500`, `fps = 60`,
`base_red = 0.4`, `max_red = 1.0`, `bpf = 2.0`, `min_br = 1000`.

No-loss tick:

| step | value |
|---|---|
| `anchor_encoder` = 20 960 / 1.4 | 14 971 kbps |
| `packets_per_frame` = 14 971 × 1000 / (60 × 1500 × 8) | 20.79 |
| `k = int(20.79 / 2.0)` | **10** |
| `n_base = ceil(10 × 1.4) + 0` | 14 |
| `bitrate = 20 960 × 10 / 14` | **14 971 kbps** |
| `block_fill = 14 × 12 000 / 20 960` | 8.0 ms (0.48× frame) ✓ |

Max-escalation tick (escalation=6):

| step | value |
|---|---|
| `k` | 10 (per-MCS, unchanged) |
| `n_unclamped = ceil(10 × 1.4) + 6` | 20, capped by `n_max = ceil(10 × 2.0) = 20` |
| `bitrate = 20 960 × 10 / 20` | **10 479 kbps** |
| `block_fill = 20 × 12 000 / 20 960` | 11.5 ms (0.69× frame) ✓ |

### Invariants enforced every tick

- **Wire safety**: `bitrate × n / k ≤ wire_target_kbps ≤ eff_phy × util`.
  By construction (`compute_bitrate_kbps`). Unchanged from previous spec.
- **Bitrate floor**: `bitrate ≥ min_bitrate_kbps`. By
  `clamp_n_for_bitrate_floor`. Unchanged.
- **(k, n)-bitrate coherence**: bitrate computed from the emitted
  `(k, n)`. Unchanged.
- **NEW — block-fill bound**: `block_fill_ms ≤ frame_period_ms / bpf` for
  any emitted `n ≤ n_max` when `bpf ≥ 1 + max_redundancy_ratio`. By
  construction; default `bpf` guarantees the hard form
  `block_fill_ms ≤ frame_period_ms`.

### What stays the same

- `wire_target_kbps` derivation, `compute_bitrate_kbps`, and
  `clamp_n_for_bitrate_floor`.
- `NEscalator` (3-tick-up / 10-tick-recover hysteresis).
- `EmitGate` (debounce 2 ticks for solo (k, n) changes; emit on MCS
  change).
- `compute_n` signature and formula.
- `predictor.predict` and `fit_or_degrade` (depth-budget gate, still
  only adjusts depth).
- Wire format (`dl_wire.h` / `wire.py`).
- Drone applier and all backends.
- IDR-burst behavior on MCS change.

### Behavior diff vs current branch

At the deploy config (mtu=1500, fps=60, util=0.7, base_red=0.4,
max_red=1.0):

| MCS | k (now → new) | n_base (now → new) | bitrate@n_base (now → new) | block_fill@n_max (now → new) |
|---:|---:|---:|---:|---:|
| 0 | 5 → 2 | 7 → 3 | 2 975 → 2 777 | 28.8 → 11.5 ms |
| 1 | 10 → 3 | 14 → 5 | 5 489 → 4 610 | 31.2 → 9.4 ms |
| 2 | 14 → 5 | 20 → 7 | 7 486 → 7 639 | 29.2 → 11.2 ms |
| 3 | 18 → 6 | 26 → 9 | 9 208 → 8 867 | 28.9 → 10.8 ms |
| 4 | 24 → 8 | 34 → 12 | 12 412 → 11 723 | 27.3 → 10.9 ms |
| 5 | 29 → 10 | 41 → 14 | 14 825 → 14 971 | 26.9 → 11.5 ms |

Bitrate at no-loss is within rounding of the current branch. Smaller `k`
trades FEC granularity (fewer source packets per block → less burst
spreading) for a hard latency bound.

## Design choices considered and rejected

**Option A — hard-tie to `1/fps` with no config knob.** Compute the
ceiling from `fps` alone, no operator surface. Rejected because the
existing yaml already references `blocks_per_frame` (declarative-only at
the moment), and reviving it as a real knob lets operators tighten for
high-fps screens or loosen for FEC granularity at low MCS.

**Option B — cap `n` directly by the block_fill constraint, leave
`compute_k` wire-anchored.** Simulated: at `bpf=1.0`, the cap forces
`n ≤ k` everywhere (no FEC parity) because `k = wire_pps_per_frame`
already saturates the bound. At `bpf<1.0`, partial FEC with block_fill >
one frame. Structurally cannot have all three of (wire-anchored `k`,
non-zero parity, block_fill ≤ frame_period) at full wire utilization.
Rejected.

**Option D — derive `blocks_per_frame` from `max_redundancy_ratio`
internally, no yaml key.** Simplest. Rejected because the user wants the
knob exposed for screen-specific tuning, and the previous WIP stash
landed on a configurable form.

## Implementation outline

### `gs/dynamic_link/dynamic_fec.py`

Add `blocks_per_frame: float` to `DynamicFecConfig` with a sensible
default (e.g., `2.0`, matching the resolved deploy default). Add
non-positive validation in `__post_init__`. Change `compute_k`:

```python
def compute_k(*, wire_target_kbps, mtu_bytes, fps, cfg):
    if wire_target_kbps <= 0 or mtu_bytes <= 0 or fps <= 0:
        return cfg.k_min
    anchor_encoder_kbps = wire_target_kbps / (1.0 + cfg.base_redundancy_ratio)
    packets_per_frame = (anchor_encoder_kbps * 1000.0) / (fps * mtu_bytes * 8.0)
    k = packets_per_frame / cfg.blocks_per_frame
    return max(cfg.k_min, min(cfg.k_max, int(k)))
```

Update the module docstring to reference the new bound.

### `gs/dynamic_link/service.py`

In `_build_policy_config`, resolve the default at load time and warn on
under-spec:

```python
max_red = float(fec_raw.get("max_redundancy_ratio", 1.0))
hard_bpf = 1.0 + max_red
bpf = float(fec_raw.get("blocks_per_frame", hard_bpf))
if bpf < hard_bpf:
    log.warning(
        "config: fec.blocks_per_frame=%.2f is below "
        "1 + max_redundancy_ratio (%.2f) — block_fill will exceed one "
        "frame period under sustained loss. Set blocks_per_frame >= "
        "%.2f for the hard latency bound.",
        bpf, hard_bpf, hard_bpf,
    )

dynamic_fec = DynamicFecConfig(
    ...,
    blocks_per_frame=bpf,
    ...,
)
```

No change to the legacy-keys list; `blocks_per_frame` is the new
canonical key, not legacy.

### `conf/gs.yaml.sample` and `deploy/gs/gs.yaml`

Add a commented-out entry with the comment explaining the bound:

```yaml
fec:
  base_redundancy_ratio: 0.4
  max_redundancy_ratio: 1.0
  # blocks_per_frame: number of FEC blocks per video frame at n_base.
  # Sets block_fill latency budget: each block fills in
  # frame_period / blocks_per_frame ms. Default = 1 + max_redundancy_ratio
  # (2.0 with max_red=1.0), which guarantees block_fill ≤ one frame even
  # at maximum n escalation. Setting lower trades latency for FEC
  # granularity; service warns if below 1 + max_redundancy_ratio.
  # blocks_per_frame: 2.0
```

### `gs/dynamic_link/predictor.py`

Side fix in the same PR: the docstring at lines 106-108 still claims the
trailing controller calls `max_n_for_latency` and chooses `(k, n)` under
the latency cap. That has not been true since dynamic FEC landed.
Replace with a one-line note that the function is reference code, not
wired into the live pipeline.

### What's explicitly NOT touched

- `policy.py` (call site unchanged — `compute_k` still takes
  `wire_target_kbps`).
- `bitrate.py` (`wire_target` and `compute_bitrate_kbps` unchanged).
- `wire.py`, `dl_wire.h`, `dl_applier.c`, `tx_cmd.h` (no protocol
  change).
- `NEscalator`, `EmitGate`, `fit_or_degrade` (unchanged).
- MAVLink status path (unchanged).

## Testing

### New unit tests (`tests/test_dynamic_fec.py`)

- `test_compute_k_uses_anchor_encoder` — at fixed `(wire, mtu, fps,
  base_red, bpf)`, returned `k` matches the closed-form expectation.
- `test_compute_k_falls_back_to_kmin_on_degenerate_inputs` — `wire ≤ 0`,
  `mtu ≤ 0`, `fps ≤ 0` each return `k_min`. Preserves existing contract.
- `test_dynamic_fec_config_rejects_nonpositive_bpf` — `bpf=0` and
  `bpf=-1` raise `ValueError`.
- `test_block_fill_bound_holds_with_default_bpf` — parameterized over
  `(MCS 0..7) × (n_escalation 0..max_n_escalation)` for
  `(mtu=1500, fps=60)` and `(mtu=1400, fps=60)` against the
  `m8812eu2` radio profile; asserts
  `n × mtu_bits / wire_target ≤ 1000/fps` for every cell after
  `clamp_n_for_bitrate_floor`. Load-bearing invariant test.
- `test_block_fill_bound_holds_when_bpf_exceeds_floor` — same
  parameterization at `bpf=3.0`; asserts bound holds, proving the
  formula isn't only correct at the exact floor.

### New service-loader tests (`tests/test_service_config.py` or
adjacent, following the existing config-loading test pattern)

- `test_blocks_per_frame_default_is_one_plus_max_red` — yaml without the
  key → resolved value equals `1 + max_red`.
- `test_blocks_per_frame_explicit_override` — yaml with explicit value
  overrides regardless of `max_red`.
- `test_blocks_per_frame_below_hard_bound_warns` — `bpf=1.0`,
  `max_red=1.0` triggers a `WARNING` log via `caplog` containing the
  threshold and the operator's value.
- `test_blocks_per_frame_at_or_above_hard_bound_silent` — no warning at
  `bpf=2.0` or `bpf=3.0`.

### Existing tests to update

- `tests/test_dynamic_fec.py` — `compute_k` tests that hardcoded
  expected `k` from the current (wire-anchored, no-bpf) formula. Keep
  test structure; recompute expected values from
  `wire / (1 + base_red) / bpf` math.
- `tests/test_policy.py` — tests asserting emitted `(k, n)` per MCS or
  per scenario. Recompute expected values. The death-spiral regression
  test (`test_death_spiral_*`) must still pass — wire safety is
  preserved by the same `clamp_n_for_bitrate_floor` mechanism.
- `tests/test_drone_e2e.py` — scenarios asserting decision-packet bytes,
  if any pin specific `(k, n)`. Update expected bytes; wire format
  unchanged.
- `tests/test_wire_contract.py` — verify still green; no change
  expected.

### Integration verification (manual, before commit)

1. `python3 -m pytest --ignore=tests/test_mavlink_status.py` — full
   suite green.
2. `make -C drone test` — C unit tests green (no C change).
3. Smoke replay on a captured stats feed if available; confirm:
   - `(k, n)` per MCS matches the simulation table above.
   - bitrate matches across escalation.
   - latency log shows `block_fill_ms ≤ 16.67` at deploy config.

## Operational consequences

- **Smaller `k` everywhere.** At MCS 5: `k = 10` (was `29`). Fewer
  source packets per block. A single dropped block now loses 10
  packets' worth of video instead of 29; finer-grained but the recovery
  hit is bigger per block. The `n_max=20` parity ceiling still covers
  up to 50% loss within a block.
- **MCS 0 lands at `k=2`** under the default `bpf=2.0` — the natural
  math, not the `k_min` floor (anchor_encoder ≈ 2 976 kbps,
  packets/frame ≈ 4.13, ÷ 2.0 → 2). Row is `(k=2, n_base=3, n_max=4)`:
  33% parity at base, 100% at max. Block_fill at n_max = 11.5 ms,
  inside the one-frame bound. If an operator raises `k_min` or `bpf`
  high enough that `k_min` becomes the binding constraint at low MCS,
  the bound can be violated there; field-validate.
- **Bitrate baseline unchanged.** No-loss bitrate per MCS within
  rounding of the current branch; same encoder behaviour for nominal
  links.
- **Bitrate under sustained loss slightly lower than current branch**
  (e.g., MCS 5 max: 10 479 vs 12 932 kbps). Cost of keeping block_fill
  within one frame.

## Out of scope

- Revisiting the `bitrate-aware-fec` spec rationale for closed-form vs
  iterative `k`. The anchor-encoder approach here remains closed-form
  (no per-tick fixed-point solve) — just anchored on a different
  starting quantity.
- Removing `predictor.max_n_for_latency`. It's orphan reference code
  but the tests still exercise it; keep it with a docstring note.
- Operator-facing telemetry for block_fill (it's already in the
  predictor's latency log line).
- Drone-side changes — none required.
