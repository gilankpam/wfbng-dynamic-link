# Adopt alink dynamic bitrate + IDR burst path

Status: design — pending implementation
Date: 2026-05-10

## Summary

Bring two specific behaviors from `alink_gs` into the dynamic-link
GS controller, while keeping our deterministic per-MCS FEC table
(the deliberate departure from alink documented in
`docs/knob-cadence-bench.md`):

1. **Dynamic encoder bitrate** computed each tick from the PHY
   data rate × utilization factor × actual coding rate `(k/n)`,
   replacing the fixed `bitrate_Mbps` column in the radio profile's
   `fec_table`.
2. **Burst IDR request path** decoupled from the 10 Hz stats
   cadence: when loss is detected, send N copies of the
   IDR-flagged decision packet at fast spacing (default 4 × 20 ms)
   to survive lossy UDP on the return link.

The existing dual-channel gate (`LeadingSelector` in `policy.py`)
is already an explicit port of `alink_gs.ProfileSelector.select()`
and needs no changes here. Defaults for the new bitrate knobs are
taken verbatim from `alink_gs.conf`.

## Motivation

- The fixed per-row `bitrate_Mbps` requires the operator to tune
  bitrate at every MCS row in the profile YAML. A single
  `utilization_factor` is easier to reason about and matches how
  alink operators tune the field. The PHY data rate per `(bw, mcs)`
  is already in the profile (`data_rate_Mbps_LGI`), so no new
  airframe data is needed.
- The current IDR signaling rides on the regular per-tick
  decision packet (one packet per 100 ms tick). On a 25% return-
  link loss rate, ~25% of IDR requests never arrive. A 4-packet
  burst at 20 ms spacing reduces that to ~0.4% (per-packet losses
  treated as independent — see simulation below). The drone-side
  HTTP throttle in `dl_backend_enc_request_idr`
  (`drone/src/dl_backend_enc.c:233`) already collapses duplicate
  arrivals to a single camera API call, so bursting is safe.

## Non-goals

Not adopted from alink, even though they exist in `alink_gs.conf`:

- Reactive `(k, n)` rewrites or `n+=1` under loss. FEC `(k, n)`
  stays deterministic per-MCS as today
  (`docs/knob-cadence-bench.md`).
- Latency-targeted FEC `K` sizing
  (`alink: block_latency_budget_ms`). We don't size FEC; the
  operator does, in the profile.
- Short-GI selection. All current profiles are LGI; SGI selection
  is not on the table.
- Adaptive online learning of `snr_safety_margin`,
  `fec_redundancy_ratio`, `utilization_factor`, `hysteresis_up_db`
  (`alink: [ml]` section). Out of scope.
- Outcome-labeled telemetry JSONL (`alink: [telemetry]`). We have
  `flight_log.py` for this concern.
- H:/I: handshake + RTT (`alink: [handshake]`). We use MAVLink
  STATUSTEXT downlink + Phase-3 ping/pong for timesync.

The existing `gate:` and `profile_selection:` defaults in
`conf/gs.yaml.sample` already match `alink_gs.conf` exactly
(snr_ema_alpha=0.3, snr_safety_margin=3, hysteresis_up_db=2.5,
max_mcs=5, hold_modes_down_ms=2000, etc.). No changes needed there.

## Part 1 — Dynamic bitrate

### New module

`gs/dynamic_link/bitrate.py` (~25 lines):

```python
from dataclasses import dataclass
from .profile import RadioProfile

@dataclass(frozen=True)
class BitrateConfig:
    utilization_factor: float   # 0 < u <= 1
    min_bitrate_kbps: int
    max_bitrate_kbps: int

def compute_bitrate_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    k: int,
    n: int,
    cfg: BitrateConfig,
) -> int:
    """Encoder bitrate (kbps) = PHY data rate * utilization * (k/n),
    clamped to [min, max]. The (k/n) factor accounts for FEC overhead
    so the encoder produces what the link can actually carry after
    parity. PHY rate comes from data_rate_Mbps_LGI[bw][mcs] (LGI
    only — we don't do SGI selection)."""
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    raw_kbps = phy_Mbps * 1000.0 * cfg.utilization_factor * (k / n)
    return int(max(cfg.min_bitrate_kbps,
                   min(cfg.max_bitrate_kbps, raw_kbps)))
```

### Profile schema change

`gs/dynamic_link/profile.py`:

- `FECEntry` becomes `{k: int, n: int}` — drop `bitrate_Mbps`.
- `MCSRow` drops `bitrate_Mbps`.
- `_validate()` rejects any `fec_table[bw][mcs]` containing a
  `bitrate_Mbps` key with a clear error:
  `ProfileError(f"{source}: fec_table[{bw}][{mcs}] contains "
                 f"bitrate_Mbps; bitrate is computed from "
                 f"policy.bitrate — drop this field")`.
  Loud failure beats silent ignore for operators with stale
  profiles.

### Policy integration

`gs/dynamic_link/policy.py`:

- `PolicyConfig` gains `bitrate: BitrateConfig`.
- `Policy.__init__` computes the cold-boot `bitrate_kbps` from the
  boot row's `(k, n)` via `compute_bitrate_kbps`. The
  `SafeDefaults.bitrate_kbps` field is removed — cold-boot value
  is now derived, not configured.
- In `Policy.tick()`, the line currently reading
  `encoder_kbps = row.bitrate_Mbps * 1000.0` becomes:

  ```python
  bitrate_kbps = compute_bitrate_kbps(
      self.profile, self.state.bandwidth, row.mcs, row.k, row.n,
      self.cfg.bitrate,
  )
  encoder_kbps = float(bitrate_kbps)
  ```

  Same value type, same downstream consumers
  (`_ipi_ms_for_encoder`, the latency-budget predictor) — they
  see no change.
- `state.bitrate_kbps` is written from `bitrate_kbps`.
  `knobs_changed.append("bitrate")` continues to work as before
  (compares to `prev.bitrate_kbps`).

### gs.yaml

New block, defaults matching `alink_gs.conf` (`[dynamic]
utilization_factor`, `[hardware] min_bitrate`, `[hardware]
max_bitrate`):

```yaml
policy:
  bitrate:
    utilization_factor: 0.8     # alink [dynamic] utilization_factor
    min_bitrate_kbps: 1000      # alink [hardware] min_bitrate
    max_bitrate_kbps: 24000     # alink [hardware] max_bitrate
```

`service.py` parses these into a `BitrateConfig` and passes it on
the existing `PolicyConfig` it constructs.

### Validation

At gs.yaml load time:

- `0 < utilization_factor <= 1` — reject otherwise.
- `min_bitrate_kbps > 0`.
- `max_bitrate_kbps >= min_bitrate_kbps`.

Boot-time WARN (not fatal) if `min_bitrate_kbps` exceeds the
computed bitrate for the boot row — the clamp will floor every
tick and the operator probably misconfigured one of them.

### Migration

- `conf/radios/m8812eu2.yaml`: strip the 16 `bitrate_Mbps` lines
  from `fec_table` (8 MCS rows × 2 bandwidths). One commit.
- `conf/gs.yaml.sample`: add the `policy.bitrate` block above.
- `docs/dynamic-link-design.md` (§4 / §6) and
  `docs/knob-cadence-bench.md`: replace the "FEC table is operator-
  validated `(k, n, bitrate)` triple" wording with "`(k, n)` pair;
  bitrate computed from PHY × utilization × (k/n)". One paragraph
  each.
- No drone-side change. The wire packet still carries
  `bitrate_kbps` as a uint16; the drone applier doesn't know or
  care how the GS computed it.

## Part 2 — IDR burst path

### Goal

When `Decision.idr_request == True`, send N copies of an IDR-
flagged decision packet at fast cadence on the wire, decoupled
from the 100 ms stats tick. Lossy UDP gets multiple shots; drone-
side throttle (`dl_backend_enc_request_idr`,
`drone/src/dl_backend_enc.c:233`) collapses duplicate arrivals to
one HTTP IDR call to the camera.

### Wire format

**No change.** Each burst packet is a normal 32-byte `DLK1`
decision frame (`drone/src/dl_wire.h`):

| Off | Size | Field          | Burst-packet value |
|----:|-----:|----------------|--------------------|
|   0 |    4 | magic          | `0x444C4B31` ("DLK1") |
|   4 |    1 | version        | `1` |
|   5 |    1 | flags          | `DL_FLAG_IDR_REQUEST` (`0x01`) — set on every burst packet |
|   6 |    2 | _pad           | 0 |
|   8 |    4 | sequence       | fresh per repeat (post-incremented by `Encoder.encode()`); avoids drone dedup |
|  12 |    4 | timestamp_ms   | as today |
|  16 |    1 | mcs            | snapshot of the triggering Decision — unchanged across the burst |
|  17 |    1 | bandwidth      | ditto |
|  18 |    1 | tx_power_dBm   | ditto |
|  19 |    1 | k              | ditto |
|  20 |    1 | n              | ditto |
|  21 |    1 | depth          | ditto |
|  22 |    2 | bitrate_kbps   | ditto |
|  24 |    1 | roi_qp         | 0 |
|  25 |    1 | fps            | 0 |
|  26 |    2 | _pad2          | 0 |
|  28 |    4 | crc32          | recomputed (since `sequence` changed) |

Why no new packet type or schema bit:

- Drone dedup keys on `sequence`. Fresh sequences = each burst
  packet enters the apply path.
- Per-backend diff apply
  (`last_tx`/`last_radio`/`last_enc` in `dl_applier.c`) makes the
  radio/encoder reapplications no-ops because all values match.
  Only the IDR call fires.
- `dl_backend_enc_request_idr` throttle collapses the N attempted
  HTTP calls to 1.
- `tests/test_wire_contract.py` (`dl-inject --dry-run` ↔ Python
  hex diff) is unaffected — same encoder, same bytes for given
  inputs.

### New module

`gs/dynamic_link/idr_burst.py` (~40 lines):

```python
import asyncio
from dataclasses import dataclass
from .decision import Decision
from .return_link import ReturnLink
from .wire import Encoder

@dataclass(frozen=True)
class IdrBurstConfig:
    enabled: bool = True
    count: int = 4              # alink: idr_max_messages
    interval_ms: int = 20       # alink: idr_send_interval_ms

class IdrBurster:
    """Sends N IDR-flagged decision packets at `interval_ms` spacing
    on asyncio. Re-triggering during an in-flight burst overwrites
    the counter and the snapshot decision (alink semantics — see
    keyframe_request_remaining = idr_max_messages). Each repeat
    re-encodes against a fresh sequence so the drone applier
    doesn't dedup the IDR away."""

    def __init__(self, cfg: IdrBurstConfig,
                 return_link: ReturnLink, encoder: Encoder):
        self._cfg = cfg
        self._return_link = return_link
        self._encoder = encoder
        self._latest_decision: Decision | None = None
        self._remaining = 0
        self._task: asyncio.Task | None = None

    def trigger(self, decision: Decision) -> None:
        if not self._cfg.enabled:
            return
        # count - 1: the regular per-tick path already sent the
        # first IDR-flagged packet on this tick.
        self._latest_decision = decision
        self._remaining = max(0, self._cfg.count - 1)
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._pump())

    async def _pump(self) -> None:
        while self._remaining > 0:
            await asyncio.sleep(self._cfg.interval_ms / 1000.0)
            if self._latest_decision is None:
                break
            packet = self._encoder.encode(
                self._latest_decision, idr_request=True,
            )
            self._return_link.send(packet)
            self._remaining -= 1
```

### service.py integration

`gs/dynamic_link/service.py`, near the existing per-tick send
path (currently `service.py:471-473`):

```python
if enabled and return_link is not None and wire_encoder is not None:
    packet = wire_encoder.encode(decision)
    return_link.send(packet)
    if decision.idr_request and idr_burster is not None:
        idr_burster.trigger(decision)
```

Construction next to the existing `wire_encoder` setup
(`service.py:373-374`):

```python
idr_burster = IdrBurster(cfg.idr_burst, return_link, wire_encoder)
```

### gs.yaml

```yaml
policy:
  idr_burst:
    enabled: true       # alink [keyframe] allow_idr
    count: 4            # alink [keyframe] idr_max_messages
    interval_ms: 20     # alink [keyframe] idr_send_interval_ms
```

### Re-trigger semantics

If `trigger()` is called while a pump task is already running
(e.g. loss persists across two consecutive 100 ms ticks):

- `_remaining` resets to `count - 1`
- `_latest_decision` updates to the newest one
- The pump task keeps running rather than being cancelled and
  restarted (simpler; no race window)

Matches alink: `keyframe_request_remaining = idr_max_messages`
overwrites unconditionally.

### Burst timeline (defaults: count=4, interval_ms=20)

```
  t=+ 0.00 ms  pkt #1/4  IDR_FLAG=1  (regular tick — loss detected)
  t=+20.13 ms  pkt #2/4  IDR_FLAG=1  (burst)
  t=+40.25 ms  pkt #3/4  IDR_FLAG=1  (burst)
  t=+60.36 ms  pkt #4/4  IDR_FLAG=1  (burst)

  next regular tick at t=+100 ms (10 Hz stats cadence)
```

Burst completes ~40 ms before the next stats tick — no overlap
with normal traffic. asyncio scheduling overhead measured at ~0.1
ms per slot.

### Loss-survival math

Per-packet losses treated as independent:

| per-pkt loss | single-pkt miss | burst-of-4 miss |
|-------------:|----------------:|----------------:|
| 10%          | 10.0%           | 0.01%           |
| 25%          | 25.0%           | 0.39%           |
| 50%          | 50.0%           | 6.25%           |
| 75%          | 75.0%           | 31.64%          |

Independence is conservative — short-burst losses are correlated
in practice, so 20 ms spacing helps if the burst is shorter than
60 ms.

### Drone-side requirements

None new. Already in place:

- `dl_backend_enc_request_idr` throttles at `min_idr_interval_ms`
  (`drone/src/dl_backend_enc.h:27`,
  `drone/src/dl_backend_enc.c:233`). For a single burst to
  collapse to one camera HTTP call, the throttle must be **longer
  than the burst window** = `interval_ms × (count - 1)` (default
  60 ms). If it isn't, multiple HTTP calls fire from one burst —
  not broken, just wasteful. Sustained loss across many ticks
  should still produce one IDR call per `min_idr_interval_ms`
  window regardless. Confirm the deployed drone configuration at
  rollout.
- Per-backend diff apply already no-ops radio/encoder writes when
  values are unchanged (`drone/src/dl_applier.c:380-413`).

### Bandwidth cost

4 × 32 bytes = 128 bytes per loss event over the wfb-ng tunnel.
At 10 Hz worst case (loss every tick), that's 1.28 KB/s — round-
ing error against the video stream.

## Testing

### New tests

- `tests/test_bitrate.py`: table-driven cases for
  `compute_bitrate_kbps` (typical, clamp-to-min, clamp-to-max,
  k/n at extremes, MCS at extremes) using a fixture profile.
- `tests/test_idr_burst.py`: with a fake `ReturnLink` (capture
  sent packets) and `asyncio.sleep` patched, assert:
  - One `trigger()` produces exactly `count - 1` packets after
    the trigger, all carrying `FLAG_IDR_REQUEST` with monotonically
    increasing sequence numbers.
  - Re-trigger mid-burst resets `_remaining` to `count - 1` and
    updates `_latest_decision`.
  - `enabled: false` produces zero burst packets.

### Modified tests

- `tests/test_profile.py`: add a case asserting that a profile
  containing `bitrate_Mbps` in `fec_table` raises `ProfileError`.
- `tests/test_policy.py` / `tests/test_policy_dual_gate.py`: any
  test asserting a specific `bitrate_kbps` value gets recomputed
  against the new formula. Tests asserting *which knobs changed*
  are unaffected.
- `tests/test_drone_e2e.py`: extend an existing loss-scenario
  test to assert the drone applier sees ≥2 IDR-flagged packets
  within the burst window. Tolerant assertion — exact count
  depends on UDP/asyncio timing under CI.

### Unaffected

- `tests/test_wire_contract.py` — wire schema unchanged; existing
  hex-diff contract still holds.
- `make -C drone test` — no drone-side code changes.

## Out of scope (for follow-up if/when desired)

- Adaptive online learning of safety margin / hysteresis /
  utilization (alink `[ml]`). Would slot in next to
  `BitrateConfig` as `AdaptiveBitrateConfig` if/when needed.
- Short-GI selection. Requires adding `data_rate_Mbps_SGI` to the
  profile YAML and a per-tick GI decision next to the MCS pick.
- Outcome-labeled telemetry. Would extend `flight_log.py` rather
  than introduce a parallel JSONL.

## Files touched

New:

- `gs/dynamic_link/bitrate.py`
- `gs/dynamic_link/idr_burst.py`
- `tests/test_bitrate.py`
- `tests/test_idr_burst.py`

Modified:

- `gs/dynamic_link/profile.py` — `FECEntry`/`MCSRow` schema +
  validator
- `gs/dynamic_link/policy.py` — `PolicyConfig`, cold-boot,
  `Policy.tick()` bitrate line
- `gs/dynamic_link/service.py` — load `BitrateConfig` /
  `IdrBurstConfig`, construct `IdrBurster`, call
  `idr_burster.trigger()` on idr-bearing decisions
- `conf/gs.yaml.sample` — add `policy.bitrate` and
  `policy.idr_burst` blocks
- `conf/radios/m8812eu2.yaml` — strip `bitrate_Mbps` from
  `fec_table` (16 lines)
- `docs/dynamic-link-design.md` — §4 / §6 wording update
- `docs/knob-cadence-bench.md` — wording update for the bitrate
  source of truth
- `tests/test_profile.py`, `tests/test_policy.py`,
  `tests/test_policy_dual_gate.py`, `tests/test_drone_e2e.py` —
  adjustments noted above

Drone code: untouched.
