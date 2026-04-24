# Phase 0 Implementation Guide

Companion to [dynamic-link-design.md](dynamic-link-design.md). The design doc tells
you *what* the controller does; this guide tells you *where it lives
in the code* and *how the pieces fit*.

**Scope:** Phase 0 only — the GS-side Python observer. No drone
applier, no wire serialiser, no MAVLink status channel. The service
subscribes to wfb-ng's JSON stats API, runs the §4 control loops,
and writes the decisions it *would* have sent to a log file.

---

## 1. Module overview

| Module | Responsibility | Key public types |
|---|---|---|
| `stats_client.py` | asyncio TCP client for wfb-ng's JSON stats API; parses newline-delimited JSON into typed events; reconnect + file-replay | `StatsClient`, `ReplayClient`, `RxEvent`, `SessionEvent`, `SettingsEvent`, `TxEvent`, `RxAnt`, `SessionInfo`, `parse_record`, `iter_events_from_reader` |
| `signals.py` | Per-100 ms window aggregator + EWMA smoother; computes §3 derived signals from an `RxEvent` stream | `Signals`, `SignalAggregator` |
| `profile.py` | Loads and validates radio-card profile YAML (§6.1); builds the runtime `rssi_mcs_map` | `RadioProfile`, `MCSRow`, `ProfileError`, `load_profile`, `load_profile_file` |
| `predictor.py` | Latency-budget predictor (§4); pure math; FEC-ladder constants | `Proposal`, `Prediction`, `PredictorConfig`, `BudgetExhausted`, `predict`, `fit_or_degrade`, `LADDER_STEPS`, `LADDER_DROP` |
| `policy.py` | Two-loop policy engine (§4.1 leading + §4.2 trailing) | `Policy`, `PolicyConfig`, `LeadingLoop`, `TrailingLoop`, `LeadingLoopConfig`, `CooldownConfig`, `FECBounds`, `SafeDefaults` |
| `decision.py` | Data class the policy returns on every tick | `Decision` |
| `sinks.py` | Writes decisions to stdout / file as one-line JSON | `LogSink` |
| `service.py` | Entry point — arg parsing, config loading, wiring, event loop, signal handling | `main`, `parse_args` |

Packaged artifacts:

- `conf/radios/m8812eu2.yaml` — default radio profile.
- `conf/gs.yaml.sample` — sample GS config.
- `packaging/dynamic-link-gs.service` — systemd unit.

---

## 2. Data flow

```
                                                  gs.yaml.sample
                                                   |
                                                   v
    wfb-ng (unchanged)                        PolicyConfig
      wfb_rx + server            conf/radios/m8812eu2.yaml
      JSON stats :8103                 |
           |                           v
           |                      RadioProfile
           |                     (rssi_mcs_map)
           |                           |
           v                           v
    +---------------+    RxEvent    +--------------+
    | StatsClient   |-------------->| Signal-      |
    | (asyncio TCP) |  SessionEvent | Aggregator   |
    +---------------+               | (§3 derived  |
                                    |  + EWMA)     |
                                    +------+-------+
                                           | Signals snapshot
                                           v
                                    +--------------+
                                    |   Policy     |
                                    |  +--------+  |
                                    |  |Leading |  | (§4.1)
                                    |  |Loop    |  |
                                    |  +---+----+  |
                                    |      |       |
                                    |  +---v----+  |
                                    |  |Trailing|  | (§4.2)
                                    |  |Loop    |  |
                                    |  +--------+  |
                                    |      |       |
                                    |  +---v----+  |
                                    |  |predictor| | fit_or_degrade
                                    |  +--------+  |
                                    +------+-------+
                                           | Decision
                                           v
                                     +------------+
                                     |  LogSink   |  (Phase 0 terminus)
                                     |  JSONL →   |
                                     |  --log-file|
                                     +------------+
```

Phase 2 swaps `LogSink` for a wire serialiser sending decision
packets to the drone's applier. The Phase 0 wiring is otherwise
unchanged.

### 2.1 Event types crossing the boundary

| Event | Produced by | Consumed by | Phase 0 behaviour |
|---|---|---|---|
| `SettingsEvent` | `StatsClient.connectionMade` equivalent (server's first line) | `service.on_event` | Logged at INFO |
| `RxEvent` | `rx` record at `log_interval` cadence | `SignalAggregator.consume` → `Policy.tick` → `LogSink.write` | The hot path |
| `SessionEvent` | `new_session` record on-change | `SignalAggregator.update_session` | Keeps the in-flight `(k, n, depth)` current |
| `TxEvent` | `tx` record | ignored | TX stats aren't used by Phase 0 policy |

---

## 3. Signal math

All formulas below are **§3 of the design doc**, re-expressed against
the actual JSON field names wfb-ng emits
(`wfb_ng/protocols.py:92-107`). The JSON `packets` dict maps each
counter to a `[window_count, cumulative_count]` tuple; we keep the
window value only.

### 3.1 Per-window raw counters (`Signals._w` fields)

```
tx_primaries   = out + lost                     # primaries the TX emitted
residual_loss_w = lost / tx_primaries           # 0.0 when tx_primaries == 0
fec_work_rate_w = fec_rec / tx_primaries
packet_rate_w   = data / 0.1                    # fragments/sec
burst_rate_w    = bursts_rec / 0.1
holdoff_rate_w  = holdoff / 0.1
late_rate_w     = late_deadline / 0.1
```

`data` includes primaries + parity, so it is *not* a loss-fraction
denominator — see the `count_p_outgoing + count_p_lost` discussion in
design doc §3 "Field reference".

### 3.2 Antenna reduction

One window may carry multiple `rx_ant_stats` entries (one per
`(freq, mcs, bw, ant)` key). We reduce across all of them:

```
rssi_min_w = min over antennas of rssi_min     # operational driver
rssi_avg_w = mean over antennas of rssi_avg    # diversity estimate
snr_min_w  = min over antennas of snr_min
snr_avg_w  = mean over antennas of snr_avg
```

`rssi_min_w` is the **weakest-antenna** number. On diversity setups a
per-antenna failure (obstructed, disconnected) must not hide the fact
that the surviving antenna is near its sensitivity floor.

### 3.3 EWMA smoothing

At 10 Hz sampling,  `α = 1 − exp(−Δt / τ)`. The configured defaults:

| Signal | α | Approx τ | Why |
|---|---|---|---|
| `rssi`, `snr` | 0.2 | 500 ms | Leading feed-forward; react fast but damp single-window fades. |
| `fec_work` | 0.2 | 500 ms | Headroom signal; moves roughly as fast as RSSI. |
| `burst_rate`, `holdoff_rate`, `late_rate` | 0.1 | 1 s | Infrequent per-window events; slower EWMA reduces noise without masking trends. |
| `residual_loss_w` | — | none | **Intentionally not smoothed** (§3). One lost block is a visible FPV glitch — the controller must react on the raw value. |

Implementation: `signals._ewma()` bootstraps on the first observation
(`prev is None → return new`), then runs `α·new + (1−α)·prev`.

### 3.4 State carried across windows

`SignalAggregator` holds one `Signals` dataclass across ticks. When
an `RxEvent` lands with no `rx_ant_stats` (no fragments this window),
the RSSI fields are *not* reset — the operating point doesn't vanish
just because a window was empty. Packet counters always reset to 0
implicitly via the fresh `packets_window` dict on each event.

---

## 4. Radio profile and rssi_mcs_map

### 4.1 Profile schema

`RadioProfile` is a validated, frozen dataclass built from a YAML
file under `conf/radios/*.yaml`. The packaged default is
`m8812eu2.yaml` (BL-M8812EU2 / RTL8812EU).

Validation rules (`profile._validate`):

1. `mcs_max ≤ 7` (design policy — 1T1R SISO only).
2. For every bandwidth in `bandwidth_supported`, every
   `mcs ∈ [mcs_min, mcs_max]` must have a `sensitivity_dBm` entry
   **and** a `data_rate_Mbps_LGI` entry.
3. `bandwidth_default` must be in `bandwidth_supported`.
4. `preferred_k[mcs] ∈ {2, 4, 6, 8}` for every `mcs` in range.
5. `preferred_k` is **monotone non-increasing** as MCS decreases
   (higher MCS uses bigger k blocks).
6. `encoder_bitrate_frac ∈ (0, 1]`.

### 4.2 Runtime row table

`RadioProfile.rssi_mcs_map(bandwidth, rssi_margin_db)` returns a list
of `MCSRow` rows, highest MCS first. Each row has:

```
rssi_floor_dBm = sensitivity_dBm[bw][mcs] + rssi_margin_db
bitrate_Mbps   = data_rate_Mbps_LGI[bw][mcs] * encoder_bitrate_frac
preferred_k    = preferred_k[mcs]
```

For the packaged `m8812eu2` HT20 profile at the default 8 dB margin
and 0.40 encoder fraction, this matches the row table in design doc
§4.1:

| MCS | rssi_floor (dBm) | bitrate (Mbps) | preferred_k |
|---|---|---|---|
| 7 | −69 | 26.0 | 8 |
| 6 | −72 | 23.4 | 8 |
| 5 | −75 | 20.8 | 6 |
| 4 | −77 | 15.6 | 6 |
| 3 | −80 | 10.4 | 4 |
| 2 | −83 |  7.8 | 4 |
| 1 | −85 |  5.2 | 2 |
| 0 | −88 |  2.6 | 2 |

### 4.3 Profile resolution order

`service._run` builds the search path as:

1. `<leading_loop.radio_profiles_dir>/<name>.yaml` — operator override.
2. `<repo>/conf/radios/<name>.yaml` — packaged default.

Operator overrides let you ship field-calibrated numbers without
editing packaged files.

---

## 5. Policy engine

### 5.1 Leading loop (§4.1) — `LeadingLoop`

Two nested controllers keyed by time constant:

**Outer — MCS row selector with hysteresis.** State lives in
`LeadingState.candidate_row_idx` + `candidate_seen_since_ts`.

- *Down move.* If smoothed `rssi` falls below the current row's
  floor, walk the row list to find the highest-MCS row whose floor
  still clears. Hold that candidate for `rssi_down_hold_ms` (default
  500 ms) before committing. Asymmetric: fast down, slow up.
- *Up move.* If `rssi` clears the next-higher row's floor + the
  `rssi_up_guard_db` guard (default 3 dB), enqueue an upgrade
  candidate. Hold for `rssi_up_hold_ms` (default 2000 ms) before
  committing. The guard keeps us from bouncing at row boundaries.
- *Idle.* When `rssi` is inside the current row's band, the
  candidate state is cleared.

Both fire conditions run in a single tick now (no artificial two-tick
delay on hold=0) — the hysteresis is purely the `seen_since_ts`
timestamp.

**Inner — TX-power closed loop.** Target `rssi_target_dBm`
(default −60 dBm), fired at most once per `tx_power_cooldown_ms`
(default 1 s). Anti-oscillation guards (all load-bearing):

| Guard | Default | Purpose |
|---|---|---|
| `rssi_deadband_db` | 3 dB | Ignore `rssi` within ±3 dB of target (RF jitter). |
| `tx_power_cooldown_ms` | 1000 ms | At most one step per second. |
| `tx_power_freeze_after_mcs_ms` | 2000 ms | Hold power steady 2 s after any MCS change (avoids 2-loop fights during radio re-init). |
| `tx_power_step_max_db` | 3 dB | Clamp any single-step delta — even a 15 dB fade moves at most 3 dB per step. |
| `tx_power_gain_up_db` / `tx_power_gain_down_db` | 1.0 / 1.0 | Optional asymmetry (raise eagerly, drop conservatively). |

The power tick runs unconditionally at the end of `LeadingLoop.tick`;
the guards decide whether anything actually happens.

**Forced MCS drop.** When the trailing loop detects sustained loss
(§4.2), it calls `LeadingLoop.tick(..., forced_mcs_drop=True)`, which
steps the current row down one immediately, bypassing hysteresis.

### 5.2 Trailing loop (§4.2) — `TrailingLoop`

Fires on the raw (non-smoothed) `residual_loss_w`. Three paths:

| Trigger | Action |
|---|---|
| `residual_loss_w > 0` this window | Step `(k, n)` one notch up the §4.2 ladder; set `idr_request=True`; bypass `min_change_interval_ms_fec` cooldown (a glitch is a glitch). |
| Plus `burst_rate > 1 /s` **and** `holdoff_rate > 0 /s` | Also raise `depth` by 1 (subject to `min_change_interval_ms_depth` cooldown and `fec.depth_max` ceiling). |
| `residual_loss_w == 0` **and** `fec_work > 0.05` | Preemptive `(k, n)` step; respects `min_change_interval_ms_fec`; no IDR. |

The §4.2 ladder table (constants in `predictor.LADDER_STEPS` /
`LADDER_DROP`):

```
 k=8:  (8,12) → (8,14) → (8,16) → [(6,12) via LADDER_DROP]
 k=6:  (6,10) → (6,12) → (6,14) → [(4, 8) via LADDER_DROP]
 k=4:  (4, 8) → (4,10) → (4,12) → [(2, 6) via LADDER_DROP]
 k=2:  (2, 4) → (2, 6) → (2, 8) → [(1, 4) via LADDER_DROP]
```

`_ladder_step_up(k, n)` walks the current band; at the band's top it
falls through to `LADDER_DROP` for the next-lower band's floor.

**Sustained-loss detection.** `TrailingLoop.state.recent_loss_windows`
is a short sliding window of booleans, length
`cfg.sustained_loss_windows` (default 3). `sustained_loss()` returns
True when every slot is True — consumed by `Policy.tick` to decide
whether to flip `forced_mcs_drop` on the leading loop.

### 5.3 Top-level composition (`Policy.tick`)

One call per `RxEvent`:

```
1. Check TrailingLoop.sustained_loss(). If true AND this window has
   loss → request a forced MCS drop on the leading loop this tick.

2. LeadingLoop.tick(signals.rssi, ts_ms, forced_mcs_drop) returns
   (row, tx_power, mcs_changed).

3. If mcs_changed: rebase (k, n) to the new band's floor
   (§4.2 "band boundary crossings").

4. TrailingLoop.tick(signals, k, n, depth) returns the next
   (k, n, depth, idr_request).

5. fit_or_degrade on the proposed (k, n, depth) against
   max_latency_ms. Drops depth first, then k-band; refuses
   (BudgetExhausted) if no combination fits.

6. Commit state, assemble the Decision, record knobs_changed.
```

`knobs_changed` is a list of strings (`"mcs"`, `"bitrate"`,
`"tx_power"`, `"fec"`, `"depth"`, `"idr"`) that actually moved this
tick. Phase 0's `LogSink` uses this to suppress steady-state ticks
unless `--verbose` is passed.

---

## 6. Latency predictor

Formula from
[wfb-ng/doc/design/fec-enhancements-v2.md §4.2](../wfb-ng/doc/design/fec-enhancements-v2.md):

```
latency_block = block_fill_time + block_airtime + fec_decode_time
block_fill_time  = k × inter_packet_interval_ms
block_airtime    = n × per_packet_airtime_us / 1000
fec_decode_time  ≈ 1.0 ms  (zfex SIMD at reference (k=8, n=12))
latency_total    = latency_block + (depth − 1) × block_duration_ms
```

`block_duration_ms` is the "+interleave" overhead per depth step at
the reference operating point — a configured constant (12 ms default,
matching the §4.2 worked-example table).

### 6.1 `fit_or_degrade` priority

When the proposed `(k, n, depth)` exceeds `max_latency_ms`:

1. Drop `depth` by 1 (reclaims one `block_duration_ms`).
2. If `depth == 1` and still over, drop to the next-lower band's
   floor via `LADDER_DROP`.
3. If already `(2, 4, 1)` and still over — raise `BudgetExhausted`.

Controller holds current state and flags `budget_exhausted` in the
Decision reason string. In Phase 1+, this surfaces to the GS
operator and (optionally) the flight controller.

### 6.2 Reference numbers

At `PredictorConfig` defaults (MCS7 HT40, 80 µs/pkt, 1.4 ms
inter-packet):

| `(k, n, d)` | Predicted | §4.2 worked example |
|---|---|---|
| (8, 12, 1) | ~13.2 ms | 12 ms |
| (8, 14, 3) | ~37.3 ms | 37 ms |
| (8, 14, 4) | ~49.3 ms | 49 ms (at the cap) |

Tests in `tests/test_predictor.py` assert within ±2 ms of the
worked-example values — close enough for the Phase 0 observer,
exact enough to catch regressions.

---

## 7. Configuration reference

### 7.1 `gs.yaml`

Top-level keys consumed by `service._build_policy_config`:

| Key path | Consumer | Default |
|---|---|---|
| `enabled` | informational only in Phase 0 | `false` |
| `wfb_ng.stats_api` | `StatsClient` endpoint URL | `tcp://127.0.0.1:8103` |
| `video.framerate` | reserved for future depth-ceiling logic | 60 |
| `video.per_packet_airtime_us` | `PredictorConfig.per_packet_airtime_us` | 80 |
| `video.max_latency_ms` | `PolicyConfig.max_latency_ms` | 50 |
| `fec.*` | `FECBounds` (n_min, n_max, k_min, k_max, depth_max) | 4 / 16 / 2 / 8 / 3 |
| `leading_loop.*` | `LeadingLoopConfig` — every field maps 1:1 | see `LeadingLoopConfig` defaults |
| `leading_loop.radio_profile` | `load_profile()` name | `m8812eu2` |
| `leading_loop.radio_profiles_dir` | override search path | `/etc/dynamic-link/radios` |
| `smoothing.ewma_alpha_*` | `SignalAggregator` alpha constants | 0.2 / 0.2 / 0.1 |
| `cooldown.*` | `CooldownConfig` (fec, depth, radio, cross) | 200 / 200 / 500 / 50 ms |
| `safe_defaults.*` | `SafeDefaults` (k, n, depth, mcs, bitrate_kbps) | 8 / 12 / 1 / 1 / 2000 |

### 7.2 wfb-ng prerequisite

Set `log_interval = 100` in the `[common]` section of wfb-ng's
`master.cfg` on the GS host. This is wfb-ng-wide — `wfb-cli` tables
will animate 10× faster, but rate calculations stay correct (wfb-cli
divides byte counts by `log_interval`).

Without this change the stats feed arrives at 1 Hz, far too slow for
the controller's §3 cadence; the policy engine will still run but
its reactions will lag by hundreds of milliseconds.

### 7.3 Port naming note

The design doc refers to the stats endpoint as `stats_port`, but
wfb-ng's `master.cfg` reserves that name for the **msgpack** feed
wfb-cli uses (8002/8003). The **JSON** feed dynamic-link consumes is
bound to `api_port` (8102/8103). `gs.yaml.sample` uses an
endpoint-URL (`tcp://host:port`) to sidestep the naming ambiguity.

---

## 8. Running and testing

### 8.1 Install

```
pip install -e '.[dev]'
```

### 8.2 Against a live wfb-ng

```
dynamic-link-gs --config conf/gs.yaml.sample --log-file /tmp/dl.log
tail -f /tmp/dl.log
```

Steady-state ticks are suppressed by default; pass `--verbose` to log
every tick (useful for validating the smoothing / row hysteresis on
bench captures).

### 8.3 Replay a captured stream

```
nc 127.0.0.1 8103 > capture.jsonl               # one-time capture
dynamic-link-gs --config conf/gs.yaml.sample \
                --replay capture.jsonl \
                --log-file /tmp/dl.log
```

`ReplayClient` streams the file into the same `on_event` callback the
live client uses — everything downstream is identical.

### 8.4 Tests

```
pytest
```

Six suites:

- `test_profile.py` — YAML loader + §6.1 validation against packaged
  profile and crafted-bad fixtures.
- `test_signals.py` — §3 derivations, EWMA bootstrap + α values,
  min-across-antennas, `residual_loss_w` left unsmoothed.
- `test_predictor.py` — §4.2 worked-example numbers, `fit_or_degrade`
  priority (depth before k), `BudgetExhausted` on impossible caps.
- `test_policy_leading.py` — MCS hysteresis holds; TX-power
  dead-band, cooldown, MCS-freeze; step clamp; forced MCS drop.
- `test_policy_trailing.py` — `residual_loss_w > 0` fires same tick
  with IDR; ladder `(8,12)→(8,14)→(8,16)→(6,12)`; depth gated on
  burst + holdoff together; preemptive step on `fec_work` respects
  cooldown.
- `test_stats_client.py` — JSON parser for all record types;
  `contract_version` rejection; malformed-line tolerance; replay
  end-to-end.

---

## 9. What's deliberately not in Phase 0

Deferred per design doc §7:

- **Wire serialiser** (`gs/dynamic_link/wire.py`) — the
  decision-packet struct matching the drone applier's `dl_applier.h`.
  Phase 2.
- **Return-link writer** (`return_link.py`) — raw UDP into wfb-ng's
  `tunnel` stream. Phase 2.
- **Oscillation detector** (`oscillation.py`, GS failsafe 2). Phase 2.
- **MAVLink status reader** (`mavlink_status.py`) — consumes the
  drone applier's rejection / status messages. Phase 2.
- **Drone applier** — the C binary, backends, ceilings, watchdog,
  OSD output. Phase 1.

`Decision` already carries every field the Phase 2 wire packet needs
(`mcs`, `bandwidth`, `tx_power_dBm`, `k`, `n`, `depth`,
`bitrate_kbps`, `idr_request`); the `reason` / `knobs_changed` /
`signals_snapshot` fields stay behind as local introspection.
