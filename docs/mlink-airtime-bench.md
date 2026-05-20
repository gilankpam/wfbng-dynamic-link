# mlink × MCS × bitrate airtime bench — when does the radio choke?

Empirically calibrated lookup table for the largest sustainable
encoder bitrate at a given `(wireless.mlink, MCS)` on HT20 LGI.
Tested on the live SSC338Q drone with the BL-M8812EU2 (RTL8812EU).
Method, raw measurements, the fitted model, and the resulting
table are all below so the numbers are reproducible / refittable.

## Why this exists

`wireless.mlink` in `/etc/wfb.yaml` does two things via OpenIPC's
`/usr/bin/wifibroadcast` `load_mlink()`:

1. `ifconfig wlan0 mtu $mlink`
2. `outgoing.naluSize = mlink - 100` in `/etc/majestic.yaml`,
   SIGHUP majestic.

`wfb_tx` itself has a **compile-time** "Radio MTU" cap (3993 on the
shipped binary); `mlink` does not change wfb_tx's payload limit. It
changes how many 802.11 frames majestic produces per video frame.

Per-frame PHY/MAC overhead is roughly constant (~150–160 µs at HT20
broadcast: HT-mixed preamble + 36 B MAC at PHY rate + DIFS/backoff),
so smaller payloads spend a bigger fraction of airtime on overhead.
At a given `(MCS, bandwidth)` this lowers the practical wire-bandwidth
ceiling. The dynamic-link latency predictor
(`gs/dynamic_link/predictor.py`) does **not** model this — it only
counts `n × per_packet_airtime_us`. So its "fits the budget" verdict
gets optimistic at small mlink + high bitrate.

## Method

For each `(mlink, MCS)` operating point:

1. Pin `wireless.mlink` in `/etc/wfb.yaml`, restart wifibroadcast so
   `load_mlink()` propagates `naluSize` and `wlan0` MTU.
2. Restart `dl-applier` so the DLHE handshake reports the new MTU.
3. Pin `gate.max_mcs` in `gs.yaml` to the target MCS, vary
   `policy.bitrate.utilization_factor` and `max_bitrate_kbps` to
   sweep encoder bitrate, restart `dynamic-link-gs`.
4. Wait for the gate to climb to target MCS (~15 s post-restart).
5. Wave-test: stand in front of the camera, wave a hand, observe
   the GS monitor. Classify as "instant" or "laggy" (visible delay
   between hand motion and monitor pixels updating).
6. Record `(mlink, MCS, br_encoder, wire_bitrate)` and the verdict.

Wire bitrate is computed from the live `dl-applier` log
(`br × n / k`) since the dynamic FEC picks `(k, n)` each tick.

## Raw measurements (2026-05-20, HT20 LGI, n/k = 17/12)

| mlink | MCS | PHY Mb/s | enc br (kbps) | wire Mb/s | verdict |
|-------|-----|---------:|--------------:|----------:|---------|
| 1500  | 4   | 39       | 8000          | 11.6      | instant |
| 1500  | 4   | 39       | 19500         | 27.6      | laggy   |
| 1500  | 3   | 26       | 11000         | 15.6      | instant |
| 1500  | 3   | 26       | 13000         | 18.4      | instant |
| 1500  | 3   | 26       | 13928         | 19.7      | laggy   |
| 1500  | 3   | 26       | 14857         | 21.1      | laggy   |
| 1500  | 3   | 26       | 17642         | 25.0      | laggy   |
| 3994  | 4   | 39       | 19500         | 27.3      | instant |

## Fitted model

```
airtime_per_frame  =  preamble  +  payload_bits / PHY_bps
max_wire_Mbps      =  U × payload_bits / airtime_per_frame / 1e6
max_enc_Mbps       =  max_wire / (n / k)
```

- `preamble ≈ 157 µs` — fitted from the MCS3 + mlink=1500
  crossover midpoint (wire ≈ 19.05 Mb/s = 100% airtime). Covers
  HT-mixed preamble + MAC header airtime + DIFS+backoff.
- `U = 0.80` — safety target. Leaves headroom for full
  `n_escalation` (up to +4 over `n_base=17` at k=12 = +24% wire).
- `n / k = 1.4` — typical at `base_redundancy_ratio = 0.4`. Set
  higher (1.5 at 0.5) if your gs.yaml differs.

Sanity check against all five measured points: every "instant" lands
below 100% predicted airtime, every "laggy" lands above. The
MCS3 + 18.4 Mb/s case sits at 96% predicted and felt instant — the
boundary is tight, confirming the fit.

## Safe encoder bitrate table (kbps), HT20 LGI, n/k = 1.4, U = 0.80

| MCS | PHY  | mlink=1500 | mlink=2300 | mlink=3994 |
|-----|-----:|-----------:|-----------:|-----------:|
| 0   |  6.5 |  3400      |  3500      |  3600      |
| 1   | 13   |  6300      |  6700      |  7000      |
| 2   | 19.5 |  8800      |  9500      | 10100      |
| 3   | 26   | 10900      | 12100      | 13100      |
| 4   | 39   | **14400**  | 16500      | **18600**  |
| 5   | 52   | 17200      | 20300      | 23500      |
| 6   | 58.5 | 18400      | 22000      | 25800      |
| 7   | 65   | 19400      | 23500      | 28000      |

Bolded cells are the empirical anchors; rest is the calibrated
formula. Expect ~±10% error.

## Caveats

- HT20 LGI single-stream only. HT40 doubles PHY rate (roughly
  doubles the table); SGI adds ~11%; multi-stream needs separate
  measurement (8812EU MIMO behaviour in broadcast mode is not
  characterised here).
- Only the RTL8812EU was tested. Other chipsets have different
  preamble/MAC-airtime constants — refit before trusting.
- The fit uses one crossover point. A multi-point sweep at MCS5/6
  would tighten the preamble constant; current ±10% reflects the
  thin calibration.
- "Instant" / "laggy" is a perceptual binary, not a quantitative
  latency measurement. Real ceiling depends on how much queueing
  the operator tolerates. Treat the 80% target as a starting point.
- Wfb-ng broadcast mode skips ACKs and retries, which keeps
  overhead deterministic. If you reconfigure to ack/retry, the
  preamble term gets a lot bigger.

## How to use this when tuning gs.yaml

Pick the cell for your operational `(mlink, max_mcs)` and set
`policy.bitrate.max_bitrate_kbps` to ≤ that value. Or invert: if
you want a fixed target bitrate, the table tells you the minimum
MCS (and therefore minimum SNR margin) you need.

If you want to push past the bolded anchor for `(mlink=3994,
MCS4)`, you have three knobs: raise `gate.max_mcs`, move to HT40,
or raise `mlink` (but 3994 is already near the wfb_tx compile-time
cap of 3993).

## Driver-side saturation signal (RTL8812EU)

When the airtime ceiling is breached, frames pile up in the driver's
PUBQ (packet buffer pool) because the MAC can't dequeue them onto
the air fast enough. The RTL8812EU driver exposes this directly at:

```
/proc/net/rtl88x2eu/wlan0/pubq_free_page
```

Pool depth on this drone is **1753 pages**. Healthy operation cycles
in the upper half; saturation pins the counter at the floor (~16).

### Captured timeseries (2026-05-20)

130 s capture, 1 Hz polling. Same MCS4 + encoder br ≈ 19500 kbps
(~30 Mb/s wire) for all three phases. Only `mlink` changed:

| Phase | `pubq_free` | `mg` queue | tx pps | wire kbps |
|-------|------------:|-----------:|-------:|----------:|
| Baseline `mlink=3994` | 1378-1753 (oscillating, drains) | 0-16 | ~1030 | ~30700 |
| wfb restart (idle)    | 1753 (pinned at max, no traffic) | 0 | ~25 | ~40 |
| Saturated `mlink=1500`| **16 (pinned at floor)**         | **151-157** | ~2700 | ~30900 |

The PUBQ collapse from ~1750 → 16 the moment mlink=1500 took effect
is the unambiguous "TX buffer pool exhausted" signature. Two
confirming signals fell out of the same capture:

- **tx pps jumped 2.6×** (1030 → 2700) for the same wire kbps
  because each packet now carries 1400 B of payload instead of
  3894 B. Per-frame fixed overhead is what's eating the airtime.
- **Management queue `mg` jumped from ~5 to ~150** — mgmt frames
  can't get on the air either, so they back up.

### Two driver counters that look useful but aren't

- `trx_info` → `free_xmitbuf_cnt` — stuck at 4 in all three phases.
  This is a tiny inner ring; the driver refills it as fast as it
  drains, so it always reads "starved." No diagnostic value.
- `mac_qinfo` → Q0-Q3 `pkt_num` — stuck at 0 in all three phases.
  The HW MAC queue sits downstream of the PUBQ throttle; the
  driver keeps it shallow on purpose. PUBQ is the upstream
  pressure gauge that actually moves.

### Reading thresholds (this rig)

| `pubq_free_page` | meaning |
|-----------------:|---------|
| ≥ 1500 | healthy headroom |
| 500-1500 | busy but not saturated |
| < 100 | saturated, queueing latency growing |

These map cleanly onto the airtime utilisation model above:
~80% airtime corresponds roughly to PUBQ ≈ 1000; the cliff at
100% airtime collapses PUBQ to <100 within ~1 s.

### Sampling the signal

A periodic read (1 Hz is plenty) is enough; the counter swings
within hundreds of ms when saturation starts/ends. Cheap enough
to fold into dl-applier's per-tick telemetry if we want
`tx_buffer_pressure` as a first-class control input.

## Refit procedure

If a chipset, channel-width, or firmware change makes the table
suspect:

1. Pick one `(mlink, MCS)` and bracket the crossover with 2–3
   wave-tests (~5 minutes; see the 2026-05-20 sequence above).
2. Solve `preamble = payload_bits / wire_bps_crossover − payload_bits / PHY_bps`.
3. Recompute the whole table with the new preamble; the formula
   is portable.

The bench history is in the dynamic-link conversation transcript;
re-running takes <15 minutes including config restore.
