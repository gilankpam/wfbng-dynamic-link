# Knob-cadence bench — empirical cost of MCS / FEC / bitrate changes

Bench validation of the cost of bundled and high-cadence
controller-knob changes. Drives the choice of
`min_change_interval_ms_*` defaults in §4 of the design doc and
explains why **MCS thrash hurts the control plane while FEC
thrash hurts the data plane**.

## Setup

- **Date**: 2026-04-28
- **Hardware**: OpenIPC SSC338Q drone (channel 132, 5660 MHz,
  20 MHz width, encoder = majestic) + OpenIPC GS (dual
  84:fc:14:6c:36:e6 / :f4 monitor radios).
- **GS controller**: `enabled: false` (observer mode) — controller
  computes decisions but does not emit. All knob changes were
  driven by `dl-inject` from the drone targeting `127.0.0.1:9999`,
  so the only thing pressuring the radio chain was the test.
- **Video tap**: GS port 5601, fed by the operator's existing
  gst-launch tee (`udpsrc 5600 ! tee ! udpsink 5601 + 5602`).
- **PONG cadence**: 5 Hz (default).
- **Per-flight log dir**: `/var/log/dynamic-link/flights/flight-NNNN/`
  rotated on every service restart.

## Methodology

For each run:

1. Restart `dynamic-link-gs` on GS → fresh flight dir.
2. Restart `dl-applier` on drone → fresh `applier.log`.
3. Wait 5 s, verify `video_rtp.jsonl` and `latency.jsonl` are
   growing.
4. Run `dl-inject` in a busybox shell loop on the drone for
   60 s with the run's specific schedule (cadence + per-step
   `(bitrate, mcs, k, n)` tuple).
5. `scp` the GS flight dir + drone applier log to a workstation;
   run `dl-report` and compute drift-swing percentiles.

Per-tick metrics:

- `latency_drift_us` — RTP-elapsed vs gs-mono-elapsed gap,
  anchored on the first packet of the flight. Only the
  *changes* are meaningful (see `gs/dynamic_link/video_tap.py`
  docstring); absolute baseline is arbitrary.
- `200 ms drift swing` — sliding-window `max - min` of drift
  over a 200 ms window. Captures the per-transition disturbance.
- RTT comes from `latency.jsonl` PONG round-trips.

## Run matrix

|  | 1 Hz | 2 Hz | 10 Hz |
|---|---|---|---|
| Bitrate only (MCS+FEC fixed at 5 / 8 / 12) | ✓ | — | ✓ |
| Bitrate + bundled (MCS, FEC k/n) per step | ✓ | ✓ | ✓ |
| Bitrate + MCS only (FEC pinned 6/9) | — | — | ✓ |
| Bitrate + FEC only (MCS pinned 5) | — | — | ✓ |

The bundled ladder steps were:

| step | bitrate | MCS | k / n | utilization at MCS goodput |
|---|---|---|---|---|
| 1 | 4 Mbps | 1 | 4 / 8 | 62% |
| 2 | 8 Mbps | 2 | 6 / 9 | 47% |
| 3 | 12 Mbps | 4 | 8 / 12 | 35% |
| 4 | 16 Mbps | 5 | 8 / 10 | 34% |

Each rung lives well inside its own MCS's goodput envelope —
no run was deliberately over-driven, so any stress observed
came from the *transition*, not from a saturated steady state.

## Results

| Run | swing p50 | swing p95 | swing max | RTT p99 | RTT max | RTT outliers | drift range | frame ia max | post-FEC loss | emergencies |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 Hz, bitrate only | 8.4 | 26.6 | 27 (after warmup) | 31.7 | 36.0 | 0 | 71.5 | 66.6 | 0.000% | 0 |
| 10 Hz, bitrate only | 16.4 | 35.9 | 104.7 | 34.8 | 52.8 | 0 | 106.3 | 65.3 | 0.003% | 0 |
| 1 Hz, full ladder | 10.8 | 48.1 | 105.9 | 83.1 | 136.8 | 5 | 147.2 | 72.5 | 0.000% | 0 |
| 2 Hz, full ladder | 32.9 | 68.3 | 115.5 | 123.8 | 135.5 | 18 | 147.1 | **175.9** | 0.003% | 0 |
| 10 Hz, full ladder | 33.6 | 79.4 | 171.4 | 829.7 | 867.3 | 33 | 390.6 | 94.6 | 0.000% | **2 (14–15% bursts)** |
| 10 Hz, FEC=6/9 fixed (MCS varies) | 21.2 | 42.2 | 61.7 | 68.7 | 96.4 | 1 | 78.1 | 59.2 | 0.000% | 0 |
| 10 Hz, MCS=5 fixed (FEC varies) | 21.8 | 83.1 | 403.3 | 46.8 | 54.3 | 0 | 134.7 | 115.5 | 0.000% | 0 |
| 5 Hz, full ladder (waybeam, stagger=50 ms) #pilot | 13.8 | 33.8 | 107.4 | 35.3 | 55.3 | **0** | 110.8 | 74.6 | 0.013% | 5 |
| 5 Hz, full ladder (waybeam, stagger=50 ms) #1 | 21.0 | 35.0 | 68.4 | 44.3 | 49.6 | 0 | 748.7 | 75.9 | 0.000% | — |
| 5 Hz, full ladder (waybeam, stagger=50 ms) #2 | 20.5 | 35.6 | 113.3 | 48.5 | 81.3 | 1 | 975.2 | 60.6 | 0.000% | — |
| 5 Hz, full ladder (waybeam, **stagger=0**) #1 | 22.7 | 64.3 | 117.5 | 89.8 | 108.2 | 7 | 976.1 | 87.2 | 0.000% | — |
| 5 Hz, full ladder (waybeam, **stagger=0**) #2 | 23.5 | 64.6 | 97.4 | 82.4 | 85.0 | 6 | 907.6 | 81.2 | 0.009% | — |

Units: drift / RTT / frame interarrival in **ms**. "swing" =
sliding 200 ms drift max-min. "frame ia max" = worst single
inter-frame gap. "Emergencies" = controller-side observed loss
≥5% events (would-emit decisions in observer mode; not actually
applied).

## Findings

### 1. Bitrate change alone is cheap when the link has headroom

At 1 Hz with MCS pinned at 5 and FEC at 8/12, a 4-step bitrate
sweep (4 → 8 → 12 → 16 Mbps) produces ~10 ms drift swings, no
measurable RTT impact, and zero loss. The encoder buffer
breathes a bit on each step but the link is undisturbed.

### 2. Cadence raises bitrate-only cost roughly linearly

10 Hz with the same fixed (MCS, FEC) doubles the typical drift
swing (8.4 → 16.4 ms) and adds first-appearance video loss
(0.003%, all FEC-recovered). RTT mostly unaffected. This is the
encoder-pipeline cost of bursting bitrate too quickly to settle
between steps.

### 3. Bundling MCS+FEC+bitrate at 1 Hz roughly doubles disturbance again

Same cadence (1 Hz), but each step now also retunes MCS and
FEC. Drift range climbs from 71 → 147 ms (×2.1), RTT max from
36 → 137 ms (×3.8). The cost shows up across both planes. No
loss yet.

### 4. Bundling × cadence is multiplicative — not additive

10 Hz full ladder is the worst run by every metric: drift
range 391 ms, RTT max 867 ms, 33 RTT outliers, and **two
real loss bursts of 14–15%** observed by GS. Each axis cost ~2×
on its own; together they cost ~5–25×. The radio reconfig path
does not tolerate the combined pressure: airtime backs up,
fragments collide, FEC ramps up to compensate.

### 4b. Direction-aware staggered apply substantially flattens the cadence curve

A matched A/B on **2026-05-03** measured the bundled 4-step ladder
at **5 Hz** — the cadence the prior bench labelled *broken*
(`≥ ~5 Hz`) — toggling only `apply_stagger_ms` (50 vs 0) on the
drone. Encoder, GS, sweep schedule, and link conditions held
constant across all four runs.

Setup, common to all rows below:

- Encoder: `waybeam_venc` (binary `/usr/bin/venc`).
  API-compatible with majestic (`GET /api/v1/set?…`,
  `docs/encoder_api.md`).
- Applier: direction-aware staggered apply (commit `4de7f6d`).
  Toggled per run via `apply_stagger_ms` in `drone.conf`. With
  staggering on, every bundled transition was confirmed to fire
  two phases ~50 ms apart in `applier.log`; with it off, the
  same transitions ran single-shot (zero `phase2` log lines).
- GS: observer mode (`enabled: false`); `dl-inject` from the
  drone drove the radio.
- Sweep: 60 s, 5 Hz, ladder rungs `(4 Mbps / MCS1 / 4–8) →
  (8 / MCS2 / 6–9) → (12 / MCS4 / 8–12) → (16 / MCS5 / 8–10)`.

Four runs, two per side:

| Run | swing p50 | swing p95 | swing max | RTT p99 | RTT max | RTT outliers | frame ia max | loss % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| stagger=50 #1 | 21.0 | 35.0 | 68.4 | 44.3 | 49.6 | 0 | 75.9 | 0.000 |
| stagger=50 #2 | 20.5 | 35.6 | 113.3 | 48.5 | 81.3 | 1 | 60.6 | 0.000 |
| stagger=0  #1 | 22.7 | 64.3 | 117.5 | 89.8 | 108.2 | 7 | 87.2 | 0.000 |
| stagger=0  #2 | 23.5 | 64.6 | 97.4  | 82.4 | 85.0  | 6 | 81.2 | 0.009 |

Group means:

| Metric | stagger=50 (n=2) | stagger=0 (n=2) | Δ |
|---|---:|---:|---|
| swing p50 (ms) | 20.8 | 23.1 | +11 % |
| swing p95 (ms) | **35.3** | **64.5** | **+83 % (1.83×)** |
| swing max (ms) | 90.9 | 107.5 | +18 % |
| RTT p99 (ms) | **46.4** | **86.1** | **+86 % (1.86×)** |
| RTT max (ms) | 65.5 | 96.6 | +47 % |
| RTT outliers | **0.5** | **6.5** | **13×** |
| Frame ia max (ms) | 68.3 | 84.2 | +23 % |
| Video loss (%) | 0.000 | 0.005 | first sub-percent appears |

Read:

- **Swing p95 and RTT p99 / outliers are the cleanest signals.**
  Within-group variance is tiny (swing p95: 35.0 vs 35.6 with
  stagger on; 64.3 vs 64.6 with stagger off), between-group
  difference is ~30 ms — clearly not noise. Same for outlier count
  (0–1 vs 6–7).
- **Swing max and drift range are noisy.** Each side had one run
  with a fat-tail spike, so `max` alone is ambiguous; use p95.
- **First sub-percent video loss appears with stagger off.**
  At 5 Hz × 60 s × 4 rungs that's a credible boundary signal:
  with the stagger absorbing the inter-stage burst, frames stayed
  intact across all transitions; without it, one of the two runs
  edged into real (if tiny) loss territory.
- **Steady-state cost of staggering is zero.** Median swing is
  unchanged within noise; the gap costs nothing on stable links.

The cadence curve really did shift: the 5 Hz bundled regime that
the prior bench placed firmly in the *broken* band now sits inside
the *tolerable* band on every control-plane metric, and the
isolation run shows the staggered apply is what moved it there.

Remaining caveats:

- **n = 2 per side.** Strong directional signal but the small-n
  variance estimate is itself noisy. Worth re-running if a
  decision rides on this.
- **Single airframe, single bench location.** No fade-margin or
  multi-link contention. Fade conditions could change the
  relative cost of the unstaggered queue burst.
- **5 Hz only.** Higher cadences (10 Hz bundled) weren't re-run
  with the stagger toggle. The original 10 Hz bundled run
  produced real loss bursts; whether the stagger rescues that
  regime or merely shifts the cliff later is open.
- **`encoder_kind = majestic` is still set in `drone.conf`** —
  harmless because the API path is shared, but a label drift to
  fix.

### 4a. The cost transition is sharp around 1–2 Hz, not gradual

The 2 Hz full-ladder run shows that bundled-change cost saturates
fast: between 1 Hz and 2 Hz, `swing_p50` triples (10.8 → 32.9 ms),
RTT outliers jump from 5 to 18, and the worst single-frame stall
peaks at **176 ms** (~10 frames at 60 fps — a clearly visible
stutter). Going from 2 Hz to 10 Hz changes the *control plane*
(RTT max 135 → 867 ms) and adds real loss bursts, but the
data-plane drift swing barely moves further. Two regimes:

| regime | bundled cadence (stagger=0) | character |
|---|---|---|
| tolerable | ≤ 1 Hz | encoder buffer + radio chain recover between transitions |
| stressed | ~2 Hz | most disturbance metrics saturate; first sub-percent loss |
| broken | ≥ ~5 Hz | control plane backs up; real loss bursts; emergencies |

The 1→2 Hz step looks like crossing a settling-time threshold:
at 1 Hz the radio chain finishes draining the previous reconfig
before the next one lands; at 2 Hz it doesn't, and most of the
disturbance budget is already consumed.

The matched A/B in §4b shows the staggered apply moves the 5 Hz
bundled regime back into the *tolerable* band on swing-p95 and
RTT-p99, so the column header above explicitly qualifies it as the
**stagger=0** baseline. The 10 Hz bundled cliff has not been
re-tested with stagger on.

### 5. MCS thrash and FEC thrash hurt **different planes**

The two isolation runs at 10 Hz make this clear:

- **MCS varies, FEC pinned** → small data-plane impact (frame
  ia max 59 ms, swing max 62 ms, drift range 78 ms) but
  meaningful **control-plane impact** (RTT max 96 ms, p99 69 ms).
  Verdict mix: "Link stress" — the classifier sees RTT spikes
  as queue pressure.
- **FEC varies, MCS pinned** → minimal control-plane impact
  (RTT max 54 ms, **0 outliers**) but big **data-plane impact**
  (frame ia max **115 ms**, single drift swing **403 ms**).
  Verdict mix: "Encoder pipeline" — classifier sees data-plane
  stalls without RTT pressure.

Plausible mechanism:

- MCS change → wfb_tx reconfigures the radio's per-frame rate
  selector. Queue contents are untouched, but anything that
  has to *transit* during the reconfig is briefly held — and
  PONG packets are small/regular, so they take the visible hit.
- FEC k/n change → wfb_tx rewrites the block structure. Frames
  already queued have to ride out the in-progress FEC block
  before the new k/n applies, producing a multi-frame stall.

### 6. Operational ranking (high cadence)

For an FPV use case where smooth video matters more than control-
plane RTT precision, the ranking from best to worst at 10 Hz is:

1. 🥇 **FEC fixed, MCS varies** (drift max 62, range 78, no loss).
2. 🥈 **MCS fixed, FEC varies** (RTT max 54 but drift max 403).
3. ⚠️ **Both vary** (real loss bursts, RTT 867 ms).

The absolute best across all runs is still **don't change MCS
or FEC at all** if the link can carry the offered bitrate
(1 Hz bitrate-only: drift max 27, RTT max 36).

## Implications for controller cooldown defaults

This bench supports the existing per-knob rate limits in
`gs.yaml.sample`:

| knob | current default | bench evidence |
|---|---|---|
| `min_change_interval_ms_radio` (MCS, TX power) | 500 ms (≈2 Hz) | MCS thrash bites the control plane at 10 Hz; 2 Hz is comfortably below |
| `min_change_interval_ms_fec` (k, n) | 200 ms (≈5 Hz) | FEC thrash bites the data plane at 10 Hz; 5 Hz is the borderline — fine here, would want a fade-margin run before tightening |
| `min_change_interval_ms_depth` | 200 ms | not stressed by this bench |
| `min_change_interval_ms_cross` (any cross-knob change) | 50 ms | **too short for bundled MCS+FEC changes** — bench shows 2 Hz bundled is already in the stressed regime, so cross-knob bundling needs ≥ 1 s cooldown. The 50 ms value protects against tick-level races, not airtime pressure |

Practical guidance for tick-level decision logic:

- **Change MCS responsively, change FEC reluctantly.** MCS can
  absorb a fast cadence; FEC k/n is a structural parameter
  whose change-cost shows up in the goggle stream.
- **Avoid bundling MCS and FEC changes in the same tick** —
  the disturbances compound non-linearly, especially at high
  cadence. The cooldown blocks (`§4` of the design doc) already
  do this implicitly via independent timers per knob; the
  bench validates the choice.
- **Bitrate is the cheap knob** — change it as often as the
  encoder will tolerate, regardless of MCS/FEC state.

## Reproducing

The sweep scripts and pulled bundles live under
`debug-bitrate-sweep/` (gitignored by default — local
analysis directory). To re-run any of the six configurations:

1. SSH idiom in the auto-memory `ssh_access_drone_gs.md`.
2. Set `enabled: false` in `/etc/dynamic-link/gs.yaml` on GS.
3. Restart both services (GS service + drone applier).
4. Drop `bitrate_sweep.sh` (bitrate-only) or `ladder_sweep.sh`
   (per-step `(bitrate, mcs, k, n)`) onto the drone at `/tmp/`.
5. Run `/tmp/<script>.sh 60 > /tmp/sweep.log 2>&1`.
6. `scp` the matching `flight-NNNN/` plus `applier.log` to a
   workstation.
7. `python3 -m gs.tools.dl_report --bundle <dir> --output <name> --format markdown`.

Cadence is set by the script's inner `sleep` value
(`sleep 1` = 1 Hz, `sleep 0.1` = 10 Hz; busybox `sleep`
accepts decimals on this image).
