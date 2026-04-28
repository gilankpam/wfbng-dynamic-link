# Debugging dynamic-link

Operator-facing how-to for the post-flight debug suite (Phase 3).
The suite addresses one specific class of bug: **goggle-perceived
latency that the regular controller log says is "fine."** It does
this by giving you four streams of forensic data on a single
unified timeline.

If you're new to the design rationale, read
`docs/phase3-implementation.md` after this — it explains *why*
the suite is structured the way it is.

## What you get

| Stream | Source | Tells you |
|---|---|---|
| `gs.jsonl` + `gs.verbose.jsonl` | GS (already there pre-Phase-3) | What the controller decided, when, why |
| `latency.jsonl` | GS (Phase 3) | Control-plane RTT every 200 ms |
| `video_rtp.jsonl` | GS (Phase 3) | Per-frame video latency drift, jitter, packet loss |
| `dl-events.jsonl` (on drone SD card) | Drone (Phase 3) | Every backend failure with full detail (HTTP body, errno, exit code) |

The four streams are joined at review time on a single timeline
using a continuously-sampled drone↔GS clock offset (the PING/PONG
loop). You don't have to sync wall clocks; the offset estimator
handles it.

## TL;DR — running a debug flight

1. **One-time:** edit `gs.yaml` and `drone.conf` to enable debug.
   See [Configuration](#configuration).
2. **Before the flight:** clear the SD card's `dl-events/`
   directory (or accept that old files will be there).
3. **Fly.**
4. **After the flight:** pull the SD card; run `dl-bundle` on the
   GS; run `dl-review` on your dev workstation against the bundle
   plus the SD log.
5. **If you want to repro at the bench:** run `dl-replay` on a
   bench drone using the GS's `gs.verbose.jsonl`, then
   `dl-events-diff` to compare the bench SD log against the flight
   SD log.

## Configuration

### GS — `conf/gs.yaml`

```yaml
debug:
  enabled: false              # master switch (default OFF)
  ping_pong: null             # null=follow master; true/false to override
  latency_log: null           # latency.jsonl (requires ping_pong)
  video_tap: null             # RTP/H.265 frame-level tap
  video_tap_port: 5600        # wfb-ng's video output port
```

Per-feature flags accept `null` (follow `enabled`), `true`, or
`false`. To turn the whole suite on for a debug flight, set
`enabled: true`. To run a specific subset (e.g. just video tap,
no PING/PONG): `enabled: false`, `video_tap: true`.

### Drone — `conf/drone.conf`

```
debug_enable     = 0          # master (default OFF)
dbg_log_enable   = -1         # tristate: -1=follow master, 0=off, 1=on

# GS-side endpoint for drone→GS tunnel traffic (PONGs).
gs_tunnel_addr   = 10.5.0.1
gs_tunnel_port   = 5801

# SD-card failure log.
dbg_log_dir      = /sdcard/dl-events
dbg_max_bytes    = 33554432    # 32 MiB cap; rotates to .1, .2
dbg_fsync_each   = 0           # 1 = fsync each record (slow, durable)
```

Set `debug_enable = 1` for a debug flight. The drone never writes
anything beyond `dl-events.jsonl` on the SD card; if the SD isn't
mounted, dl_dbg becomes a no-op and the applier keeps running.

### CLI flags (GS)

The service writes its log files where the existing `--log-file`
and `--verbose-log-file` flags point. Phase 3 adds two more:

```
dynamic-link-gs --config /etc/dynamic-link/gs.yaml \
    --log-file         /var/log/dynamic-link/gs.jsonl \
    --verbose-log-file /var/log/dynamic-link/gs.verbose.jsonl \
    --latency-log-file /var/log/dynamic-link/latency.jsonl \
    --video-rtp-log-file /var/log/dynamic-link/video_rtp.jsonl
```

If `debug.latency_log: true` is set but `--latency-log-file` isn't
supplied, the service warns and skips the latency log. Same for
`video_rtp.jsonl`.

## Post-flight workflow

### 1. Bundle the GS-side files

```bash
gs/tools/dl-bundle
# → flight-20260428T143052Z.tar.gz
```

The default `LOG_DIR` is `/var/log/dynamic-link`; override with
the env var if your logs live elsewhere. Optional positional arg
is the bundle tag (default is the UTC timestamp).

### 2. Pull the drone SD log

Pop the SD card; copy `dl-events/dl-events.jsonl` to your dev
workstation. (If the file rolled, also copy the `.1` and `.2`
siblings — but in practice failures are sparse and you'll only
ever have the active file.)

### 3a. Generate a flight report (recommended starting point)

`dl-report` produces either an interactive HTML file or a Markdown
text file (or both). Pick by output extension or `--format`:

```bash
# Interactive HTML (default)
python -m gs.tools.dl_report --bundle flight.tar.gz -o report.html

# Plain Markdown — agent-friendly, easy to paste into a chat or issue
python -m gs.tools.dl_report --bundle flight.tar.gz -o report.md

# Both at once
python -m gs.tools.dl_report --bundle flight.tar.gz -o report.html \
    --format both
```

The HTML is self-contained (~6 MB inline, ~1 MB with `--cdn`).

Both formats share the same structure:

- **TL;DR — diagnosis** — anomaly classifications grouped by
  category. Answers "why did I see stutter?" up front:

      1× Drone TX silent
      4× Encoder pipeline
      4× Link congestion
      11× Link stress (FEC absorbed)
      6 emergency events in the controller log

  Each verdict comes from a heuristic table mapping (RTT, drift,
  loss, starvation, SSRC change) onto a likely cause. Thresholds
  live in `classify_anomaly` if you need to tune them.

- **Summary** — duration, MCS time-distribution, RSSI/SNR/RTT
  percentiles, video loss, drift range.

- **Anomaly leaderboard** — top 20 1-second windows ranked by a
  combined score (drift excursion × frame interarrival ×
  (lost+1) × RTT factor). Each row carries a verdict + severity
  marker.

- **Top events explained** — for each top anomaly, a "Why" and
  "What you probably felt" block in plain language. This is the
  bit that goes from "here's the data" to "here's what happened."

The HTML report adds:

- **Linked timeline** — six stacked subplots sharing one x-axis
  (MCS/bitrate, RSSI/SNR, loss/fec, RTT, video drift, frame
  interarrival). Zoom or pan in any panel and the others follow.
  Vertical red/orange dashed lines mark emergency events and
  watchdog trips.
- **Distributions** — RTT, interarrival, drift histograms with
  p50/p95/p99 markers.

### 3b. CLI review (when you know the moment and want raw data)

```bash
python -m gs.tools.dl_review \
    --bundle flight-20260428T143052Z.tar.gz \
    --drone-events ./dl-events.jsonl \
    --around 100.0 --window 2.0
```

Output is one line per event from any source, sorted by
GS-monotonic timestamp:

```
   99.950000 [video     ] frame seq=100-102 pkts=3/3 lost=0 drift=+0.2ms
  100.000000 [gs        ] mcs=5 k=8 n=14 d=2 br=12000 changed=mcs reason=boot
  100.000000 [gs.verbose] mcs=5 k=8 n=14 d=2 br=12000 reason=boot
  100.045000 [drone     ] [warn] ENC_RESPONSE_BAD {"http":500,"body":"rate-limited"}
  100.200000 [latency   ] rtt=4.5ms offset=50000000 stddev=100
```

Filter to a specific source or drone reason:

```bash
dl-review --bundle … --sources latency,drone
dl-review --bundle … --reasons ENC_RESPONSE_BAD,WATCHDOG_TRIPPED
```

### 4. Bench-replay a flight (when you want to reproduce)

On the dev workstation, with a bench drone reachable at e.g.
`10.5.0.2:5800`:

```bash
python -m gs.tools.dl_replay \
    --source ./flight/gs.verbose.jsonl \
    --target 10.5.0.2:5800
```

The bench drone applier will receive identical decision packets
at the recorded cadence. Configure the bench drone with
`debug_enable = 1` and a writable `dbg_log_dir` so it produces
its own `dl-events.jsonl`. Then diff:

```bash
python -m gs.tools.dl_events_diff \
    flight/dl-events.jsonl \
    bench/dl-events.jsonl
```

A clean replay produces no output and exit code 0. Behavioural
divergence (e.g. encoder responded differently to the same
request, watchdog tripped on one side but not the other) shows up
as `+ seq=… only in B` or `~ seq=…: A=…/… vs B=…/…` lines.

## Reading the streams

### `latency.jsonl` — control-plane RTT

```jsonl
{"ts_gs_mono_us":12345678,"ts_gs_wall_us":1714312345000000,"gs_seq":42,
 "rtt_us":4500,"drone_mono_recv_us":50000,"drone_mono_send_us":50050,
 "offset_us":12289123,"offset_stddev_us":150,"outlier":false}
```

- `rtt_us` — full round-trip on the link, drone-applier scheduling
  included.
- `offset_us` — current estimate of `(gs_mono − drone_mono)`.
- `offset_stddev_us` — confidence in the estimate. Widens under
  congestion; itself a useful debug signal.
- `outlier: true` — sample's RTT was > 3× the recent median; the
  smoothed offset wasn't moved. Look for runs of these as a
  congestion indicator.

### `video_rtp.jsonl` — per-frame metrics

```jsonl
{"ts_gs_mono_us":99950000,"ts_gs_wall_us":1714312341000000,
 "rtp_seq_first":4096,"rtp_seq_last":4108,"rtp_ts":90000123,
 "ssrc":"0xa1b2c3d4","packets":13,"expected":13,"lost_in_frame":0,
 "latency_drift_us":1240,"frame_interarrival_us":16670}
```

- `latency_drift_us` — **the headline metric.** Difference between
  GS-arrival-elapsed and encoder-elapsed since the reference
  frame. Constant under steady link conditions; a 200 ms goggle
  freeze shows up as a 200 ms jump over a few frames. Resets to
  zero when SSRC changes (encoder restart).
- `lost_in_frame` — packets missing within this frame's seq range.
- `frame_interarrival_us` — gap to the previous frame's arrival.
  Should be ~16667 µs at 60 fps; spikes correlate with link
  events.

### `dl-events.jsonl` — drone failure log

```jsonl
{"t":12345678,"seq":17,"sev":"warn","reason":"ENC_RESPONSE_BAD",
 "detail":{"http":500,"body":"rate limited"}}
```

- `t` — drone monotonic microseconds (NOT a wall clock). Joined
  to GS timeline post-flight via `latency.jsonl` offsets.
- `seq` — per-boot counter. Gaps mean events were lost (SD stall,
  applier crash mid-flight); dl-review surfaces these.

Reasons emitted today:

| Reason | Where | Detail fields |
|---|---|---|
| `DECODE_BAD` | wire decode failed | `code`, `len` |
| `CEILING_REJECT` | failsafe 4 tripped | `reason`, `seq` |
| `APPLY_FAIL` | one or more backends returned non-zero | `seq` |
| `WATCHDOG_TRIPPED` | failsafe 1 (GS-link silence) | `timeout_ms` |
| `TX_APPLY_FAIL` | wfb_tx control socket error | `cmd`, `errno`/`rc`/`reply_len` |
| `RADIO_APPLY_FAIL` | `iw` exit non-zero | `errno`/`exit`, `dBm` |
| `ENC_APPLY_FAIL` | encoder connect/send/resolve fail | `errno`, `host`, `port` |
| `ENC_RESPONSE_BAD` | encoder returned non-2xx HTTP | `http` (status), `body` (first 64 B) |

`ENC_RESPONSE_BAD` is the killer one — closes the "encoder
accepted the knob silently but didn't apply it" pain. Without it,
silent encoder failures look identical to successes in the GS
log.

## Troubleshooting

### "offset never converges"

Symptoms: `latency.jsonl` has many `outlier: true` samples; the
smoothed offset jumps around.

- Confirm the drone has tunnel reachability *both ways*. Test with
  `ping 10.5.0.1` *from the drone*. PINGs going GS→drone aren't
  enough; we need PONGs back.
- Confirm `drone.conf:gs_tunnel_addr` matches the GS's tunnel
  endpoint (`10.5.0.1` by default per wfb-ng's tunnel profile).
- Check the GS log at startup: should see
  `tunnel_listener: bound 0.0.0.0:5801`. If you see
  `bind … failed`, another process owns the port.

### "video_tap port already in use"

Linux's `SO_REUSEPORT` *should* let the tap and the recorder
share port 5600. If your recorder doesn't set `SO_REUSEPORT`,
the tap will silently lose half the packets to round-robin
fan-out. Two options:

1. Set `debug.video_tap_port` to a different port, and configure
   your video stream to send to *both* ports (use a UDP-fanout
   helper in the wfb-ng pipeline). More setup but reliable.
2. Accept halved sampling density — per-frame metrics still work
   at 30 Hz instead of 60 Hz. Adequate for spotting latency
   spikes.

### "dl-events.jsonl is empty / missing"

- Check the drone log at startup. If you see
  `dbg: failure log → /sdcard/dl-events/dl-events.jsonl`, dl_dbg
  is enabled and ready.
- If you see `dbg: open …: No such file or directory`, the SD
  isn't mounted at `dbg_log_dir`. Fix the mount or change the
  config.
- Empty file with `ls -l` showing nonzero size means logs are
  buffered. Either the applier is still running (graceful exit
  flushes), or set `dbg_fsync_each = 1` in drone.conf. The latter
  costs throughput on SD.
- Truly empty after a clean shutdown: there were no failures.
  That's a good thing.

### "drone events appear at weird timestamps in dl-review"

If the events show `(no offset; raw drone-mono)` in the
`summary`, ping/pong wasn't running on this flight, so dl-review
fell back to plotting drone events at their raw drone-mono
timestamps. They'll be ordered correctly relative to *each other*
but won't line up with GS-side events. Re-fly with
`debug.ping_pong: true` to fix this for next time.

### "the controller's verbose log doesn't have offset_us in the lines"

`offset_us` only gets stamped onto a log line *after* the first
PONG arrives and the smoothed estimate is non-empty. If you see
plain log lines for the first few seconds and then offset-stamped
lines after that, that's expected. If it never appears, see
"offset never converges" above.

## A worked example

You land after a flight where the goggles felt laggy at 200 m.
Pull a bundle, pull the SD log, and:

```bash
$ dl-review --bundle flight-20260428T143052Z.tar.gz \
    --drone-events ./dl-events.jsonl \
    --around 187.0 --window 0.5
```

```
  186.500000 [video     ] frame seq=4900-4912 pkts=13/13 lost=0 drift=+0.4ms
  186.516000 [video     ] frame seq=4913-4925 pkts=13/13 lost=0 drift=+0.5ms
  186.700000 [latency   ] rtt=4.2ms offset=50000000 stddev=100
  186.770000 [video     ] frame seq=4926-4937 pkts=12/13 lost=1 drift=+12.3ms  ← 
  186.852000 [video     ] frame seq=4938-4949 pkts=10/12 lost=2 drift=+47.1ms  ← 
  186.900000 [latency   ] rtt=89.0ms offset=49998900 stddev=4500 OUTLIER       ← 
  186.940000 [drone     ] [warn] ENC_RESPONSE_BAD {"http":500,"body":"queue overflow"}
  187.000000 [gs        ] mcs=5 k=8 n=18 d=2 br=8000 changed=n,bitrate_kbps reason=loss_step
  187.105000 [video     ] frame seq=4950-4955 pkts=6/6 lost=0 drift=+47.5ms
  187.300000 [latency   ] rtt=4.8ms offset=50000050 stddev=120
```

Reading top-to-bottom: link was clean (drift 0.4 ms) until 186.77.
Then frames started losing packets and drift jumped 12 → 47 ms.
RTT spiked to 89 ms — link congestion. The encoder choked
(`queue overflow` body) and the controller dropped bitrate at
187.0 in response. Drift didn't recover within this window
because the encoder backlog had to drain.

That's the kind of root-cause story the regular GS log alone
cannot tell you.
