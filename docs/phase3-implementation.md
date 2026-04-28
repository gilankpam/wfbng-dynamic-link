# Phase 3 implementation — debug suite

Companion to `docs/dynamic-link-design.md`. Operator how-to lives
at `docs/debugging.md`; this doc is the design rationale and JSON
schemas — read it when you need to *change* the suite, not when
you need to *use* it.

## Why this exists

The pilot reports a recurring bug class: **goggle-perceived
latency that the GS log says is "fine."** Investigation revealed
five concrete gaps:

1. **No latency in the GS log.** The 10 Hz event/verbose logs
   carry RSSI, SNR, FEC counts, MCS — but never a latency number,
   so a moment of high latency in the goggles cannot be cross-
   referenced against any controller-side metric.
2. **No common timeline.** Drone and GS use independent monotonic
   clocks. Even if both sides had timestamps in their logs, joining
   "the goggles felt laggy at T+47s" to "the GS commanded a depth
   change at T+47.05s" was guesswork.
3. **The H.264 recording lies.** wfb-ng's recorder concealed/
   skipped frames during loss; the recording reflects the decoded
   stream, not what the goggles displayed at the time. Latency
   measurement from the recording is also impossible because PTS
   comes from the encoder, not from receive time.
4. **Encoder failures are silent.** The drone applier did
   `http_get(...)` and discarded the response. If the encoder
   accepted a `bitrate=` knob over HTTP but refused to apply it
   (HTTP 500, "queue overflow"), the controller never knew. Looked
   identical to a successful apply.
5. **No drone-side debug log.** Drone logs to stderr/syslog; both
   are wiped on reboot, and drone-side stderr isn't visible
   without an SSH session that's unlikely to survive the failure
   you're trying to debug.

The Phase 3 suite addresses all five — Pillar A handles (1) and
(2), Pillar B handles (3), Pillar C handles (4) and (5).

## Architecture

```
                              ┌────────────────────────┐
                              │ GS (Python, asyncio)   │
                              │                        │
   wfb-ng tunnel (5800/5801)  │  return_link (5 Hz     │   /var/log/dynamic-link/
   <───────────────PING ─────────  ping task)          │     gs.jsonl
   ─────────────  PONG ──────►│  tunnel_listener       │     gs.verbose.jsonl
                              │  timesync (Cristian +  │     latency.jsonl       (Pillar B.1)
                              │  EWMA + outlier reject)│     video_rtp.jsonl     (Pillar B.2)
                              │                        │
   wfb-ng video (RTP/H.265)   │  video_tap (passive    │
   ─────────────  port 5600 ─►│  RTP parser, marker-   │
                              │  bit triggered frames) │
                              │                        │
                              │  LogSink stamps every  │
                              │  decision line with    │
                              │  current offset_us     │
                              └────────────────────────┘

   ┌────────────────────────────┐
   │ Drone (C, single-threaded) │
   │                            │
   │  dl_applier recv path:     │     /sdcard/dl-events/
   │    peek magic →            │       dl-events.jsonl     (Pillar C)
   │      DECISION → backends   │       dl-events.jsonl.1
   │      PING     → PONG send  │       dl-events.jsonl.2
   │                            │
   │  failure call sites:       │
   │    backends → dl_dbg       │
   │    applier  → dl_dbg       │
   └────────────────────────────┘
```

## Feature-flag table

Two-level: a master switch and per-feature overrides. Production
deploys leave both masters off and pay zero cost.

### GS — `conf/gs.yaml` `debug:` block

| Key | Type | Default | Effect |
|---|---|---|---|
| `enabled` | bool | `false` | Master switch |
| `ping_pong` | bool/null | follow master | Run the 5 Hz ping task + tunnel listener |
| `latency_log` | bool/null | follow master | Write `latency.jsonl` (auto-disabled if `ping_pong=false`) |
| `video_tap` | bool/null | follow master | Bind RTP tap on `video_tap_port` |
| `video_tap_port` | int | 5600 | wfb-ng's video output port |

### Drone — `conf/drone.conf`

| Key | Type | Default | Effect |
|---|---|---|---|
| `debug_enable` | bool | 0 | Master switch (open gs_tunnel socket; init dl_dbg) |
| `dbg_log_enable` | tristate | -1 | -1=follow master, 0=off, 1=on |
| `gs_tunnel_addr` | string | 10.5.0.1 | Where to send PONGs |
| `gs_tunnel_port` | int | 5801 | Where to send PONGs |
| `dbg_log_dir` | string | /sdcard/dl-events | SD mount dir for failure log |
| `dbg_max_bytes` | int | 33554432 | Per-file cap before rotate |
| `dbg_fsync_each` | bool | 0 | Force durability on each write (slow) |

## Pillar A — drone↔GS clock offset

### Wire formats

`dl_wire.{c,h}` defines two new magics; `gs/dynamic_link/wire.py`
mirrors byte-for-byte. The contract is anchored by extending
`dl-inject` with `--ping --gs-seq N --gs-mono US` and adding hex-
diff fixtures to `tests/test_wire_contract.py`. Same wire-format-
authority pattern as the existing decision packet.

```
DL_PING (GS→drone, 24 bytes on-wire):
  off  size  field
   0    4    magic       = 0x444C5047 ("DLPG")
   4    1    version     = 1
   5    1    flags
   6    2    _pad
   8    4    gs_seq
  12    8    gs_mono_us
  20    4    crc32(bytes[0..19])

DL_PONG (drone→GS, 40 bytes on-wire):
   0    4    magic              = 0x444C504E ("DLPN")
   4    1    version            = 1
   5    1    flags
   6    2    _pad
   8    4    gs_seq
  12    8    gs_mono_us_echo
  20    8    drone_mono_recv_us
  28    8    drone_mono_send_us
  36    4    crc32(bytes[0..35])
```

### Cristian + EWMA

Standard NTP-style four-timestamp exchange. T1 is set on the GS
just before `sendto`; T2 on the drone immediately after
`recvfrom` (before parsing — keeps parser jitter out of the
estimate); T3 on the drone just before `send`; T4 on the GS
inside `datagram_received` before any parsing.

```
RTT      = (T4 − T1) − (T3 − T2)
offset   = ((T2 − T1) + (T3 − T4)) / 2
```

EWMA with α=0.2. Outliers — samples whose RTT exceeds 3× the
recent median (16-sample window) — are not folded into the
estimate but are still emitted to `latency.jsonl` with
`"outlier": true`.

The estimator's stddev tracks via a Welford-ish recursive
formula on the same EWMA cadence. Reported alongside
`offset_us` on every log line and every latency record. Wide
stddev = link is congested (or never converged) — itself a
useful signal that the offset shouldn't be trusted right now.

### Schema — `latency.jsonl`

```jsonc
{
  "ts_gs_mono_us": 12345678,        // GS-side recv time of the PONG
  "ts_gs_wall_us": 1714312345000000, // wall-clock at recv (joinable to PR/issues)
  "gs_seq": 42,                      // sequence echoed back from PING
  "rtt_us": 4500,                    // round-trip
  "drone_mono_recv_us": 50000,       // T2
  "drone_mono_send_us": 50050,       // T3
  "offset_us": 12289123,             // smoothed offset estimate
  "offset_stddev_us": 150,           // confidence
  "outlier": false                   // true → not folded into the smoothed offset
}
```

## Pillar B — latency telemetry

### B.1 Control-plane RTT

A free side-effect of Pillar A. Every PONG produces one
`latency.jsonl` line. Sample rate matches the ping cadence
(currently 5 Hz; raise via the pinger task interval if needed).

### B.2 RTP/H.265 video tap

The wfb-ng video output is standard RTP per the reference
GStreamer pipeline (`udpsrc port=5600 ! caps='application/x-rtp,
encoding-name=H265,clock-rate=90000' ! rtph265depay ! ...`).
Every video UDP packet carries a 12-byte fixed header:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|V=2|P|X|  CC   |M|     PT      |       sequence number         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                           timestamp                           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|           synchronization source (SSRC) identifier            |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

We bind a passive listener with `SO_REUSEPORT` to share port 5600
with whatever already consumes the stream (recorder, goggle
pipeline). On each packet:

1. Stamp arrival time before parsing.
2. Parse seq/ts/marker/SSRC.
3. On SSRC change → reset reference and drop in-progress frame.
4. Accumulate per-frame packet count.
5. On marker bit (end of frame), emit a `FrameRecord`:
   - `latency_drift_us = (gs_mono_now − gs_mono_0) − (rtp_ts_now − rtp_ts_0) × 1e6 / 90000`
   - Constant under steady link conditions.
   - Captures the *change* in encoder-pipeline-plus-network
     latency over time. Cannot measure absolute baseline.

#### `SO_REUSEPORT` caveat

Linux fans packets across all bound sockets round-robin. If the
operator's recorder also binds 5600 *without* `SO_REUSEPORT`, the
tap will silently get half the packets. Per-frame metrics
degrade gracefully: frame-rate halves, marker-bit detection still
works, drift is computed from arrived packets only. Operators
who need every packet should set `debug.video_tap_port` to a
dedicated port and use a UDP fanout in the wfb-ng pipeline.

A future v2 may add a proxy mode where the tap forwards to the
consumer; not implemented in v1 because the SO_REUSEPORT path
covers the common debug case.

#### Schema — `video_rtp.jsonl`

```jsonc
{
  "ts_gs_mono_us": 99950000,
  "ts_gs_wall_us": 1714312341000000,
  "rtp_seq_first": 4096,
  "rtp_seq_last": 4108,
  "rtp_ts": 90000123,
  "ssrc": "0xa1b2c3d4",     // hex string for human readability
  "packets": 13,             // packets actually received
  "expected": 13,            // (last_seq - first_seq) & 0xFFFF + 1
  "lost_in_frame": 0,        // expected - packets, clamped ≥ 0
  "latency_drift_us": 1240,
  "frame_interarrival_us": 16670
}
```

### Why no OSD-watermark / OCR pipeline?

It was on the original plan as Pillar B.2 v2 — drone writes a
monotonic timestamp into the OSD via msposd, GS-side post-flight
OCR extracts it from the recorded H.264. Deferred because:

1. RTP-derived `latency_drift_us` answers the user's stated pain
   (debugging spikes) without OCR.
2. The OCR path needs OpenCV / template matching which would be
   a heavy v1 dependency.
3. Absolute baseline latency was never the user's question.

The hook is in place: `dl_osd_write_*` is the obvious place to
add a `dl_osd_write_watermark(o, mono_ms)` function later if
absolute glass-to-glass latency ever matters.

## Pillar C — drone failure log

### Storage

The drone never writes to RAM-disk for debug. Failures land on
SD card (the VTX has a slot). This trades durability for the only
viable persistence option — tmpfs is tiny and wiped on reboot,
exactly when an interesting failure is most likely.

`dl_dbg.{c,h}` is a singleton, mirroring `dl_log` (also a
singleton). Threading model is single-threaded (the applier
runs one event loop). The singleton avoids plumbing a `dl_dbg_t *`
through every backend's open/apply call.

### Fail-soft

If `dbg_log_dir` isn't mounted or isn't writable at startup, dl_dbg
logs a single warning and the rest of the API turns into no-ops.
The applier never blocks the link to log a debug event. This
matters: the SD card may be missing on a recovery flight.

### Sync policy

Line-buffered (no per-event `fsync`). On consumer SD cards `fsync`
costs 10–100 ms per call which would back-pressure the applier
loop. Hard crashes lose the last few buffered events; that's
acceptable for a debug log. `dbg_fsync_each = 1` is provided for
the rare case where durability matters more than throughput.

### Rotation

Cap at `dbg_max_bytes` (default 32 MiB), two-deep ring (.jsonl →
.jsonl.1 → .jsonl.2). Failures are sparse so the cap is rarely
reached in normal use; the size cap exists to bound a runaway
flight where some failure recurs every tick.

### Schema — `dl-events.jsonl`

```jsonc
{
  "t": 12345678901,           // drone monotonic microseconds (NOT wall clock)
  "seq": 17,                   // per-boot counter; gaps signal lost events
  "sev": "warn",               // debug | info | warn | error
  "reason": "ENC_RESPONSE_BAD",
  "detail": { "http": 500, "body": "rate-limited" }
}
```

### Hook points

All failure call sites that previously logged to stderr/syslog
now also emit to dl_dbg. The hooks are kept narrow — successful
applies don't produce records; only their broken siblings.

| Reason | Emitter | Detail keys |
|---|---|---|
| `DECODE_BAD` | `dl_applier.c` wire decode failed | `code`, `len` |
| `CEILING_REJECT` | `dl_ceiling.c` failsafe 4 | `reason`, `seq` |
| `APPLY_FAIL` | any backend returned non-zero | `seq` |
| `WATCHDOG_TRIPPED` | failsafe 1 (GS-link silence) | `timeout_ms` |
| `TX_APPLY_FAIL` | `dl_backend_tx.c` send/recv/short/non-zero rc | `cmd`, `errno`/`rc`/`reply_len` |
| `RADIO_APPLY_FAIL` | `dl_backend_radio.c` posix_spawnp/waitpid/iw exit | `errno`/`exit`, `dBm` |
| `ENC_APPLY_FAIL` | `dl_backend_enc.c` socket/connect/send/resolve | `errno`, `host`, `port` |
| `ENC_RESPONSE_BAD` | `dl_backend_enc.c` non-2xx HTTP | `http`, `body` (first 64 B) |

`ENC_RESPONSE_BAD` is the closure of the user's killer pain. The
encoder backend `http_get` previously read the response body and
discarded it. It now parses the status line, finds the body after
`\r\n\r\n`, captures the first 64 bytes for non-2xx responses,
and emits an SD-log record. The function still returns 0 on a
completed HTTP exchange (legacy contract preserved) so the
applier doesn't pipe every weird response through MAVLink
`apply_fail`.

## Dev-workstation tools

All in `gs/tools/`. Plain Python scripts; no install required.

### `dl-replay`

Reads a captured `gs.verbose.jsonl`, reconstructs each tick's
Decision via the existing `wire.encode`, and `sendto`s a target
host:port at the *recorded* inter-tick cadence (sleeping by
`timestamp` deltas). Tolerant of extra fields on records (the
LogSink may have stamped `offset_us` etc on top of the
dataclass).

```
python -m gs.tools.dl_replay \
    --source gs.verbose.jsonl \
    --target 10.5.0.2:5800 \
    [--speed 1.0] [--from-ts 100.0] [--until-ts 200.0]
```

### `dl-events-diff`

Diffs two SD failure logs, aligned by `seq`. Ignores `t`
(run-relative) and any timing-derived state. A clean replay
produces zero diffs and exit code 0.

Where both sides have a `body` field and they differ, the diff
highlights it — that's the "encoder responded differently to the
same input" case, which is one of the exact things you'd run a
bench replay to find.

### `dl-bundle` (shell)

GS-side only. Tars the four log files into a timestamped
`flight-<UTC ISO>.tar.gz`. Drone SD log is supplied separately
to `dl-review`.

### `dl-review`

Loads a bundle (tarball or directory) and an optional drone-side
SD log. Joins drone events to GS timeline by binary-searching
the `latency.jsonl` offset samples. Prints one chronological
stream:

```
   99.950000 [video     ] frame seq=… drift=…
  100.000000 [gs        ] mcs=5 …
  100.045000 [drone     ] [warn] ENC_RESPONSE_BAD {…}
```

Filters: `--around TS --window SEC`, `--reasons LIST`,
`--sources LIST`, `--limit N`. CLI-only; no TUI (solo dev tool;
`jq` is the fallback for ad-hoc queries).

## Testing

Test counts at the moment of v1 landing:

- 227 Python tests:
  - 8 debug_config (flag resolution + coherence checks)
  - 6 timesync (Cristian + EWMA + outlier rejection convergence)
  - 3 tunnel_listener (UDP dispatch, malformed reject)
  - 1 latency_sink (JSONL writer)
  - 12 video_tap (RTP parsing, marker-bit, SSRC reset, drift,
        seq-wrap, end-to-end UDP)
  - 3 dl_replay (decode + send, time filters, bad-record skip)
  - 5 dl_events_diff (clean / extra / mismatch / body / ignore-t)
  - 6 dl_review (timeline merge, around-window, source filter,
        no-offset fallback, tarball, dl-bundle smoke)
  - 1 phase3_e2e (real applier + real dl-replay + real mock
        encoder returning HTTP 500 → SD log contains
        ENC_RESPONSE_BAD with body)
  - …plus existing pre-Phase-3 suites unchanged.

- 58 C tests:
  - 7 wire (5 new for ping/pong + peek + bad-CRC)
  - 5 config (4 new for debug-flag tristate resolution)
  - 7 dbg (master-off no-op, JSONL emit, seq counter, force-off,
        body escape, rotation, huge-detail truncate)
  - …plus existing pre-Phase-3 suites unchanged.

Run with:

```bash
python3 -m pytest        # all GS tests
make -C drone test        # all C tests
```

The Phase-3 e2e test (`tests/test_phase3_e2e.py`) is the closure
of the user's stated workflow: it spawns a real applier with a
real mock encoder returning HTTP 500, drives it via a real
dl-replay, and asserts the drone SD log contains the expected
`ENC_RESPONSE_BAD` record with the actual body bytes intact.

## Things deliberately left out

Decisions to revisit only if the user reports new pain that
demands them:

- **OSD-watermark + OCR pipeline.** Absolute encode→GS latency.
  Deferred (see Pillar B note).
- **TUI for `dl-review`.** Solo developer tool; `jq | less` is
  the fallback.
- **Per-flight bundle auto-rotation.** No log-rotation hook in
  the GS service; operators run `dl-bundle` by hand or via cron.
- **GS-side capture of drone events over the link.** All drone
  events go to SD instead of streaming back; survives link-down
  failures (which are exactly the interesting class).
- **Watermark in OSD for absolute latency baseline.** The hook
  is in `dl_osd.c` waiting if needed.
- **Sub-100-ms watermark resolution.** msposd write rate caps
  this anyway; not relevant unless paired with the OCR pipeline.
