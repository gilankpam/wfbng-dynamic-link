# dynamic-link

Adaptive link controller for wfb-ng — separate repo, pure consumer of
wfb-ng's already-stable interfaces. See `docs/dynamic-link-design.md` for
the full design. Implementation notes per phase live at
`docs/phase0-implementation.md` (GS observer), `docs/phase2-implementation.md`
(end-to-end wiring), `docs/phase3-implementation.md` (post-flight debug suite).

Two components:

- **GS controller** (`gs/dynamic_link/`, Python) — subscribes to
  wfb-ng's JSON stats feed, runs the §4 control loops, optionally
  sends decisions to the drone.
- **Drone applier** (`drone/src/`, C) — receives decision packets,
  validates against a per-airframe ceiling, dispatches to wfb_tx /
  `iw` / encoder HTTP. Emits STATUSTEXT back to the GS on rejections
  and watchdog trips.

## Status

| Phase | Scope | State |
|---|---|---|
| 0 | GS observer (log-only) | ✅ |
| 1 | Drone `dl-applier` + `dl-inject` CLI | ✅ |
| 2 | GS → drone wire + drone → GS MAVLink status + oscillation detector | ✅ |
| 3 | Post-flight debug suite (timesync, latency log, RTP video tap, drone SD failure log, dev tools) | ✅ |
| 4 | Airframe tuning + flight validation | pending |

Through Phase 2 the end-to-end path works: flip `enabled: true` in
`gs.yaml` and the GS starts shipping decisions to the drone over
wfb-ng's tunnel stream. With `enabled: false` the GS stays in Phase 0
observer mode.

## Operational prerequisites (wfb-ng `master.cfg`)

Two one-time edits on the wfb-ng install — default values won't work
for us:

1. **GS host** — `[common] log_interval = 100`
   (default is 1000; we need 10 Hz stats cadence per §3).
2. **Drone host** — `control_port = 8000` in the `[<tx-section>]`
   block for the video stream (default 0 picks a random port our
   applier can't target).

Both are in `wfb-ng/wfb_ng/conf/master.cfg`. Restart wfb-ng after
editing.

## GS install and run

```bash
pip install -e '.[dev]'
```

Phase 2 adds a runtime dep on wfb-ng's bundled MAVLink codec. If
wfb-ng is already installed on the GS (standard for anyone flying),
this is free. For dev on a workstation without wfb-ng packaged:

```bash
VERSION=1.0.0 COMMIT=abc1234 pip install -e /path/to/wfb-ng
```

(wfb-ng's `setup.py` requires both env vars; any PEP-440-ish version
string works.)

Run the controller:

```bash
dynamic-link-gs --config /etc/dynamic-link/gs.yaml --log-dir /var/log/dynamic-link/flights
```

The service auto-rotates per flight: each time the drone reconnects
(non-empty `rx_ant_stats` after a quiet period) it opens a fresh
`flight-NNNN/` subdirectory holding `gs.jsonl`, `gs.verbose.jsonl`,
`latency.jsonl`, `video_rtp.jsonl`, and a `flight.json` manifest. After
`debug.flight_rotation.gap_seconds` of empty rx (default 10 s — drone
power-off / battery swap), the directory is closed and writes pause
until the next reconnect. `gs/tools/dl_report.py` consumes one such
directory directly. Omit `--log-dir` to mirror decisions to stdout
instead (useful for `--replay` smoke tests).

Observer mode (Phase 0 behaviour — the default with `enabled: false`):
decisions are computed and logged but never sent. Tail the latest
`flight-*/gs.jsonl` to watch the policy engine work.

End-to-end mode (flip `enabled: true` in gs.yaml): every decision is
also serialised onto the wfb-ng tunnel stream and the drone applies it.

Replay a captured JSON stream offline:

```bash
nc 127.0.0.1 8103 > capture.jsonl     # capture once from a flying wfb-ng
dynamic-link-gs --config conf/gs.yaml.sample --replay capture.jsonl --log-dir /tmp/dl-flights
```

## Drone install and run

Build natively:

```bash
make -C drone
# → drone/build/dl-applier, drone/build/dl-inject
```

Cross-compile for ARM / OpenIPC:

```bash
make -C drone CROSS_COMPILE=arm-linux-gnueabihf-
```

Install + enable systemd unit:

```bash
sudo make -C drone install PREFIX=/usr/local
sudo cp conf/drone.conf.sample /etc/dynamic-link/drone.conf
sudo cp packaging/dynamic-link-applier.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dynamic-link-applier
```

OpenRC equivalent:

```sh
sudo cp packaging/dynamic-link-applier.init /etc/init.d/dynamic-link-applier
sudo rc-update add dynamic-link-applier default
sudo /etc/init.d/dynamic-link-applier start
```

Drive the applier by hand (Phase 1 bringup, or to test individual
backends without the GS):

```bash
dl-inject --target 10.5.0.2:5800 \
          --mcs 5 --bandwidth 20 --tx-power 18 \
          --k 8 --n 14 --depth 2 \
          --bitrate 12000 --fps 60
```

## Return-link path

GS sends decisions to the drone's wfb-ng TUN IP (`10.5.0.2:5800` by
default — mirrors `drone.conf:listen_port`). Nothing tunnel-specific
in the code: `sendto()` on one side, `recvfrom()` on the other; the
kernel routes via wfb-ng's `gs-wfb` / `drone-wfb` TUN interfaces.
Details in `docs/phase2-implementation.md`.

## Debugging

Phase 3 adds a post-flight debug suite — clock-aligned cross-side
timeline, control-plane RTT log, per-frame RTP/H.265 latency-drift
metrics, and an SD-card failure log on the drone. Production
deploys leave it off; flip `debug.enabled: true` in `gs.yaml` and
`debug_enable = 1` in `drone.conf` for a debug flight. Operator
how-to lives at [`docs/debugging.md`](docs/debugging.md).

## Status channel (drone → GS)

Piggybacks on wfb-ng's existing `mavlink` stream. The drone applier
emits MAVLink v1 `STATUSTEXT` frames (msgid 253) with a `DL ` prefix
on three events:

- `DL REJECT <reason>` — failsafe-4 ceiling rejections.
- `DL WATCHDOG safe_defaults` — failsafe-1 GS-link silence.
- `DL APPLY_FAIL backend` — one or more backends returned non-zero.

Rate-limited to ≤ 2 msg/s per reason. The GS's `mavlink_status.py`
reader parses them and surfaces at INFO level in the service log;
they also show up in QGroundControl's "Status Text" panel alongside
flight-controller telemetry.

## Port-naming caveat

Design doc §6 refers to the stats endpoint as `stats_port`. In
wfb-ng's `master.cfg` that name is the **msgpack** feed wfb-cli uses
(8002/8003); the **JSON** feed dynamic-link subscribes to is bound
to `api_port` (8102/8103). The sample config points at 8103 by
default via the neutral endpoint-URL form.

## Tests

```bash
pytest                     # 83+ tests: unit + end-to-end
make -C drone test         # 37 C unit tests
```

The Python e2e suite spawns the real `dl-applier` binary against a
mock `wfb_tx` UDP server, a mock encoder HTTP server, and a mock
MAVLink sink — no live radio required.

## Layout

```
gs/dynamic_link/         Python package
  service.py             entry point
  stats_client.py        async JSON stats-API client
  signals.py             per-window aggregator + EWMA
  profile.py             radio-profile loader
  predictor.py           latency-budget predictor
  policy.py              leading + trailing control loops
  decision.py            Decision dataclass
  sinks.py               LogSink
  wire.py                Phase 2: decision-packet encoder (mirrors dl_wire.c)
  return_link.py         Phase 2: UDP writer to drone TUN IP
  oscillation.py         Phase 2: failsafe 2 (GS-side)
  mavlink_status.py      Phase 2: drone → GS STATUSTEXT reader
  debug_config.py        Phase 3: feature-flag resolver
  timesync.py            Phase 3: Cristian + EWMA offset estimator
  tunnel_listener.py     Phase 3: async UDP listener for PONGs
  latency_sink.py        Phase 3: latency.jsonl writer
  video_tap.py           Phase 3: passive RTP/H.265 video tap

gs/tools/                Dev-workstation post-flight tools (Phase 3)
  dl_replay.py           Replay GS verbose.jsonl to a bench drone
  dl_events_diff.py      Diff two SD failure logs
  dl_review.py           Unified-timeline viewer (CLI)
  dl_report.py           Self-contained HTML flight report (Plotly)
  dl-bundle              Tar GS-side logs into a flight bundle

drone/
  Makefile               Native + CROSS_COMPILE build
  src/
    dl_applier.c         Main event loop
    dl_inject.c          CLI — craft+send one decision (also `--dry-run`)
    dl_wire.{c,h}        Wire format (authority for both languages)
    dl_config.{c,h}      drone.conf parser
    dl_watchdog.{c,h}    Failsafe 1 (GS-link timeout)
    dl_backend_tx.{c,h}  tx_cmd.h dispatch (FEC, RADIO, DEPTH)
    dl_backend_radio.{c,h}  iw shell-out (TX power)
    dl_backend_enc.{c,h}    Raw HTTP GET to majestic/waybeam
    dl_osd.{c,h}         /tmp/MSPOSD.msg writer (§4B)
    dl_mavlink.{c,h}     Phase 2: MAVLink v1 STATUSTEXT emitter
    dl_dbg.{c,h}         Phase 3: SD-card JSONL failure log
    dl_log.{c,h}         Tiny level-based logger (syslog + stderr)
    vendored/
      tx_cmd.h           Pinned copy of wfb-ng's control protocol
      README.md          Source SHA + refresh procedure

conf/
  gs.yaml.sample         Sample GS config
  drone.conf.sample      Sample drone config
  radios/m8812eu2.yaml   Default radio profile (BL-M8812EU2)

packaging/
  dynamic-link-gs.service         GS systemd unit
  dynamic-link-applier.service    Drone systemd unit
  dynamic-link-applier.init       Drone OpenRC init

tests/                   pytest suite (unit + e2e)
tests/drone/             C unit tests (tests/drone/*.c, run via `make -C drone test`)
docs/                    Design doc + per-phase implementation notes
```
