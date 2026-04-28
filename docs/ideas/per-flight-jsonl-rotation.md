# Per-flight gs.jsonl rotation

## Goal

Slice `/var/log/dynamic-link/gs.jsonl` into one file per flight for
post-flight analysis. A "flight" is approximately one battery's worth
of activity (drone power-on → power-off). Operator workflow: swap
battery between flights → drone reboots → new flight file should appear
without manual intervention.

Decided to defer; capturing the design conclusions here.

## Options considered

### A. MAVLink HEARTBEAT arm bit (true arm→disarm semantics)

`base_mode & MAV_MODE_FLAG_SAFETY_ARMED` (0x80) flips on arm/disarm.
Verified `wfb_ng.mavlink` exposes `MAVLINK_MSG_ID_HEARTBEAT=0` and
`MAV_MODE_FLAG_SAFETY_ARMED=128`. wfb-ng itself uses the exact pattern
internally (`wfb-ng/wfb_ng/mavlink_protocol.py:155-158`).

Betaflight (verified against `betaflight/src/main/telemetry/mavlink.c:445-447`)
uses `MAV_MODE_MANUAL_ARMED = 0xC0` which happens to include the
`SAFETY_ARMED = 0x80` bit, so `base_mode & 0x80` does the right thing
on Betaflight too.

**Operator prerequisites on the FC:**
- Map MAVLink to a UART (Configurator → Ports, or
  `serial <uartIndex> 4096 115200 57600 0 115200`).
- `set mavlink_extra2_rate = 1` — **defaults to 0 (off)**. Without
  this, no HEARTBEAT is emitted and arm detection silently never
  fires. Most likely failure mode.
- `feature TELEMETRY` (usually already on).

GS-side wfb-ng MAVLink uplink is already plumbed (the dynamic-link
applier already pushes STATUSTEXT through it).

**Pros:** exact arm→disarm boundaries, semantically correct.
**Cons:** ~250 LoC, two-side prerequisite (FC + drone). Easy to forget
the `mavlink_extra2_rate` step and silently produce no events.

### B. wfb-ng session epoch change

`SessionEvent.epoch` field is set on the TX side via the `wfb_tx -e`
argument. Within a wfb_tx run, epoch stays constant; on a fresh
`wfb_tx` start it changes — provided `-e` is set to something
unique-per-boot. Default is `0` (per `wfb-ng/src/tx.cpp:1934`); current
operator launch on 192.168.10.152 is using the default, so epoch is
always 0 in the live JSONL.

To make epoch unique-per-boot the operator can wrap the wfb_tx launch
script with one of:

- `wfb_tx -e $(od -An -N6 -tu8 /dev/urandom | tr -d ' \n') ...`
  — 48-bit random; entropy pool is well-seeded by the time the init
  script launches wfb_tx even on a no-RTC headless box.
- Persistent counter file (e.g. `/usr/local/etc/wfb_epoch`)
  incremented at each launch.

Note: `date +%s` does NOT work on this stack — neither drone nor GS
has an RTC battery; clock starts from a default value at boot, often
identical across reboots.

Important wfb-ng behaviour confirmed in
`wfb-ng/wfb_ng/protocols.py:466-470`: `SessionEvent` fires on every
session-block change, **including each FEC k/n change** during
flight. So we cannot rotate on every SessionEvent — only on `epoch`
transitions specifically. Fortunately k/n changes leave epoch alone.

The session_key (regenerated per wfb_tx run via `randombytes_buf` at
`wfb-ng/src/tx.cpp:242`) would also be a unique-per-boot signal but is
NOT surfaced through the JSON stats stream (`protocols.py:457-460`
exposes only fec_type, fec_k, fec_n, epoch, interleave_depth,
contract_version), so we can't use it.

**Pros:** clean, robust, ~50 LoC GS-side.
**Cons:** requires drone-side wfb_tx launch script edit.

### C. GS-side rx-stream-gap heuristic (recommended for revisit)

Empty `rx_ant_stats` for ≥N seconds → "between flights"; first
non-empty `rx_ant_stats` after the gap → "new flight starts".

Critical mechanism check (verified):
- `rx` events on the wfb-server JSON stats stream **do NOT gap** —
  `wfb-ng/src/rx.cpp:554` emits the `PKT` line every `log_interval`
  unconditionally; wfb-server forwards that as an `rx` event.
- `rx_ant_stats` (the antenna list inside that event) **DOES gap** —
  `rx.cpp:542` only emits RX_ANT lines for antennas that actually
  received fragments this window. Drone off → empty list.

So the signal we watch is "windows with empty `rx_ant_stats`", not
"event stream silence".

**False-positive matrix:**

| Event | Empty rx_ant_stats? | Acceptable? |
|---|---|---|
| Drone powered off | Yes — sustained | yes (this is the true positive) |
| Battery swap | Yes — sustained | yes |
| Antenna covered (partial) | No (other antennas receive) | n/a |
| Antenna covered (all 4) | Yes — transient | tunable via gap threshold |
| In-flight ≥ N s blackout | Yes — sustained | acceptable: total video loss in FPV is functionally end-of-flight |

**Pros:** zero drone-side change. No MAVLink, no FC config, no `-e`
arg. ~30 LoC GS-side.
**Cons:** flight boundary granularity is "drone power on/off", not
"arm/disarm". An aggressive flier with frequent deep fades could see
spurious closes.

**Tuning knob:** `flight_gap_seconds` config, default 10 s. Increase
if real flights dip into fades.

## Sketch (option C)

`gs/dynamic_link/flight_log.py` (new):

```python
class FlightLogRotator:
    def __init__(self, log_dir: Path) -> None: ...
    def current_stream(self) -> TextIO: ...
    def open_flight_file(self, ts: float) -> None: ...
    def close_flight_file(self, ts: float) -> None: ...
    def close(self) -> None: ...
```

`gs/dynamic_link/sinks.py` — `LogSink.__init__` accepts
`rotator: FlightLogRotator | None`; `write` calls
`rotator.current_stream().write(...)` per tick.

`gs/dynamic_link/service.py` — additions:

```python
state = "between_flights"
last_rx_ant_ts: float | None = None
GAP_S = 10.0

def on_event(ev):
    ...
    if isinstance(ev, RxEvent):
        if ev.rx_ant_stats:
            last_rx_ant_ts = ev.timestamp
            if state == "between_flights":
                rotator.open_flight_file(ev.timestamp)
                state = "in_flight"
        elif state == "in_flight" and last_rx_ant_ts is not None:
            if ev.timestamp - last_rx_ant_ts > GAP_S:
                rotator.close_flight_file(ev.timestamp)
                state = "between_flights"
        ...
```

Files:
- `gs.bench.jsonl` (always, when between flights)
- `gs.flight-YYYYMMDD-HHMMSS.jsonl` (one per flight, named by
  GS-clock-at-arm-detection — the GS clock and drone clock disagree
  but the GS clock is what sorts the directory listing).

Tests: standard pattern from `tests/test_signals.py`'s `_rx()` helper
to fabricate RxEvents with/without `rx_ant_stats`. Unit-test
state transitions. New `tests/test_flight_log.py` for the rotator
itself.

## Verification when implemented

1. Pre-flight check: `rx_ant_stats` in JSONL is non-empty during
   normal operation (it is — confirmed in current logs).
2. Power-cycle drone with GS service running. Within `flight_gap_seconds`,
   the flight file should close; new flight file should open within
   ~1 s of drone power-on.
3. No false-close during the antenna-cover test from the Phase
   2/2.5 work (partial cover keeps `rx_ant_stats` non-empty on the
   uncovered antennas).

## Status: implemented (2026-04-28)

Shipped via Option C (rx-stream-gap heuristic). Lives at
`gs/dynamic_link/flight_log.py` (`FlightDirRotator`); tests at
`tests/test_flight_log.py`.

**Divergence from the sketch above:** directory-per-flight rather
than filename-prefixed file-per-flight. Each flight is a self-contained
bundle dir (`flight-NNNN/`) holding all four JSONL streams and a
`flight.json` manifest, matching what `gs/tools/dl_report.py` already
consumes (and what the operator was producing manually as
`debug-flight2/` etc.).

**Naming:** incremental `flight-NNNN/` (4-digit, zero-padded), not
GS-wall-clock — the GS box has no NTP guarantee at boot, and the
counter resumes from disk by scanning existing dirs at startup.

**Idle behavior:** writes are dropped entirely between flights (rather
than spilling to a `bench/` dir). Continuing to log when
`rx_ant_stats` is empty just records ghost RSSI/SNR from the last
received packet — see `signals.py:135–137` — so dropping is the
honest signal.

**CLI:** the four `--log-file` / `--verbose-log-file` /
`--latency-log-file` / `--video-rtp-log-file` flags were removed;
a single `--log-dir DIR` replaces them.
