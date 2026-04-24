# dynamic-link

Adaptive link controller for wfb-ng — separate repo, pure consumer of
wfb-ng's already-stable interfaces. See `docs/dynamic-link-design.md` for the
full design.

## Status: Phase 0 — GS observer (log-only)

Only the GS Python service is shipped. It subscribes to wfb-ng's JSON
stats API, runs both §4 control loops, and writes the decisions it
*would* have emitted to a log file. No commands reach the drone.

Phase 1 (drone C applier) and Phase 2 (GS → drone wire) are not in
this release.

## Prerequisite — wfb-ng config

The controller samples at 10 Hz. Set `log_interval = 100` in the
`[common]` section of wfb-ng's `master.cfg` on the GS host
(default is 1000). This is wfb-ng-wide; `wfb-cli` tables will
animate 10× faster but byte-count divisions stay correct.

## Install

```bash
pip install -e '.[dev]'
```

## Run

```bash
dynamic-link-gs --config conf/gs.yaml.sample --log-file /tmp/dl.log
```

Replay a captured JSON stream:

```bash
# capture once from a flying wfb-ng
nc 127.0.0.1 8103 > capture.jsonl

# replay into the service offline
dynamic-link-gs --config conf/gs.yaml.sample --replay capture.jsonl --log-file /tmp/dl.log
```

Tail the decision log:

```bash
tail -f /tmp/dl.log
```

## Port mapping note

`docs/dynamic-link-design.md` refers to the stats endpoint as `stats_port`.
In wfb-ng's `master.cfg` that name refers to the msgpack feed wfb-cli
consumes; the **JSON** feed dynamic-link subscribes to is bound to
`api_port` (8103 on GS, 8102 on drone). The sample config points at
8103 by default.

## Tests

```bash
pytest
```

## Layout

```
gs/dynamic_link/         Python package
  service.py             entry point
  stats_client.py        async TCP JSON client
  signals.py             per-window aggregator + EWMA
  profile.py             radio-profile loader
  predictor.py           latency-budget predictor
  policy.py              leading + trailing loops
  decision.py            Decision dataclass
  sinks.py               LogSink

conf/
  gs.yaml.sample         sample GS config
  radios/m8812eu2.yaml   default radio profile (BL-M8812EU2)

packaging/
  dynamic-link-gs.service    systemd unit

tests/                   pytest suite
docs/                    design docs (source of truth)
wfb-ng/                  reference checkout (not built here)
```
