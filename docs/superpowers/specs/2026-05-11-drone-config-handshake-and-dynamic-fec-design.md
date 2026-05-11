# Drone config handshake + dynamic FEC

Design spec — 2026-05-11.

Companion to `docs/dynamic-link-design.md`. Supersedes the
operator-validated `fec_table` mechanism in radio profiles.

## Motivation

Today the GS picks `(k, n)` by indexing `profile.fec_table[bw][mcs]`
— a table the operator hand-tunes per airframe. The table bakes in
two assumptions:

1. **`mtu_bytes` matches the GS-side hardcoded `1400`.** The actual
   radio MTU lives in `/etc/wfb.yaml`'s `wireless.mlink` on the
   drone (often `3994` — substantially larger). The GS-side
   `fec_table` is silently calibrated against the wrong MTU on most
   real installs.
2. **`fps` is whatever the operator typed into `gs.yaml`.** The
   real FPS the encoder is running at lives in
   `/etc/majestic.yaml`'s `video0.fps`. If the operator changes
   one and forgets the other, the table no longer reflects the
   per-frame packet budget — block-fill drifts past a frame
   boundary and adds latency the controller never sees.

This spec moves the authoritative source of `mtu_bytes` and `fps`
to the drone (which already owns those configs), adds a small
drone→GS handshake to report them, and replaces the static
`fec_table` with a per-tick computation parameterised by
`(bitrate, mtu, fps)` plus a revived `n`-escalation loop driven by
`residual_loss`.

## Non-goals

- No change to MCS selection, the leading/trailing loops, the
  predictor's overall shape, or the wfb-ng stats feed.
- No drone-side FPS or MTU writes — drone is read-only against
  `/etc/wfb.yaml` and `/etc/majestic.yaml`. Operator still
  configures these files.
- No `(k, n)` discovery from the encoder over HTTP. Drone parses
  the YAML files directly.
- No general YAML support on the drone — just a line-scan that
  handles the two specific file shapes.

## Architecture

Two concerns, one feature:

1. **Drone → GS config handshake** — drone announces `mtu_bytes`
   and `fps` to the GS via a new `DLHE` packet on the wfb-ng
   back-tunnel; GS replies with `DLHA` on the existing GS→drone
   wire. State machine governs cadence and reboot detection via
   `generation_id`.
2. **Dynamic FEC selection** — once GS has `(mtu, fps)`, the
   policy loop computes `k` from per-frame packet budget (clamped
   to operator bounds) and lets `n` evolve with `residual_loss`
   under hysteresis. Emit-gating bundles `(k, n)` changes onto
   MCS-change ticks to honour the `knob-cadence-bench.md` lesson.

Until a HELLO has been received and ACKed, the GS holds its
decisions at `safe_defaults` (existing `safe.video.{k,n}` and
`safe.depth`). Drone reboot (detected by `generation_id` change in
an incoming HELLO) → GS reverts to AWAITING + safe_defaults until
the new handshake completes.

## Wire format

Both packet types follow the existing `DLK1` shape (32 bytes total,
magic-first, version byte at offset 4, CRC32 trailer). Existing
dedup/CRC machinery handles them with no new infrastructure.

### `DLHE` (drone → GS, on the wfb-ng back-tunnel — same UDP path Phase 3 PONG uses)

```
offset  size  field
0       4     magic = 'DLHE' (0x444C4845)
4       1     version = 1
5       1     flags (reserved, 0)
6       2     padding (zero)
8       4     generation_id (uint32 BE, random per drone boot)
12      2     mtu_bytes (uint16 BE)
14      2     fps (uint16 BE)
16      4     applier_build_sha (uint32 BE, low 4 bytes of git SHA)
20      8     reserved (zero)
28      4     CRC32
total: 32 bytes
```

`fps` is `uint16` (not `uint8` as in the GS→drone wire's existing
FPS field) — defensive headroom for the future, even though we
expect ≤120 fps in practice.

### `DLHA` (GS → drone, on the existing `DLK1` UDP wire)

```
offset  size  field
0       4     magic = 'DLHA' (0x444C4841)
4       1     version = 1
5       3     padding (zero)
8       4     generation_id_echo (uint32 BE, copied from acked HELLO)
12      16    reserved (zero)
28      4     CRC32
total: 32 bytes
```

### Where the layouts live

- C: `drone/src/dl_wire.h` grows `struct dl_hello_packet` and
  `struct dl_hello_ack_packet`. Existing `struct dl_decision_packet`
  unchanged.
- Python: `gs/dynamic_link/wire.py` grows `encode_hello`,
  `decode_hello`, `encode_hello_ack`, `decode_hello_ack` mirroring
  the C layout byte-for-byte.
- Contract test: `tests/test_wire_contract.py` extended with
  `dl-inject --hello` and `dl-inject --hello-ack` dry-run paths;
  Python encoder hex-matched against the C tool output.

## Drone-side state machine

New module `drone/src/dl_hello.c` (+ `.h`).

```
              boot
               │
               ▼
        ┌──────────────┐    timer (500 ms)        send DLHE
        │  ANNOUNCING  │ ────────────────────►
        │              │ ◄──── recv DLHA matching generation_id
        └──────┬───────┘
               │ ACK matches gen_id
               ▼
        ┌──────────────┐    timer (10 s)
        │   KEEPALIVE  │ ────────────────────►   send DLHE
        │              │
        └──────┬───────┘
               │ no ACK for 3 keepalive intervals (30 s)
               ▼
        (drop back to ANNOUNCING)
```

- `generation_id` set once at boot: read 4 bytes from
  `/dev/urandom`; if that fails, fall back to a hash of
  `(boot_time_ms, pid)`. The value is logged on startup for triage.
- ANNOUNCING: 500 ms cadence for the first 30 s (60 retries),
  then 5 s cadence indefinitely. A long uplink outage still
  recovers without operator intervention.
- KEEPALIVE: 10 s cadence. ACK observed → reset the
  no-ack-for-3-intervals timer. Timeout → ANNOUNCING.
- ACK matching: `generation_id_echo` from the DLHA must equal the
  current `generation_id`. Any other value is ignored (stale or
  spurious).

### MTU + FPS sourcing at boot

Both read at module init via the same helper
`dl_yaml_get_int(path, block, key, *out)`:

| Field | File | Block | Key |
|---|---|---|---|
| `mtu_bytes` | `/etc/wfb.yaml` | `wireless` | `mlink` |
| `fps` | `/etc/majestic.yaml` | `video0` | `fps` |

`dl_yaml_get_int` is a ~40-line line-scan parser:

1. Open `path` read-only; bail with `-ENOENT` on failure.
2. Scan for a line beginning in column 0 matching `<block>:`.
3. Within that block (lines starting with whitespace, until the
   next column-0 line), scan for a line matching `<indent>+<key>:`
   followed by an integer value.
4. Return the integer in `*out`; return `-EINVAL` on any
   structural deviation (missing block, missing key, non-integer
   value, comment-only block).

Robust to: trailing whitespace, trailing `#`-comments, blank
lines, CRLF. Not robust to: nested blocks beyond depth-1,
multi-line scalars, anchors/aliases, flow-style. The two files
we read don't use those constructs.

Parse failure on either file → `dl_hello` logs
`dl_hello: failed to read mtu (/etc/wfb.yaml wireless.mlink): <errno>`
and **refuses to enter ANNOUNCING**. The GS therefore stays in
AWAITING + safe_defaults forever, which is operator-visible. No
silent fallback to compiled-in defaults — the FEC math depends on
these values being correct.

Configs are read once at boot. Re-reading on SIGHUP is out of
scope; the user-described "drone reboot on battery swap" already
re-reads everything.

## GS-side state machine

New module `gs/dynamic_link/drone_config.py`.

```
                start
                 │
                 ▼
        ┌────────────────┐
        │   AWAITING     │ ─── policy loop emits safe_defaults
        │                │     (no computed FEC)
        └────────┬───────┘
                 │ recv DLHE → record (gen_id, mtu, fps);
                 │ reply DLHA
                 ▼
        ┌────────────────┐
        │   SYNCED       │ ─── policy loop emits computed (k, n)
        │   gen=X        │     reply DLHA on every received HELLO
        └────────┬───────┘     (idempotent — covers KEEPALIVE)
                 │ recv DLHE with gen_id != X
                 ▼
            (transition to AWAITING with new gen_id;
             reply DLHA; transition to SYNCED on same tick)
```

- "Reply DLHA on every received HELLO" makes the ACK lossless from
  the GS side. If the drone retries in ANNOUNCING, every retry
  gets a fresh ACK.
- AWAITING → SYNCED transition logs `drone_config_sync gen=X
  mtu=M fps=F sha=...` into the flight log.
- gen_id change logs `drone_reboot_detected gen=X->Y` then the
  fresh sync line.
- Bad-version or CRC-fail packets are dropped silently (drone
  retries cover this).

### Integration with the policy loop

`PolicyEngine` gains a reference to `DroneConfigState`:

- `state.is_synced()` — gates the computed-FEC path.
- `state.mtu_bytes` / `state.fps` — read each tick when computing
  `k`.
- On `AWAITING`, the policy `tick()` returns a `Decision` with the
  safe-default `(k, n, depth)` regardless of MCS / signals.

### FPS authority flip

Today `wire.py` includes FPS in the GS→drone `DLK1` decision
packet at offset 25 (`uint8`). With this design the drone is
authoritative for FPS, so:

- GS no longer sources FPS from `gs.yaml`'s `encoder.fps`.
- GS sends sentinel `0` (= "leave alone") in the `DLK1` FPS field
  on every tick. The wire byte stays — no version bump — and the
  drone-side encoder backend already handles `0` as a no-op.
- `gs.yaml`'s `encoder.fps` is treated as a legacy key and logged
  on load with a "now drone-reported" warning (same pattern as
  `service.py:184-194`).

## Dynamic FEC algorithm

Runs every policy tick (10 Hz). Compute every tick; gate the
emission to honour the cadence-cost lesson from
`knob-cadence-bench.md`.

### Step 1 — base `k`

```python
def compute_k(bitrate_kbps: int, mtu_bytes: int, fps: int,
              k_min: int, k_max: int) -> int:
    packets_per_frame = (bitrate_kbps * 1000) / (fps * mtu_bytes * 8)
    return max(k_min, min(k_max, int(packets_per_frame)))
```

`bitrate_kbps` is the live `compute_bitrate_kbps(...)` result —
already MCS-driven. `mtu_bytes` / `fps` are read from
`DroneConfigState`. `k_min` / `k_max` are operator bounds from
`gs.yaml` (defaults `4` and `16`).

### Step 2 — `n` from `k` + adaptive parity

```python
n_base = math.ceil(k * (1 + base_redundancy_ratio))
n      = n_base + n_escalation
n_max  = math.ceil(k * (1 + max_redundancy_ratio))
n      = min(n, n_max)
```

`n_escalation` is an integer maintained across ticks:

- `residual_loss_w > n_loss_threshold` for `n_loss_windows`
  consecutive ticks → `n_escalation += n_loss_step`.
- `residual_loss_w == 0` for `n_recover_windows` consecutive
  ticks → `n_escalation = max(0, n_escalation - n_recover_step)`.
- Clamped to `[0, max_n_escalation]`.

Defaults (matching the spirit of the removed knobs documented in
`service.py:184-194`):

| Key | Default |
|---|---|
| `base_redundancy_ratio` | `0.5` |
| `max_redundancy_ratio` | `1.0` |
| `n_loss_threshold` | `0.02` |
| `n_loss_windows` | `3` |
| `n_loss_step` | `1` |
| `n_recover_windows` | `10` |
| `n_recover_step` | `1` |
| `max_n_escalation` | `4` |

### Step 3 — emit-gating (cadence protection)

`PolicyEngine` tracks `last_emitted_k`, `last_emitted_n`,
`ticks_since_emit`. The newly computed `(k, n)` is emitted when:

- The MCS row changed this tick (a "big change" tick — bundle FEC
  in alongside MCS), OR
- `abs(new_k - last_emitted_k) >= 1` AND `ticks_since_emit >= 2`,
  OR
- `new_n != last_emitted_n` AND `ticks_since_emit >= 2`.

Otherwise `(last_emitted_k, last_emitted_n)` is carried forward.
This is the hysteresis the bench called for: rewrites only ride
along with MCS changes, or after a debounce window.

### Step 4 — latency-budget gate (unchanged)

`fit_or_degrade()` still runs as the defensive last gate. The
only thing that changes is `predictor_cfg.inter_packet_interval_ms`
now uses `state.mtu_bytes` instead of `cfg.fec.mtu_bytes`.

## Config changes

### `gs.yaml`

```yaml
fec:
  # REMOVED: mtu_bytes (now drone-reported)
  depth_max: 3
  k_bounds:                          # NEW
    min: 4
    max: 16
  base_redundancy_ratio: 0.5         # NEW (revived)
  max_redundancy_ratio: 1.0          # NEW (revived)
  n_loss_threshold: 0.02             # NEW
  n_loss_windows: 3                  # NEW
  n_loss_step: 1                     # NEW
  n_recover_windows: 10              # NEW
  n_recover_step: 1                  # NEW
  max_n_escalation: 4                # NEW

encoder:
  # `fps` is now drone-reported; key kept for back-compat but
  # ignored. Logged as legacy on load.
```

`service.py:184-194` already has the "ignored legacy key" warning
machinery; we extend it to cover `fec.mtu_bytes` and
`encoder.fps`, and remove `base_redundancy_ratio` /
`max_redundancy_ratio` from the legacy list (they're live again).

### `drone.conf`

No changes. MTU and FPS come from `/etc/wfb.yaml` and
`/etc/majestic.yaml` directly.

### Radio profile (`conf/radios/*.yaml`)

The `fec_table` is removed. Each MCS row collapses to a single
PHY-rate entry (already present as `data_rate_Mbps_LGI[bw][mcs]`).

`compute_bitrate_kbps` currently multiplies PHY rate × utilization
× `(k/n)`. With dynamic `(k, n)` we have a chicken-and-egg
problem (compute_k needs bitrate; bitrate would need k) and a
potential feedback loop (`n_escalation` lowers k/n → bitrate drops
→ IPI grows → k drops → n_max drops → n drops → bitrate rises…).

**Resolution: decouple bitrate from the dynamic `(k, n)`.**
`compute_bitrate_kbps` uses a fixed `k/n` derived from
`base_redundancy_ratio` (i.e., `k_over_n_for_bitrate =
1/(1+base_redundancy_ratio)`), not the live values. Rationale:

- The encoder is told a steady target bitrate; `n_escalation`
  reserves additional airtime for parity *out of the
  utilization-factor headroom*, not out of the encoder's
  allocation. This matches operator intuition ("encoder is allowed
  X Mb/s") and avoids encoder rate-control oscillation under FEC
  escalation.
- No circular dependency. Per-tick order is unambiguous:
  1. MCS selection (signals only).
  2. `bitrate_kbps` from PHY × utilization × fixed `k/n`.
  3. `compute_k(bitrate_kbps, mtu, fps, k_min, k_max)`.
  4. `n` escalation update.
  5. Emit-gating.
  6. `fit_or_degrade()` defensive gate.

The `bitrate.py` API gains an explicit `base_redundancy_ratio`
parameter; callers pass it instead of a live `(k, n)` pair. Old
signature (`compute_bitrate_kbps(profile, bw, mcs, k, n, cfg)`)
becomes `compute_bitrate_kbps(profile, bw, mcs, cfg)` with the
ratio resident on `BitrateConfig`. Existing tests update
accordingly.

## Failure modes

| Scenario | Behavior |
|---|---|
| Drone never sends HELLO (parse failure) | GS stays AWAITING, emits safe_defaults forever. Log line at GS each tick (rate-limited 1/s). |
| HELLO arrives, CRC fails | Drop silently. Drone retries in ANNOUNCING. |
| HELLO has `version != 1` | GS logs `drone_config_version_mismatch sha=...` and stays AWAITING. Build-SHA in the log helps triage drone-vs-GS skew. |
| Drone reboots, new `generation_id` | GS reverts to AWAITING + safe_defaults, then immediately re-syncs on the same incoming packet (state machine handles both transitions atomically). Log: `drone_reboot_detected gen=X->Y`. |
| MTU/FPS in `/etc/wfb.yaml` / `/etc/majestic.yaml` mismatch actual wfb-ng / encoder runtime | Out of scope. Drone takes the files as truth. Future: optional encoder HTTP GET sanity check on startup. |
| DLHA never reaches drone (uplink loss) | Drone retries ANNOUNCING every 500 ms. GS ACKs every received HELLO. First uplink-clear moment closes the loop. |
| Replay DLHE with stale `generation_id` (same as current) | Idempotent — GS replies DLHA, state unchanged. |
| Encoder HTTP backend already running with a different FPS than `/etc/majestic.yaml` | Out of scope. Operator's responsibility to restart encoder after editing its config. |
| First few ticks at startup before bitrate stabilises | `(k, n)` seeded from `safe_defaults`. Previous-tick `(k/n)` ratio in the bitrate calc converges in 1 tick. |
| `gs.yaml` `k_bounds.min > k_bounds.max` | Config-load validation in `service.py` raises before service starts. |

## Testing

### Wire contract

- `tests/test_wire_contract.py` extended with two dry-run paths:
  `dl-inject --hello --gen-id X --mtu M --fps F --build-sha S`
  and `dl-inject --hello-ack --gen-id X`. Python encoder hex-matched
  byte-for-byte against the C tool's output.

### Drone unit (C, `tests/drone/`)

- `test_dl_yaml.c` — line-scan parser against the example
  `/etc/wfb.yaml` content from the spec, against a sample
  `/etc/majestic.yaml`, and mutated variants: missing block,
  missing key, comment-only block, CRLF, trailing whitespace,
  wrong indentation, non-integer value, file-not-found.
- `test_dl_hello.c` — state machine: ANNOUNCING retry cadence,
  ACK matching (correct gen_id, wrong gen_id, malformed),
  ANNOUNCING → KEEPALIVE transition, KEEPALIVE → ANNOUNCING after
  3 missed ACKs. Deterministic timer injection.
- `test_dl_wire.c` — `dl_hello_packet` and `dl_hello_ack_packet`
  CRC + byte-layout assertions.

### GS unit (Python, `tests/`)

- `test_drone_config.py` — state machine: AWAITING → SYNCED on
  first HELLO, idempotent ACK on KEEPALIVE, gen_id change → revert
  + re-sync atomically, CRC-fail + version-mismatch drop paths.
- `test_dynamic_fec.py` — `compute_k` clamping at `k_min` /
  `k_max` boundaries; `n_escalation` hysteresis (loss escalates,
  recover decrements, clamped at `max_n_escalation`); emit-gating
  bundles `(k, n)` change with MCS-change ticks; debounce holds
  off-MCS-tick changes for `>= 2` ticks.
- Extend `test_policy.py` — `tick()` returns safe_defaults while
  `DroneConfigState` is AWAITING; switches to computed `(k, n)`
  after `record_hello()`.

### E2E (Python, `tests/test_drone_e2e.py`)

- Add a "handshake then decide" path: bring up `dl-applier` with
  test `/etc/wfb.yaml` + `/etc/majestic.yaml` fixtures (mocked via
  env var override or symlink in the sandbox), assert GS receives
  `DLHE`, replies `DLHA`, drone transitions to KEEPALIVE.
- Add a "drone restart" sub-test: kill + respawn the applier
  subprocess, assert new `generation_id` triggers GS revert + re-sync.
- Add a "stale MTU/FPS" negative path: corrupt `/etc/majestic.yaml`,
  start the applier, assert no HELLO is sent and the applier logs
  the parse failure.

### Replay

- `python3 -m dynamic_link.service --replay capture.jsonl` invoked
  on a flight capture, assert dynamic-FEC trace stays within
  `[k_min, k_max]`, emit-gating cadence honoured (no rewrites on
  consecutive ticks except across MCS boundaries), and the new
  `drone_config_*` log lines appear at expected points.

## Phasing

This is a Phase 4 candidate (after Phase 3 PING/PONG):

1. **P4a — handshake plumbing.** `dl_yaml_get`, `dl_hello` (drone),
   `drone_config.py` (GS), wire format + contract test. No FEC
   change yet — GS still uses `fec_table`, but the HELLO is
   observed and logged.
2. **P4b — dynamic FEC.** `compute_k`, `n`-escalation,
   emit-gating, bitrate-circular-dep resolution, profile YAML
   cleanup. Cut over when P4a has run a flight cleanly.

Splitting de-risks the wire change from the algorithm change.
P4a is the bigger surface (new module, new C parser, new
state-machine pair); P4b is mostly Python policy refactor.

## Operator prerequisites (additions to CLAUDE.md)

- `/etc/wfb.yaml` must contain `wireless.mlink: <integer>`. Most
  installs already do.
- `/etc/majestic.yaml` must contain `video0.fps: <integer>`. Most
  installs already do.
- After editing either file, restart the drone (which already
  re-reads everything on the user-described battery-swap flow).
