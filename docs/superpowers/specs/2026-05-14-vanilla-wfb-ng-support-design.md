# Vanilla wfb-ng support (no-interleaver mode)

Status: design
Owner: Gilang
Date: 2026-05-14

## Summary

Make dynamic-link operable against **upstream (vanilla) wfb-ng**, which
does not implement the `CMD_SET_INTERLEAVE_DEPTH` opcode. Today the
applier and the GS policy unconditionally assume the
`feat/interleaving_uep` branch's interleaver is available; this design
adds an opt-out so the controller can run with `depth=1` pinned and
the depth dimension entirely removed from the control loop.

Goals:

- Run unmodified against vanilla wfb-ng (no SET_INTERLEAVE_DEPTH ever
  emitted, no spurious `TX_APPLY_FAIL` for that opcode).
- One config field, one source of truth (drone-side), declared to the
  GS via the existing DLHE handshake.
- Backward compatible: today's deployments (custom branch) keep
  working with no config change.
- No wire-format version bump; the new capability rides on a free bit
  in the HELLO `flags` byte.

Non-goals:

- Runtime auto-detection of wfb-ng features. The drone declares its
  capability statically; mismatches are an operator error and surface
  in existing `TX_APPLY_FAIL` debug events.
- Mid-flight mode switching. A wfb-ng rebuild requires a drone reboot,
  which produces a fresh HELLO (new `generation_id`).
- Backporting the GS's interleaver state machine to vanilla via any
  software workaround. When the bit says vanilla, depth is 1, full
  stop.

## Background

Interleaving is implemented only on wfb-ng's `feat/interleaving_uep`
branch (vendored SHA `208ec1d`, pinned for opcode 5 = `CMD_SET_INTERLEAVE_DEPTH`).
Vanilla wfb-ng supports opcodes 1 (`CMD_SET_FEC`) and 2 (`CMD_SET_RADIO`)
only; sending opcode 5 against vanilla `wfb_tx` produces a non-zero
`rc` response.

The dependency lands in five places today:

1. **Drone applier** (`drone/src/dl_backend_tx.c::dl_backend_tx_apply`)
   — emits `send_depth(...)` on every depth change.
2. **GS policy** (`gs/dynamic_link/policy.py`) — trailing loop owns
   a depth state machine: bootstrap (1→2), refine (≥2→higher),
   step-down, cooldown.
3. **GS predictor** (`gs/dynamic_link/predictor.py`) — adds
   `(depth-1) * block_duration_ms` to the latency budget when
   evaluating proposals.
4. **GS stats parser** (`gs/dynamic_link/stats_client.py`) — pins
   `CONTRACT_VERSION_EXPECTED = 2` and requires `interleave_depth`
   to be a present key on every session record. Vanilla wfb-ng
   emits neither.
5. **Drone ceiling** (`drone/src/dl_config.[ch]`) — `safe_depth` and
   `depth_max` bounds.

All four must respect the new capability bit; otherwise the GS would
keep raising depth and the drone would either silently drop the
sub-command (if we gate it on the wire) or log per-tick errors (if we
don't).

## Configuration

### Drone (`conf/drone.conf.sample`)

```ini
# ---- Interleaver capability ----------------------------------------
# Whether the underlying wfb-ng supports the runtime
# CMD_SET_INTERLEAVE_DEPTH opcode. True only on the
# `feat/interleaving_uep` branch (SHA 208ec1d as vendored). Vanilla
# upstream wfb-ng builds: set to 0.
#
# When 0:
#   - dl-applier never sends CMD_SET_INTERLEAVE_DEPTH.
#   - The drone reports DL_HELLO_FLAG_VANILLA_WFB_NG in HELLO; the GS
#     pins depth=1 and skips its depth state machine + predictor
#     interleave-latency term.
#   - safe_depth is still honored for the wire packet but never
#     reaches wfb_tx.
interleaving_supported = 1
```

- New field in `dl_config_t`: `bool interleaving_supported`.
- Default `1` to preserve today's behavior.
- Parsed in `dl_config.c` next to existing booleans; logged once at
  applier start at INFO: `"interleaver mode: enabled"` or
  `"interleaver mode: vanilla wfb-ng (depth pinned to 1)"`.

### GS (`conf/gs.yaml.sample`)

No new field. The GS learns the capability from HELLO; until a HELLO
arrives, the GS sits in `safe_defaults` (depth=1 already), which is
safe for both modes.

## Wire format

### HELLO `flags` byte

HELLO already carries a `flags` byte at offset 5 (currently always 0)
and 8 bytes of reserved zero-fill at offsets 20–27. We allocate bit 0:

```c
/* drone/src/dl_wire.h */
#define DL_HELLO_FLAG_VANILLA_WFB_NG  (1u << 0)
```

Semantics: **bit set ⇒ vanilla, no interleaver**. Bit clear ⇒
capable (current behavior).

Choosing "set = vanilla" rather than "set = capable" means existing
drones, which always emit `flags = 0`, continue to be interpreted as
capable. No `DL_WIRE_VERSION` bump is required. Drones built against
vanilla wfb-ng must be configured with `interleaving_supported = 0`
to set the bit; if an operator misses the config, the failure is
loud (per-tick `TX_APPLY_FAIL` against opcode 5).

`gs/dynamic_link/wire.py` mirrors the constant byte-for-byte; the
existing `tests/test_wire_contract.py` hex-diff fixture is extended
to cover both `flags=0x00` and `flags=0x01` HELLO encodings.

### Decision packet

**Unchanged.** The wire packet still carries `depth` as a byte; in
vanilla mode the GS pins it to 1, the drone diff-applier sees no
change after the initial decision, and `send_depth` is gated off
regardless (see below). Keeping the field avoids a wire bump and
keeps the C/Python contract test simple.

### `dl_hello_t`

`dl_hello_t.flags` exists already as `uint8_t`; no struct change. The
existing `dl_hello_t` constructor in `drone/src/dl_hello.c` ORs the
new bit when the config says vanilla.

## GS-side changes

### `gs/dynamic_link/drone_config.py`

Extend the `DroneConfig` snapshot with a new field:

```python
@dataclass(frozen=True)
class DroneConfig:
    mtu_bytes: int
    fps: int
    interleaving_supported: bool = True   # default True for safety
    # ...existing fields
```

`on_hello(h: Hello)` reads `(h.flags & DL_HELLO_FLAG_VANILLA_WFB_NG) == 0`
and stores that as `interleaving_supported`. The HELLO event log line
gains a `"interleaver=vanilla"` or `"interleaver=enabled"` tag so
flight logs make the mode obvious without grepping config.

### `gs/dynamic_link/policy.py` (trailing loop)

`TrailingLoop.decide` short-circuits the depth dimension when the
drone capability is vanilla:

```python
def decide(self, *, current_depth, ts_ms, signals, drone_cfg, ...):
    if not drone_cfg.interleaving_supported:
        # Vanilla wfb-ng: depth is structurally 1; only the IDR
        # subsystem can fire.
        return 1, self._maybe_idr(signals, ts_ms)
    # ...existing depth bootstrap / refine / step-down ...
```

`drone_cfg` (the latest `DroneConfig` snapshot) is threaded through
the call site that currently passes signals — single new keyword
arg. Existing state fields (`last_depth_change_ts`,
`clean_windows_run`, `_loss_window_history`) remain but stay frozen
in vanilla mode (the early return prevents any update). Other
trailing-loop responsibilities (IDR requests) remain untouched.

### `gs/dynamic_link/predictor.py`

The interleave term zeroes out:

```python
interleave_ms = (
    (proposal.depth - 1) * cfg.block_duration_ms
    if drone_cfg.interleaving_supported
    else 0.0
)
```

This means the predictor's latency cap is fully consumed by
`block_fill + block_air + decode`. Predictor inputs (proposal,
config) are unchanged; we add `drone_cfg` to the call signature.

### `gs/dynamic_link/stats_client.py` (vanilla stats-feed compat)

Vanilla wfb-ng's JSON stats feed differs from `feat/interleaving_uep`'s
in two places, both of which the parser currently treats as fatal:

1. **`contract_version`**: vanilla emits `1` (or earlier); the custom
   branch emits `2`. `CONTRACT_VERSION_EXPECTED = 2` plus the strict
   inequality at `stats_client.py:144` raises `ContractVersionError`
   and aborts the stats client.
2. **`session.interleave_depth`**: this key only exists on the custom
   branch. `_parse_session` indexes it directly
   (`int(d["interleave_depth"])`), so a vanilla session record raises
   `KeyError` before `_decode_event` returns.

Changes:

- Widen the contract version check to accept the set
  `CONTRACT_VERSIONS_SUPPORTED = {1, 2}`. The strict equality is
  replaced with membership; `ContractVersionError` still fires on
  anything outside the set.
- `_parse_session` switches `interleave_depth` to
  `int(d.get("interleave_depth", 1))`. Vanilla's omission ⇒ default
  to 1, which is the truthful interleaver depth in vanilla (no
  interleaver). This decouples the parser from the controller's
  capability bit — the parser stays a pure decoder.

The capability bit (from HELLO) is the source of truth for policy;
the stats parser merely refuses to crash on a vanilla feed. This
separation matters because the stats feed is consumed before HELLO
arrives.

### `gs/dynamic_link/observed.py`

**No change.** `observed.interleave_depth` continues to report
whatever wfb-ng's RxEvent.session contains. On vanilla wfb-ng this
field is always 1 (or whatever vanilla's session defaults to);
that's truthful telemetry, not policy state.

### `gs/dynamic_link/signals.py`

No change. Signals consume `observed` and produce window statistics;
they're orthogonal to the interleaver dimension.

## Drone-side changes

### `drone/src/dl_backend_tx.c`

Gate the depth sub-command on the new config bit:

```c
if (cfg->interleaving_supported &&
    (first || prev->depth != d->depth)) {
    if (emitted) pace(bt);
    if (send_depth(bt, d->depth) < 0) rc = -1;
    emitted = true;
}
```

Same gate in `dl_backend_tx_apply_safe` — `send_depth(safe_depth)` is
skipped in vanilla mode.

`dl_backend_tx_open` does not need to read the new config field
itself; the gate is in `apply` / `apply_safe`. To keep the API
surface narrow we pass `cfg` into `apply` (it's already in scope at
the call site in `dl_applier.c`).

Update the comment block on `apply_sub_pace_ms` (currently mentions
"FEC/DEPTH/RADIO triplet") to note DEPTH is omitted in vanilla mode.

### `drone/src/dl_hello.c`

When constructing the outgoing HELLO, OR the vanilla bit into
`h.flags` if the config says so:

```c
h->flags = 0;
if (!cfg->interleaving_supported) {
    h->flags |= DL_HELLO_FLAG_VANILLA_WFB_NG;
}
```

### `drone/src/dl_config.c`

Add `interleaving_supported` to the parser table next to other
booleans; default `true`. Validate as a 0/1 integer.

### Ceiling / safe defaults

`safe_depth`, `depth_max`, and `DL_BOUND_DEPTH_MAX` (=8, mirror of
`MAX_INTERLEAVE_DEPTH`) stay as-is. They remain meaningful for
deployments with interleaving enabled; in vanilla mode they're inert
because no SET_INTERLEAVE_DEPTH is emitted. We don't gate the
ceiling-validation logic — it's a no-op cost.

## Behavior summary

| Scenario | GS depth in decision | Drone emits opcode 5? | RxEvent.session.interleave_depth |
|---|---|---|---|
| Capable drone (today) | 1..depth_max via policy | yes, on change | 1..depth_max |
| Vanilla drone | pinned to 1 | never | 1 (vanilla default) |
| Capable drone, pre-HELLO | 1 (safe_defaults) | no (no decisions yet) | n/a |
| Vanilla drone, pre-HELLO | 1 (safe_defaults) | no | n/a |
| Capability flip mid-flight | not supported; requires reboot → new generation_id | n/a | n/a |

## Failure modes

| Misconfiguration | Symptom |
|---|---|
| Vanilla wfb-ng, `interleaving_supported = 1` | Per-tick `TX_APPLY_FAIL` debug events with `cmd="set_interleave_depth"` and `rc != 0`. Loud, easy to diagnose. |
| Custom-branch wfb-ng, `interleaving_supported = 0` | Drone never raises depth; loss recovery falls back on FEC/MCS alone. No errors; degraded performance only. Operator must notice via flight logs (no depth changes ever). |
| Both endpoints rebuilt asymmetrically (drone bit toggled, GS not redeployed) | Not possible — GS learns the bit from HELLO. The GS always tracks the drone. |

## Tests

### Python (`tests/`)

- `test_stats_client.py` — new cases: (a) session JSON with
  `contract_version: 1` and no `interleave_depth` key parses to
  `interleave_depth=1` without error; (b) session JSON with
  `contract_version: 99` still raises `ContractVersionError`.
- `test_wire_contract.py` — extend the `dl-inject --dry-run` hex-diff
  to encode HELLO with `flags=0x00` and `flags=0x01`. Round-trip
  decode confirms the bit survives.
- `test_drone_config.py` (new or existing) — assert `DroneConfig`
  derived from a HELLO with the vanilla bit has
  `interleaving_supported == False`; without the bit, `True`.
- `test_policy_trailing.py` — new case: persistent loss windows with
  `drone_cfg.interleaving_supported=False` should NOT raise depth past
  1. Existing capability=True cases unchanged.
- `test_predictor.py` — new case: with `interleaving_supported=False`,
  `interleave_ms == 0` regardless of `depth`, and the latency budget
  is consumed by `block_fill + block_air + decode` only.
- `test_drone_e2e.py` — new vanilla-config scenario where the
  applier is launched with `interleaving_supported=0`. Drive several
  decisions including depth=2, depth=3, etc.; assert opcode 5
  (`CMD_SET_INTERLEAVE_DEPTH`) is NEVER captured by the mock
  `wfb_tx`. Assert the HELLO seen by the mock GS has the vanilla bit
  set.

### C (`tests/drone/`)

- `test_backend_tx.c` (or wherever the apply path is unit-tested) —
  with a fake tx-cmd sink, run `dl_backend_tx_apply` against a config
  with `interleaving_supported=false`; assert no opcode-5 packet was
  written even when `d->depth != prev->depth`.
- `test_hello.c` — encode a HELLO from a vanilla-config drone;
  assert `flags & DL_HELLO_FLAG_VANILLA_WFB_NG`.

## Docs

- `docs/dynamic-link-design.md` §2 — add a short subsection
  ("Compatibility with vanilla wfb-ng") describing the bit and
  pinned-depth behavior. Cross-reference §4.2 (depth state machine)
  to note it's a no-op in vanilla mode.
- `README.md` — operator-facing note in the prerequisites section:
  "If your drone is running upstream wfb-ng, set
  `interleaving_supported = 0` in drone.conf."
- `CLAUDE.md` — add `interleaving_supported` to the operational
  prerequisites list (alongside `wireless.mlink` and `video0.fps`).
- `drone/src/vendored/README.md` — note that the vendored
  `tx_cmd.h` is forward-compatible with vanilla: vanilla wfb_tx
  ignores the unknown opcodes (or returns rc != 0); we just don't
  send them when configured for vanilla.

## Out of scope (future work)

- A telemetry/OSD indicator showing the active interleaver mode at
  a glance. The HELLO log line + flight-log tag covers diagnosis
  today.
- Probing vanilla wfb_tx for opcode support at runtime. Deferred
  because (a) it requires either a probe opcode or interpreting
  error responses, and (b) explicit config is more honest about
  what's deployed.
- Re-vendoring `tx_cmd.h` from vanilla wfb-ng. We keep the
  `feat/interleaving_uep` header because the `cmd_req_t` layouts for
  opcodes 1 (`CMD_SET_FEC`) and 2 (`CMD_SET_RADIO`) are byte-identical
  between vanilla and the custom branch — the custom branch only adds
  the new opcode 5 union arm and bumps `WFB_IPC_CONTRACT_VERSION`. As
  long as we gate opcode 5 off, the wire we put on the socket is
  exactly what vanilla wfb_tx expects.
