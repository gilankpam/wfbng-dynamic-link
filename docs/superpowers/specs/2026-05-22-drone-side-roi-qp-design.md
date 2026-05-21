# Drone-side ROI QP

**Status:** design approved 2026-05-22, pre-implementation.

Lower the encoder's center-band QP when bitrate is starved, so center
detail is preserved under degraded link conditions. Calculation lives
on the drone, derived purely from the bitrate the GS has already
decided to send.

## Motivation

The GS adaptive controller already varies `bitrate_kbps` based on link
quality. At low bitrates the encoder spreads bits evenly and the
entire frame softens. For an FPV pilot the center of the frame is
where attention is — anything sharpening that region (at the cost of
softer edges) is a perceptual win.

waybeam_venc exposes `fpv.roiQp` (signed delta QP, range -30..30) over
its live HTTP API, applied as horizontal full-height bands tapered
from center to edge. Negative values sharpen the center. The encoder
side is already implemented and verified (see
waybeam_venc/documentation/ROI_INVESTIGATION.md).

This change wires up the dynamic-link side: a deterministic mapping
from `bitrate_kbps` to `roi_qp`, computed in the drone applier.

## Why drone-side

The mapping is a pure function of bitrate. Computing on the drone
avoids:

- Widening the GS→drone wire (it currently carries a stale,
  always-zero `roi_qp` byte the GS controller never sets).
- Coupling the GS controller to encoder-specific knobs.
- Drift between GS-decided bitrate and ROI: any path that updates
  bitrate (normal apply, safe-defaults, future safe modes) gets ROI
  for free without each callsite remembering.

## Policy

Linear ramp from `0` at `threshold_kbps` down to `floor` at
`low_anchor_kbps`, clamped at `floor` below the anchor, quantized to
multiples of `step` to suppress per-tick churn.

```
roi_qp = 0                                                  if bitrate >= threshold
roi_qp = quantize(floor * (threshold - bitrate) / span)     if low_anchor <= bitrate < threshold
roi_qp = floor                                              if bitrate < low_anchor

span     = threshold - low_anchor
quantize = (raw / step) * step    (integer division, truncates toward zero;
                                   `raw` is negative, so result ≥ `raw`)
```

Reference implementation (drone-side, C):

```c
static int compute_roi_qp(uint16_t bitrate_kbps, const dl_config_t *cfg) {
    if (bitrate_kbps >= cfg->roi_qp_threshold_kbps) return 0;
    int span  = cfg->roi_qp_threshold_kbps - cfg->roi_qp_low_anchor_kbps;
    int delta = (int)bitrate_kbps - cfg->roi_qp_low_anchor_kbps;
    if (delta < 0) delta = 0;
    int raw = (cfg->roi_qp_floor * (span - delta)) / span;
    int q   = (raw / cfg->roi_qp_step) * cfg->roi_qp_step;
    if (q < cfg->roi_qp_floor) q = cfg->roi_qp_floor;
    return q;
}
```

### Defaults

| Knob                    | Default | Range    | Meaning                              |
| ----------------------- | ------- | -------- | ------------------------------------ |
| `roi_qp_threshold_kbps` | 6000    | int      | bitrate at/above which `roi_qp = 0`  |
| `roi_qp_low_anchor_kbps`| 2000    | int      | bitrate at which `roi_qp = floor`    |
| `roi_qp_floor`          | -24     | -30..0   | most-negative QP delta               |
| `roi_qp_step`           | 3       | 1..10    | quantization                         |

`roi_qp_floor` is clamped at `-24` rather than the SDK ceiling
`-30` to leave headroom. waybeam's SDK rejects out-of-range values
and would silently no-op the entire `apply_set` if we ever passed
`-31` (e.g. via a misconfigured floor below `-30`).

### Sample trajectory (defaults)

| bitrate (kbps) | raw         | quantized | sent       |
| --------------:| -----------:| ---------:| ----------:|
| 8000           |  0          |  0        | `roiQp=0`  |
| 6000           |  0          |  0        | `roiQp=0`  |
| 5000           |  -6         |  -6       | `roiQp=-6` |
| 4000           | -12         | -12       | `roiQp=-12`|
| 3000           | -18         | -18       | `roiQp=-18`|
| 2000           | -24         | -24       | `roiQp=-24`|
| 1000           | -24 (clamp) | -24       | `roiQp=-24`|

## Configuration

New keys in `drone.conf`, all under `[encoder]`, all optional:

```ini
[encoder]
roi_qp_threshold_kbps  = 6000
roi_qp_low_anchor_kbps = 2000
roi_qp_floor           = -24
roi_qp_step            = 3
```

### Validation (at boot, in `dl_config.c`)

Reject and exit with a clear log line on:

- `roi_qp_threshold_kbps <= roi_qp_low_anchor_kbps` (divide-by-zero
  and inverted ramp).
- `roi_qp_floor` outside `-30..0`. Positive floor inverts the
  semantics (softer center); below `-30` triggers SDK reject.
- `roi_qp_step` outside `1..10`. Zero divides; >10 is too coarse to
  be useful.

Same int-range pattern as `min_idr_interval_ms` in the existing
parser.

## Wire format change (v1 → v2)

The `roi_qp` byte at v1 payload offset 24 is removed. All bytes after
it shift down by one. The `_pad` and `_pad2` regions are retained
unchanged. Payload shrinks from 28 → **27** bytes; on-wire from 32 →
**31** (CRC unchanged). `DL_WIRE_VERSION` bumps to **2**.

### v1 layout (current, for reference — from `dl_wire.c:1-31`)

```
off  size  field
 0    4   magic (DLK1)
 4    1   version (=1)
 5    1   flags
 6    2   _pad (0)
 8    4   sequence
12    4   timestamp_ms
16    1   mcs
17    1   bandwidth
18    1   tx_power_dBm
19    1   k
20    1   n
21    1   depth
22    2   bitrate_kbps
24    1   roi_qp        <-- removed in v2
25    1   fps
26    2   _pad2 (0)
28    4   CRC-32        (on-wire only; not in payload)
                        payload=28, on-wire=32
```

### v2 layout (new)

```
off  size  field
 0    4   magic (DLK1)
 4    1   version (=2)
 5    1   flags
 6    2   _pad (0)
 8    4   sequence
12    4   timestamp_ms
16    1   mcs
17    1   bandwidth
18    1   tx_power_dBm
19    1   k
20    1   n
21    1   depth
22    2   bitrate_kbps
24    1   fps           <-- shifted down from v1 offset 25
25    2   _pad2 (0)
27    4   CRC-32        (on-wire only; not in payload)
                        payload=27, on-wire=31
```

Authoritative constants:

- `DL_WIRE_VERSION = 2`
- `DL_WIRE_PAYLOAD_SIZE = 27`
- `DL_WIRE_ON_WIRE_SIZE = 31`

The Python encoder, C encoder, and `tests/test_wire_contract.py`
hex-diff together pin the exact byte layout; any drift surfaces
immediately in that test.

### Files touched by the wire bump

- `drone/src/dl_wire.h` — constants, `dl_decision_t` (remove `roi_qp`).
- `drone/src/dl_wire.c` — encode/decode + offsets.
- `gs/dynamic_link/wire.py` — mirror layout, drop `roi_qp` param.
- `drone/src/dl_inject.c` — drop `--roi-qp` CLI flag.
- `tests/test_wire_contract.py` — hex-diff against `--dry-run`.
- `tests/test_wire.py` — offset assertions.
- `tests/test_drone_e2e.py` — any decision construction.
- `gs/dynamic_link/` — any other call sites that pass `roi_qp` (the
  policy engine already passes `0`; remove the param entirely).

GS and drone deploy lockstep. No coexistence with v1.

## Encoder backend changes

### Per-backend state

`dl_backend_enc.c` grows its own `last_*` state, matching the per-
backend prev-struct pattern (see CLAUDE.md "Per-backend state
tracking"):

```c
struct dl_backend_enc {
    /* existing host/port/idr fields ... */
    dl_config_t cfg_snapshot;  /* roi_qp_threshold_kbps etc. */
    bool        last_valid;
    uint16_t    last_bitrate_kbps;
    int8_t      last_roi_qp;
    uint8_t     last_fps;
};
```

`dl_applier.c` stops carrying a `last_enc` `dl_decision_t` for
dedup-against-input — the backend owns its own diff state. (`last_enc`
in `dl_applier.c` may stay for OSD/telemetry but is no longer a
correctness input to the apply path.)

### Apply path

```c
int dl_backend_enc_apply(dl_backend_enc_t *be, const dl_decision_t *d) {
    int8_t roi_qp = (int8_t)compute_roi_qp(d->bitrate_kbps, &be->cfg_snapshot);
    if (be->last_valid &&
        be->last_bitrate_kbps == d->bitrate_kbps &&
        be->last_roi_qp       == roi_qp &&
        be->last_fps          == d->fps) {
        return 0;
    }
    int rc = apply_set(be, d->bitrate_kbps, roi_qp, d->fps);
    if (rc == 0) {
        be->last_bitrate_kbps = d->bitrate_kbps;
        be->last_roi_qp       = roi_qp;
        be->last_fps          = d->fps;
        be->last_valid        = true;
    }
    return rc;
}
```

### Bug fix in `apply_set`

Current code:

```c
if (roi_qp != 0) {
    n = snprintf(p, left, "&fpv.roiQp=%u", roi_qp);
    ...
}
```

Two problems:

1. `%u` on an unsigned field cannot send negative values.
2. Skipping emission when `roi_qp == 0` means we can never actively
   *disable* ROI — once a negative value is set, going back to `0`
   is silently dropped.

Fix:

- Signature: `int apply_set(..., int8_t roi_qp, uint8_t fps)`.
- **Always emit** `&fpv.roiQp=%d`, including `=0`. `0` is a
  legitimate "disable ROI" command per waybeam's contract.

The `fps != 0` skip in the same function is unrelated and stays —
fps semantics are different ("0 = unset, leave as-is").

### Safe-defaults

`dl_backend_enc_apply_safe` runs the same `compute_roi_qp` against
`cfg->safe_bitrate_kbps`. ROI tracks bitrate uniformly, including
under watchdog trip. The operator picks `safe_bitrate_kbps` knowing
what `roi_qp` it implies (or accepts the default mapping).

### What stays the same

- `min_idr_interval_ms`, IDR throttle, parse_http_status, all timeout
  and error-surfacing behavior — untouched.
- ROI changes do not trigger IDR. ROI is per-CTU; no SPS/PPS change.

## Testing

### C unit tests

New file `tests/drone/test_roi_qp.c`:

- `roi_qp_above_threshold_is_zero` — `bitrate=6000` → `0`,
  `bitrate=10000` → `0`.
- `roi_qp_at_low_anchor_is_floor` — `bitrate=2000` → `-24`.
- `roi_qp_below_low_anchor_clamps_at_floor` — `bitrate=1000`,
  `bitrate=500` → `-24` (no overflow toward `-30`).
- `roi_qp_midpoint_ramps_linearly` — `bitrate=4000` → `-12`.
- `roi_qp_quantization_steps` — sweep bitrates 2000..6000 in 50-kbps
  steps; assert every result is a multiple of `roi_qp_step`.
- `roi_qp_custom_config` — non-default `threshold=8000`,
  `low_anchor=3000`, `floor=-18`, `step=2`; verify endpoints and
  midpoint.

Extend `tests/drone/test_dl_backend_enc.c` (create if absent):

- Backend dedupes when computed `roi_qp` matches `last_roi_qp` even
  with small bitrate jitter inside one quantization step.
- `apply_set` always emits `&fpv.roiQp=` including when the value is
  `0` (capture the GET path via a mock socket).
- Negative `roi_qp` formats with a `-` (not `%u`-style wrap-around).

### Config tests

Extend `tests/drone/test_dl_config.c`:

- `roi_qp_threshold_kbps <= roi_qp_low_anchor_kbps` → reject.
- `roi_qp_floor` outside `-30..0` → reject.
- `roi_qp_step` outside `1..10` → reject.
- Default values applied when keys absent.

### Python / E2E tests

- `tests/test_wire_contract.py` — re-runs `dl-inject --dry-run` ↔
  Python encoder hex diff for v2 layout.
- `tests/test_wire.py` — offsets, `roi_qp` removed.
- `tests/test_drone_e2e.py` — bitrate-only decision; assert mock
  encoder HTTP receives `fpv.roiQp` value matching the formula.
  Cover the ramp (e.g. `bitrate=4000` → `-12`) and the disable
  case (`bitrate=8000` → `0`, explicitly sent).

All C tests run via `make -C drone test`. All Python tests run via
`python3 -m pytest --ignore=tests/test_mavlink_status.py`.

## Out of scope

- `fpv.roiEnabled` / `roiSteps` / `roiCenter` — operator-owned, set
  once in `waybeam.json`. Applier does not probe or write them.
- Majestic-specific ROI behavior — `fpv.roiQp` is sent to whichever
  encoder; if majestic doesn't honor the field the request is a
  harmless no-op. No `encoder_kind` branching.
- GS-side policy or telemetry on `roi_qp`. If we later want it on
  the OSD it lands via STATUSTEXT or sidecar; this spec does not
  require it.
- Non-linear / hysteretic curves. Quantization is the only damping.
- IDR coupling on `roi_qp` changes.
- `waybeam.json` provisioning. README gains a one-paragraph note
  that ROI must be pre-enabled with sensible `roiSteps`/`roiCenter`;
  the applier does not write the file.

## Operational prerequisite (add to README + this spec)

For the feature to have visible effect, the drone's `/etc/waybeam.json`
must contain:

```json
"fpv": {
    "roiEnabled": true,
    "roiSteps": 2,
    "roiCenter": 0.25,
    "roiQp": 0
}
```

If `roiEnabled` is `false`, the applier's `roiQp` updates are stored
by waybeam but produce no visible ROI banding. The applier does not
detect or warn about this state — it's an operator-config concern.
