# Debug OSD: per-apply-call latency stats

Status: design
Owner: Gilang
Date: 2026-05-13

## Summary

Add an opt-in debug OSD that surfaces drone-side latency statistics for
every parameter-change call the applier makes. Six call sites are
tracked, one OSD line each, refreshed at the existing OSD cadence
(`osd_update_interval_ms`, default 1000 ms).

Goals:

- Operator can see, in flight, whether a given knob (FEC, depth, radio,
  TX power, encoder bitrate, encoder IDR) is fast, slow, or failing.
- Zero cost when the flag is off; minimal cost when on.
- Pure drone-side change; no wire-format bump, no GS code change.

Non-goals:

- GS-side display of these stats.
- Runtime flag toggling without an applier restart.
- Configurable window size.

## Tracked call sites

| ID | Helper | File | Underlying operation |
|---|---|---|---|
| `DL_LAT_FEC` | `send_fec()` | `drone/src/dl_backend_tx.c` | CMD_SET_FEC over tx_cmd UDP |
| `DL_LAT_DPTH` | `send_depth()` | `drone/src/dl_backend_tx.c` | CMD_SET_INTERLEAVE_DEPTH |
| `DL_LAT_RADIO` | `send_radio()` | `drone/src/dl_backend_tx.c` | CMD_SET_RADIO |
| `DL_LAT_TXPWR` | `run_iw()` | `drone/src/dl_backend_radio.c` | posix_spawnp `iw … set txpower` + waitpid |
| `DL_LAT_ENC` | `apply_set()` | `drone/src/dl_backend_enc.c` | HTTP GET `/api/v1/set?video0.bitrate=…&…` |
| `DL_LAT_IDR` | `dl_backend_enc_request_idr()` | `drone/src/dl_backend_enc.c` | HTTP GET `/request/idr` |

Latency boundary: **wall-clock around the entire call site** (matches
what the operator perceives as a stall). For tx_cmd this is the full
`send_and_recv` body, including `drain_pending` and the
req_id-mismatch retry loop. For radio it is `posix_spawnp` through
`waitpid`. For encoder it is the `http_get(...)` call (URL composition
excluded — microseconds).

**IDR throttle exclusion:** when `dl_backend_enc_request_idr` returns
early because `(now - last_idr_ms) < min_idr_interval_ms`, no sample
is recorded. A throttled IDR isn't a real call.

All invocations of each helper count, including the safe-defaults
path (`_apply_safe`). Latency is a property of the call, not the
caller; watchdog-time stalls are exactly when this telemetry matters
most.

## Module: `dl_latency`

New files: `drone/src/dl_latency.c`, `drone/src/dl_latency.h`. Owns
the six ring buffers and counters in a single process-static struct;
no heap allocations after init.

### Public API

```c
typedef enum {
    DL_LAT_FEC = 0,
    DL_LAT_DPTH,
    DL_LAT_RADIO,
    DL_LAT_TXPWR,
    DL_LAT_ENC,
    DL_LAT_IDR,
    DL_LAT__COUNT,
} dl_lat_call_t;

/* Opaque; carries the start timestamp and the call id so `end` can't
 * accidentally desync from `begin`. */
typedef struct {
    uint64_t      t_ns;
    dl_lat_call_t which;
} dl_lat_handle_t;

/* Called once from dl_applier.c during init, before any backend opens. */
void dl_latency_init(void);

dl_lat_handle_t dl_latency_begin(dl_lat_call_t which);
void dl_latency_end(dl_lat_handle_t h, int rc);

/* Renders all six lines into `out`, including their msposd `&L<row>&F30 `
 * prefixes and trailing newlines (so the buffer can be written verbatim
 * by the OSD flush). Returns bytes written (excluding trailing NUL).
 * Caller decides whether to render at all based on cfg->osd_debug_latency. */
size_t dl_latency_render(char *out, size_t out_len);
```

### Internal state

```c
#define DL_LAT_WINDOW 128

struct dl_lat_slot {
    uint16_t samples_ms[DL_LAT_WINDOW];
    uint8_t  head;       /* next write index */
    uint8_t  filled;     /* min(count_pushed, DL_LAT_WINDOW) */
    uint64_t n_total;    /* cumulative call count */
    uint64_t n_err;      /* cumulative error count */
    uint16_t max_ms;     /* max since boot */
};

static struct dl_lat_slot g_slots[DL_LAT__COUNT];
```

### Behavior

- Clock: `clock_gettime(CLOCK_MONOTONIC)`.
- Sample value: `ms = (t_end_ns - t_start_ns + 500000) / 1000000`,
  clamped to `UINT16_MAX` so a >65 s hang doesn't wrap.
- `n_total` increments on every `dl_latency_end`. `n_err` increments
  only when `rc != 0`.
- `max_ms` is sticky across the lifetime of the process (never
  decremented). Persists across watchdog and safe-defaults episodes.
- Percentile computation: stack-local `uint16_t tmp[DL_LAT_WINDOW]`
  copy of `samples_ms[0..filled]`, `qsort`, then
  `p50 = tmp[filled/2]`, `p95 = tmp[(filled*95)/100]`. Computed
  on demand at render time (1 Hz); cost is negligible (six 128-element
  sorts per second).
- The module always records, regardless of the config flag. The flag
  only gates *rendering*. Cost when disabled: six pairs of
  `clock_gettime` per second — well below noise.
- Single-threaded; applier is single-threaded by design, no locks.

### Render format

Each line uses the msposd directive prefix `&L<row>&F30 ` for rows
51–56 (just under the existing status line at row 50). Per-line
template:

```
<TAG>  p50=%3u p95=%3u mx=%3u n=%llu e=%llu
```

Tags are fixed-width 5 chars: `FEC  `, `DPTH `, `RADIO`, `TXPWR`,
`ENC  `, `IDR  `. Numbers use `%3u` for ms fields so columns stay
aligned for typical values; `n` and `e` are `%llu` with no padding.

**Empty-slot rendering:** when `filled == 0` (call has never fired
since boot), print `--` in place of each numeric ms field so the
operator doesn't read "p50=0" as "fast call". `n=0 e=0` print normally.

Example rendered block:

```
FEC   p50=  2 p95=  8 mx= 41 n=1247 e=0
DPTH  p50=  3 p95=  9 mx= 22 n=  18 e=0
RADIO p50= 12 p95= 28 mx=134 n=  31 e=1
TXPWR p50= 14 p95= 35 mx=210 n=  31 e=0
ENC   p50= 25 p95= 60 mx=312 n= 412 e=2
IDR   p50= 20 p95= 55 mx=180 n=   3 e=0
```

## Call-site integration

The wrappers are applied at the **per-call helper** level, not at the
public `dl_backend_*_apply` boundary. Reasons:

- `dl_backend_tx_apply` issues 0–3 sub-commands per tick depending on
  diff; timing at that level can't separate FEC / DEPTH / RADIO.
- `dl_backend_enc_apply` includes a `bitrate_kbps == 0` skip path
  that would otherwise pollute samples as "zero-latency calls".

Insertion pattern (identical at every site):

```c
dl_lat_handle_t h = dl_latency_begin(DL_LAT_<X>);
int rc = /* existing body */;
dl_latency_end(h, rc);
return rc;
```

Sites:

- `send_fec` / `send_depth` / `send_radio` in `dl_backend_tx.c`
  wrap their existing `send_and_recv(...)` calls.
- `run_iw` in `dl_backend_radio.c` wraps from `posix_spawnp` through
  `waitpid`.
- `apply_set` in `dl_backend_enc.c` wraps its `http_get(...)` call.
- `dl_backend_enc_request_idr` in `dl_backend_enc.c` wraps its
  `http_get(...)` call only on the non-throttled path (the early
  return on throttle records nothing).

`dl-inject` (separate binary) does not link `dl_latency`; injected
traffic doesn't pollute the drone's stats.

## Encoder error-contract change

`drone/src/dl_backend_enc.c:177` currently returns `0` for non-2xx
HTTP replies, with an explicit comment defending the choice. **This
design overrides that decision** so non-2xx counts as an error for the
`ENC` line's `e` column. Concretely:

```c
/* old */ return 0;   /* line 177, non-2xx branch */
/* new */ return -1;
```

The surrounding comment at `dl_backend_enc.c:153–158` is updated to
explain the new contract: completed-but-non-2xx now propagates as an
error; `ENC_RESPONSE_BAD` continues to capture the response body for
the SD log.

**Unchanged: silent-encoder path.** When `http_get` receives zero
bytes back (lines 138–147), it still returns `0`. Only "got a reply,
status was 4xx/5xx" becomes `-1`.

### Ripple analysis

`drc < 0` from any backend apply in `dl_applier.c` triggers three
things in both the steady-apply path (lines 421–430) and the
phase-2 apply path (lines 500–509):

1. `dl_log_warn("apply: at least one backend failed seq=%u", …)`.
2. `dl_mavlink_emit(mav, "apply_fail", WARNING, "DL APPLY_FAIL backend")`
   — STATUSTEXT to the GS.
3. `dl_dbg_emit("APPLY_FAIL", DL_DBG_SEV_WARN, …)` — SD log.

No watchdog interaction, no retry, no other state change.

**Behavioral consequence:** the GS will start seeing `APPLY_FAIL backend`
STATUSTEXTs when the encoder returns 4xx/5xx, where today it is silent
on MAVLink. This is a deliberate change — the original code's comment
cited "the noisier MAVLink apply_fail path" as the reason to swallow
non-2xx. With the debug OSD landing, we want that signal surfaced.

## Config

One new key in `conf/drone.conf.sample` and the parser:

```ini
# Debug OSD: per-apply-call latency stats (p50/p95/max/n/err) for the
# six parameter-change paths. Six extra lines below the status line.
# Off by default; flip to true on a single drone for diagnosis.
osd_debug_latency = false
```

`drone/src/dl_config.h` gains one field next to existing OSD knobs:

```c
bool osd_debug_latency;
```

`drone/src/dl_config.c` changes mirror `osd_enable` exactly:

1. Default: `cfg->osd_debug_latency = false;`.
2. Parser: `else if (strcmp(key, "osd_debug_latency") == 0) SET_BOOL(osd_debug_latency);`.

No other knobs added. `N=128` is a `#define` in `dl_latency.c`.
Refresh cadence reuses `osd_update_interval_ms`. There is no GS-side
config change.

**Interaction with `osd_enable`:** when `osd_debug_latency=true` and
`osd_enable=false`, nothing visible happens — the OSD file isn't
written at all. That's correct behavior; the sample comment makes the
dependency explicit.

## OSD writer changes

`drone/src/dl_osd.c` and `drone/src/dl_osd.h`:

`struct dl_osd` gains two fields:

```c
bool debug_latency_enabled;        /* from cfg->osd_debug_latency */
char debug_block[512];             /* rendered by dl_latency_render */
```

`dl_osd_open()` copies `cfg->osd_debug_latency` into the new bool.

`flush()` write order becomes:

```
event_line   (if non-empty)
status_line
debug_block  (if non-empty)
```

`dl_osd_write_status()` (called once per OSD tick) is the single
place that refreshes the debug block:

```c
if (o->debug_latency_enabled) {
    dl_latency_render(o->debug_block, sizeof(o->debug_block));
} else {
    o->debug_block[0] = '\0';
}
```

This piggybacks on the existing 1 Hz tick and the existing atomic
tmp+rename write. No new timer, no new file.

`dl_osd_write_event()` keeps its current behavior — events render on
top, status and debug below.

## Testing

### C unit tests (`tests/drone/test_dl_latency.c`)

New file. Each test registered via the existing `DL_TEST(name)`
macro. Helper: `dl_latency_record_raw(which, ms, rc)` exposed under
`#ifdef DL_TESTING` so tests can inject samples without spinning real
syscalls.

| Test | Invariant |
|---|---|
| `latency_empty_renders_dashes` | Fresh init; render shows `p50= -- p95= -- mx= --`, `n=0 e=0` on every line. |
| `latency_records_single_sample` | One `begin`/`end` with rc=0; `n=1 e=0`, p50 == recorded ms. |
| `latency_records_error` | `begin`/`end` with rc=-1; `n=1 e=1`, latency still recorded. |
| `latency_ring_wraps_at_128` | 200 distinct samples; `filled == 128`, p50/p95 over the last 128. |
| `latency_max_is_sticky` | 500 ms sample then 128 small samples; `mx == 500` after the big one rolls out of the percentile window. |
| `latency_percentiles_correct` | Samples 1..100; p50 ≈ 50, p95 ≈ 95. |
| `latency_clamps_giant_value` | Force `t_end - t_start` exceeding `UINT16_MAX` ms; stored sample = `UINT16_MAX`, no UB. |
| `latency_render_fits_buffer` | Worst-case 9-digit `n`/`e` renders into 512 B; output null-terminated, no truncation. |

### Python e2e (`tests/test_drone_e2e.py`)

Four new tests reusing the existing `_sandbox` context manager that
runs a real `dl-applier` against mock backends.

| Test | What it asserts |
|---|---|
| `test_osd_debug_latency_disabled_default` | Default config; OSD file contains no `FEC `/`RADIO `/`TXPWR ` tokens. |
| `test_osd_debug_latency_enabled_renders_six_lines` | `osd_debug_latency=true`; after a few applies, OSD file contains all six tags. |
| `test_osd_debug_latency_error_counter_increments` | Mock encoder returns HTTP 500; after one ENC apply, `e=1` appears on the `ENC ` line. Exercises the encoder contract flip end-to-end. |
| `test_osd_debug_latency_idr_throttle_not_counted` | Two IDR triggers inside `min_idr_interval_ms`; `IDR ` line shows `n=1` (throttled second call not recorded). |

### Commands

Both layers run via the canonical commands documented in `CLAUDE.md`:

```
python3 -m pytest --ignore=tests/test_mavlink_status.py
make -C drone test
```

## Out of scope

- GS-side display of latency stats. (Would require a wire-format
  bump.)
- Runtime flag toggle without applier restart (no SIGHUP reload
  today).
- Configurable window size.
- Latency histograms beyond p50/p95/max.
- Per-error-kind buckets (errno vs HTTP status vs timeout). The SD
  log via `dl_dbg_emit` already differentiates; the OSD line is for
  triage, not forensics.
