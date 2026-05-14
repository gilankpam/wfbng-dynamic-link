# Waybeam encoder support

Status: design
Owner: Gilang
Date: 2026-05-14

## Summary

Add support for the **waybeam** encoder as a peer of **majestic** in the
drone-side handshake path. The HTTP control API (`/api/v1/set?...`,
`/request/idr`) is identical between the two encoders, so the existing
`dl_backend_enc` is already protocol-compatible. The one real
difference is the source of truth for the boot-time `video0.fps`
value that the drone reports to the GS via the DLHE handshake:

- majestic: `/etc/majestic.yaml`, key `video0.fps` (YAML).
- waybeam: `/etc/waybeam.json`, key `video0.fps` (JSON).

Goals:

- `encoder_kind = waybeam` becomes a fully-supported runtime setting,
  not just a label.
- Mirror the YAML-path semantics exactly: read once at applier
  start; failure to read is a soft fail (DLHE never sent, GS stays
  in safe_defaults), consistent with today's behavior on a missing
  `/etc/majestic.yaml`.
- No new runtime dependencies on the drone image; no new test-host
  dependencies.

Non-goals:

- A general-purpose JSON parser. We mirror `dl_yaml_get`'s
  philosophy: robust enough for the one shape we read, not more.
- Live re-reads of `video0.fps`. The handshake reads it once; an
  in-flight encoder fps change requires an applier restart, same as
  today.
- HTTP fetch of `video0.fps` from waybeam's `/api/v1/get?video0.fps`.
  The local config file is cheaper, doesn't depend on the encoder
  process being up at applier boot, and matches the existing
  read-from-disk pattern.
- GS-side awareness of the encoder kind. The GS consumes `fps` from
  DLHE; the source on the drone is opaque to it.

## Background

Today, `dl_hello_init` (drone/src/dl_hello.c:49-56) hardcodes a YAML
read against `cfg->hello_majestic_yaml_path` to obtain `video0.fps`.
For drones running the waybeam encoder there is no such YAML — the
encoder's config lives at `/etc/waybeam.json`. Without a way to read
fps for waybeam, the DLHE handshake never completes and the GS sits
in `safe_defaults` indefinitely.

The `encoder_kind` config field already exists (drone/src/dl_config.h:67),
is documented in `conf/drone.conf.sample:91` as
`waybeam | majestic | none`, and is parsed correctly — but it's only
referenced in a single log line in `dl_backend_enc.c`. The control-API
HTTP calls in `dl_backend_enc` (`/api/v1/set?video0.bitrate=...&fpv.roiQp=...&video0.fps=...`
and `/request/idr`) are byte-identical for both encoders per
`docs/encoder_api.md`, so no backend dispatch is needed there.

## Configuration

### Drone (`conf/drone.conf.sample`)

The existing `encoder_kind` line is unchanged. We add one new path
field alongside `hello_majestic_yaml_path` and update the surrounding
comment:

```ini
# --- P4a: drone→GS config handshake ----------------------------------
# Authoritative sources for MTU and FPS, read once at startup.
# MTU comes from /etc/wfb.yaml `wireless.mlink` (typically 1400 or
# 3994 depending on wfb-ng's radio_mtu setting).
#
# FPS source depends on `encoder_kind`:
#   - majestic → hello_majestic_yaml_path (YAML, `video0.fps`)
#   - waybeam  → hello_waybeam_json_path  (JSON, `video0.fps`)
hello_wfb_yaml_path      = /etc/wfb.yaml
hello_majestic_yaml_path = /etc/majestic.yaml
hello_waybeam_json_path  = /etc/waybeam.json
```

- New field in `dl_config_t`: `char hello_waybeam_json_path[DL_CONF_MAX_STR];`.
- Default `/etc/waybeam.json`.
- Parsed in `dl_config.c` via the same `SET_STR` pattern used for
  `hello_majestic_yaml_path`.

### GS (`conf/gs.yaml.sample`)

No change. The GS does not learn about the encoder kind and does
not need to.

## Wire format

**Unchanged.** The DLHE packet carries `fps` as `uint16_t`; the
drone fills it from whichever file the dispatch picks. No bit
allocation, no version bump, no contract-test fixture changes.

## Drone-side changes

### New module: `drone/src/dl_json_get.{h,c}`

Mirror of `dl_yaml_get.h`:

```c
/* dl_json_get.h — tiny line-scan helper for finding `"<key>":<int>`
 * inside a top-level `"<block>":{ ... }` object. Robust enough for
 * the one specific JSON shape we read (/etc/waybeam.json). Not a
 * general JSON parser. */
#pragma once

/* Find `"<key>":<int>` inside a top-level `"<block>":{ ... }` object
 * in `path`. On success, writes the parsed integer into *out and
 * returns 0. On failure returns a negative errno-ish code: negative
 * errno on open failure (typically -ENOENT), -EINVAL for any
 * structural / parsing problem (block not found, key not found
 * inside block, non-integer value, etc.). *out is only written on
 * success. */
int dl_json_get_int(const char *path,
                    const char *block,
                    const char *key,
                    int *out);
```

Implementation outline (~40 LOC):

1. Read file into a heap buffer; cap at 64 KB (sample file is ~2 KB).
   Reject larger files with `-EINVAL` to bound memory.
2. Find the substring `"<block>"` followed by optional whitespace,
   `:`, optional whitespace, `{`. This anchors uniquely on the
   waybeam.json shape (block names are top-level keys).
3. From immediately after that `{`, walk forward keeping a brace
   counter; track string state (inside `"…"`, with `\` escapes)
   so braces inside string values don't perturb the counter.
4. While at depth 1, look for the literal `"<key>"` followed by
   optional whitespace, `:`, optional whitespace, then decimal
   digits (with optional leading `-`).
5. Parse digits into `int` (use `strtol` with bounds check). Stop
   at first non-digit; reject empty or malformed.
6. Return `-EINVAL` if we hit the closing `}` for the block without
   finding the key, or if the value is non-numeric.

**Why anchor on the block:** waybeam.json contains
`record.fps = 0` in addition to `video0.fps = 60`. A naive scan for
`"fps"` would match the wrong one depending on ordering. Anchoring
on the block name (`"video0"` vs `"record"`) is the same defense
`dl_yaml_get` uses for `wireless` vs `video0` in YAML.

**Why not jsonfilter:** symmetric reasoning to `dl_yaml_get`. We
chose an inline scanner for YAML because it (a) avoids a runtime
fork/exec, (b) lets `make -C drone test` run on a workstation with
no extra tooling, and (c) handles the one specific shape we
actually read. The same logic applies here.

### `drone/src/dl_hello.c`

Replace lines 49-56 with a dispatch on `cfg->encoder_kind`:

```c
int fps = 0;
if (strcmp(cfg->encoder_kind, "majestic") == 0) {
    rc = dl_yaml_get_int(cfg->hello_majestic_yaml_path,
                         "video0", "fps", &fps);
    if (rc != 0) {
        dl_log_err("dl_hello: failed to read fps (%s video0.fps): %d",
                   cfg->hello_majestic_yaml_path, rc);
        return -1;
    }
} else if (strcmp(cfg->encoder_kind, "waybeam") == 0) {
    rc = dl_json_get_int(cfg->hello_waybeam_json_path,
                         "video0", "fps", &fps);
    if (rc != 0) {
        dl_log_err("dl_hello: failed to read fps (%s video0.fps): %d",
                   cfg->hello_waybeam_json_path, rc);
        return -1;
    }
} else {
    dl_log_err("dl_hello: unknown encoder_kind for fps source: %s",
               cfg->encoder_kind);
    return -1;
}
```

Range checks (`fps <= 0 || fps > 0xFFFF`) and the assignment to
`h->fps` are unchanged.

### `drone/src/dl_config.{h,c}`

- Add `char hello_waybeam_json_path[DL_CONF_MAX_STR];` to
  `dl_config_t`, immediately after `hello_majestic_yaml_path`.
- In `dl_config_init_defaults` (around dl_config.c:64), copy
  `"/etc/waybeam.json"` into the new field.
- In the parser table, add an `else if (strcmp(key, "hello_waybeam_json_path") == 0) SET_STR(hello_waybeam_json_path);`
  next to the existing majestic-path branch.

### `drone/src/dl_backend_enc.c`

**No change.** SET and IDR HTTP paths are identical for both
encoders. The existing log line at line 183 already prints
`cfg->encoder_kind`, so a waybeam applier will log
`"enc: waybeam at 127.0.0.1:80"` for free.

### `drone/src/Makefile`

Add `dl_json_get.o` to the dl-applier object list (and to the unit
test target's object list).

## Behavior summary

| Scenario                              | Boot FPS source                                    | DLHE handshake |
|---------------------------------------|----------------------------------------------------|----------------|
| `encoder_kind = majestic` (default)   | `/etc/majestic.yaml` `video0.fps` (YAML)           | sent           |
| `encoder_kind = waybeam`              | `/etc/waybeam.json` `video0.fps` (JSON)            | sent           |
| `encoder_kind = none` or unknown      | n/a — `dl_hello_init` returns -1                   | never sent     |
| File missing / unreadable / malformed | -1 from helper → `dl_hello_init` returns -1        | never sent     |

In every failure case the applier continues to run; `dl-inject` and
manual decisions still apply. Only the adaptive control loop is
inert until the operator fixes the source and restarts. This is
identical to today's majestic-missing behavior.

## Failure modes

| Misconfiguration                                              | Symptom |
|---------------------------------------------------------------|---------|
| `encoder_kind = waybeam`, `/etc/waybeam.json` missing         | `dl_hello: failed to read fps (...): -ENOENT` at startup; no DLHE; GS stays in safe_defaults. |
| `encoder_kind = waybeam`, `video0.fps` key missing or non-int | `dl_hello: failed to read fps (...): -EINVAL`; same downstream effect. |
| `encoder_kind = majestic` but drone runs waybeam              | YAML read fails (no majestic.yaml). Same loud failure. |
| `encoder_kind = waybeam` but drone runs majestic              | JSON read fails (no waybeam.json), or — if the operator has both files — fps is read truthfully and runtime behaves correctly; no harm. |
| `encoder_kind = waybeam`, waybeam.json present but contains both `video0.fps` and `record.fps` | Returns `video0.fps` (the intended value). Pinned by the block-anchoring unit test. |

## Tests

### C (`tests/drone/`)

- **`tests/drone/test_dl_json.c` (new)** — unit tests for
  `dl_json_get_int`. Each test writes a temp file to
  `tmpfile()`/`mkstemp`, calls the helper, asserts the result.
  - `happy_path_video0_fps` — full sample fixture (the one from
    the design discussion) → returns 60.
  - `block_collision_video0_vs_record` — same fixture, asks for
    `record.fps` → returns 0 (truthful value from `record`
    block, not video0's 60); asks for `video0.fps` → 60. Pins the
    block-anchoring invariant.
  - `missing_file` → `-ENOENT`.
  - `block_not_found` (`bogus`) → `-EINVAL`.
  - `key_not_found` (`xyz` inside `video0`) → `-EINVAL`.
  - `non_integer_value` (`"size": "1920x1080"` if asked for `size`)
    → `-EINVAL`.
  - `whitespace_variations` — compact-JSON (no whitespace) variant
    of the sample → still returns 60.
- **`tests/drone/test_dl_hello.c`** — extend with a waybeam-path
  test: write a temp config file with `encoder_kind = waybeam` and
  `hello_waybeam_json_path = <temp>`, write a small waybeam.json
  fixture (`{"video0":{"fps":60}}` is enough), call
  `dl_hello_init`, assert it returns 0 and `h.fps == 60` and
  `h.state == DL_HELLO_STATE_ANNOUNCING`. Also add a negative case
  with an unknown `encoder_kind` value → returns -1.
- **`tests/drone/test_config.c`** — already tests
  `encoder_kind = waybeam`; add an assertion that
  `c.hello_waybeam_json_path` defaults to `"/etc/waybeam.json"` and
  that an explicit `hello_waybeam_json_path = /tmp/foo.json` line
  is parsed into the field.

### Python (`tests/`)

No changes. The wire format, GS handshake handler, and policy are
all encoder-agnostic.

## Docs

- **`README.md`** — in the operator prerequisites section, note
  that `encoder_kind = waybeam` requires `/etc/waybeam.json` with
  `video0.fps` populated.
- **`CLAUDE.md`** — update "Operational prerequisites" #4 to read:
  "`/etc/majestic.yaml` must contain `video0.fps: <integer>` (when
  `encoder_kind = majestic`) **or** `/etc/waybeam.json` must
  contain `video0.fps: <integer>` (when `encoder_kind = waybeam`)
  — same constraint either way: missing key means the applier
  refuses to send HELLO and the GS stays in safe_defaults."
- **`docs/encoder_api.md`** — no change; the file already
  documents both encoders as a single API surface.

## Out of scope (future work)

- Live HTTP refresh of `video0.fps` after applier startup. The
  current handshake reads once; if waybeam's fps is changed at
  runtime, an applier restart is required to propagate the new
  value through DLHE. Same constraint as majestic today.
- Encoder-kind auto-detection. Operators set `encoder_kind`
  explicitly; mismatches surface as loud "failed to read fps"
  errors at startup.
- Reading other fields (codec, size, bitrate) from waybeam.json or
  majestic.yaml. The handshake only needs `fps`; everything else
  is controlled by the applier or read at decision time.
