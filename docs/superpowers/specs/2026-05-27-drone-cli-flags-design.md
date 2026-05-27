# Drone applier — per-field CLI flags

**Status:** design accepted, awaiting implementation plan.
**Date:** 2026-05-27.
**Touches:** `drone/src/dl_applier.c`, `drone/src/dl_config.{c,h}`,
`conf/drone.conf.sample`, `README.md`, `tests/drone/`, `tests/`.

## Problem

`dl-applier` currently accepts only `--config <path>` and `--debug`.
Every other operational knob is in `drone.conf`, which means even a
one-line override (e.g. raise `safe_mcs` for a single test flight)
requires editing the file on the drone. We want every relevant conf
field to be settable from the command line, while still supporting
the file as the durable, audited config surface.

## Goals

- Every conf field in scope can be set with `--<kebab-name> <value>`.
- Precedence is unambiguous: built-in defaults → conf file → CLI.
- The conf file is still supported and unchanged in format.
- `--config` becomes optional so defaults + CLI alone are usable.
- Range/format validation is identical between the conf parser and
  the CLI parser — one source of truth.
- `--help` enumerates every flag with its type and range, generated
  from the same table the parsers consult.

## Non-goals

- No env-var layer (`DRONE_*`). Three layers are enough.
- No alternative config format (JSON/TOML/YAML).
- `dl-inject` is not touched.
- `--debug` (log verbosity) is **not** moved into the conf or
  renamed. It must take effect before conf parsing, which is its own
  lifecycle. It stays a CLI-only switch.

## Decisions

1. **Layering:** built-in defaults → conf file (if `--config` given)
   → CLI overrides. CLI wins. Unset CLI flags do not clobber
   conf-file values — only fields the user explicitly passed get
   applied.
2. **Flag naming:** kebab-case. Conf key `mavlink_enable` becomes
   CLI flag `--mavlink-enable`. The mapping is mechanical (swap `_`
   ↔ `-`) so a single table backs both names.
3. **Booleans:** value-taking via `--name` (bare, = true) or
   `--name=true|false|1|0` (case-insensitive; `yes/no/on/off` also
   accepted). The `=` form is required for explicit values; a
   space-separated value is parsed as a positional. Updated
   2026-05-27 — see `docs/superpowers/specs/2026-05-27-bool-cli-args-design.md`.
4. **Scope:** every field in `dl_config_t` **except** the Phase-3
   debug-suite block, namely:
   - `debug_enable`
   - `dbg_log_enable`
   - `dbg_log_dir`
   - `dbg_max_bytes`
   - `dbg_fsync_each`

   `gs_tunnel_addr` / `gs_tunnel_port` **stay in scope** even though
   they live next to the debug suite in the sample conf — the DLHE
   handshake uses them in production.
5. **Approach:** Option A from brainstorming. Hand-written
   `getopt_long` table in `dl_applier.c`, backed by a small private
   data table inside `dl_config.c` that also drives the conf
   parser. No X-macros, no codegen.

## Architecture

### Lifecycle

`main()` in `drone/src/dl_applier.c`:

```
parse_args(argc, argv, &args)
    args.config_path    : const char *, may be NULL (was required)
    args.log_debug      : bool (existing)
    args.overrides      : dl_config_t  — fields the user passed
    args.set_int[]      : bit array per int field, "did the user pass it"
    args.set_bool[]     : bit array per bool field
    args.set_str[]      : bit array per str field
dl_log_init(...)                        # honors --debug
dl_config_defaults(&cfg)
if (args.config_path)
    dl_config_load(args.config_path, &cfg)   # may fail → fatal
dl_config_apply_overrides(&cfg, &args)       # CLI fields stamped in
dl_config_validate(&cfg)                     # cross-field invariants
...rest of boot unchanged...
```

If `--config` is not given, `dl_config_load` is skipped and `cfg`
holds the built-in defaults before the override stamp.

If `--config` is given but `dl_config_load` returns non-zero, boot
is still fatal — same behavior as today.

### Shared validation helpers

Three small parsing helpers move into `dl_config.c` (file-static if
private suffices, otherwise exported via `dl_config.h`):

```c
int dl_parse_long_ranged(const char *s, long lo, long hi, long *out);
int dl_parse_bool(const char *s, bool *out);   /* used by SET_BOOL path */
```

`dl_config_load`'s existing `SET_INT_RANGED` / `SET_BOOL` macros are
rewritten as thin shims that log with conf-file context
(`path:lineno`). The CLI parser calls the same helpers and logs with
flag context (`--flag-name`). Identical numeric ranges live in one
place.

### Field table (single source of truth)

In `dl_config.c`:

```c
typedef enum { DL_F_U8, DL_F_I8, DL_F_U16, DL_F_U32 } dl_field_type_t;

typedef struct {
    const char     *name;       /* exact conf key; CLI flag is the
                                   same string with _ → - swap */
    size_t          offset;     /* offsetof(dl_config_t, …) */
    dl_field_type_t type;
    long            lo, hi;     /* inclusive */
} dl_int_field_t;

typedef struct { const char *name; size_t offset; } dl_bool_field_t;
typedef struct { const char *name; size_t offset; } dl_str_field_t;

static const dl_int_field_t  DL_INT_FIELDS[]  = { /* one row per int field */ };
static const dl_bool_field_t DL_BOOL_FIELDS[] = { /* one row per bool field */ };
static const dl_str_field_t  DL_STR_FIELDS[]  = { /* one row per string field */ };
```

Exported API for the CLI parser:

```c
/* setter-by-name returns 0 on success, -1 on parse/range failure
 * (caller logs with its own context). */
int dl_config_set_int_by_name (dl_config_t *cfg, const char *name, const char *val);
int dl_config_set_bool_by_name(dl_config_t *cfg, const char *name, bool        val);
int dl_config_set_str_by_name (dl_config_t *cfg, const char *name, const char *val);

/* iteration for building the getopt_long option table dynamically. */
const dl_int_field_t  *dl_config_int_fields (size_t *n_out);
const dl_bool_field_t *dl_config_bool_fields(size_t *n_out);
const dl_str_field_t  *dl_config_str_fields (size_t *n_out);

/* cross-field invariants (e.g. roi_qp_threshold > roi_qp_low_anchor).
 * Called from both conf-file load path and CLI override path. */
int dl_config_validate(const dl_config_t *cfg);
```

`dl_config_load` is refactored to dispatch via these tables:

- Strip the comment and `=`, trim, get `key` and `val`.
- Try the legacy-removed-key stanza first
  (`video_k_min` etc. → loud error, same as today).
- Then look up `key` in `DL_INT_FIELDS`, `DL_BOOL_FIELDS`,
  `DL_STR_FIELDS` (linear scan; ~50 entries, called once at boot).
- On hit, call the matching `dl_config_set_*_by_name`. On miss, log
  `unknown key` (unchanged).
- After the loop, call `dl_config_validate`.

### CLI parser in `dl_applier.c`

`parse_args` now builds its `struct option opts[]` dynamically:

```
opts = [
    { "config", required_argument, 0, 'c' },
    { "debug",  no_argument,       0, 'd' },
    { "help",   no_argument,       0, 'h' },
    /* one entry per int field   — required_argument */
    /* one entry per bool field  — no_argument */
    /* one entry per str field   — required_argument */
    { 0 }
]
```

Each generated entry's long-name is the field's table name with
`_` → `-`. Numeric short codes for these entries are out of the
ASCII range (e.g. `256 + index`) so they don't collide with `c`,
`d`, `h`. `getopt_long` returns the long-option's `val`, which the
loop maps back into a `(category, table-index)` pair and dispatches
to the right setter against `args.overrides`, while marking the
matching `args.set_*[]` bit.

`dl_config_apply_overrides(cfg, args)` then walks the three set bit
arrays and, for each bit that's set, copies the field from
`args.overrides` into `cfg`.

### `--help` output

Generated from the tables at runtime, grouped by category, sorted
by name within each group. For int fields, show `(lo..hi)`. For
bool fields, show `(switch — sets to true)`. For str fields, show
`(string, max N chars)`. The hand-written usage block keeps
`--config`, `--debug`, `--help` at the top.

### Cross-field invariants

The `roi_qp_threshold_kbps > roi_qp_low_anchor_kbps` check today
lives inline in `dl_config_load`. It moves into `dl_config_validate`
so it runs after both the conf load *and* the CLI overrides — the
CLI can break the invariant just like the conf can. Validate failure
is fatal at boot (same behavior as today's conf path).

## Error handling

- **Unknown flag.** `getopt_long` prints `unrecognized option`,
  `parse_args` returns non-zero, `main` exits 2. Unchanged.
- **Bad value (out of range / non-numeric / bad bool).**
  `dl_log_err("--%s: %s out of range [%ld..%ld]", …)` or similar,
  exit 2. The shared helpers carry the same range constants as the
  conf parser, so the error message is consistent.
- **`--config` path missing/unreadable.** Today's behavior — error
  logged by `dl_config_load`, exit 3. Unchanged.
- **`--config` parses with errors.** Today's behavior — fatal.
  Unchanged. We do not reach the CLI override step in that case.
- **`--config` omitted.** New behavior. Allowed; we boot from
  defaults + CLI.
- **String length overflow.** `dl_config_set_str_by_name` rejects
  values longer than `DL_CONF_MAX_STR - 1` and returns -1 with a
  log line. CLI parse exits 2.
- **Cross-field invariant violated** (e.g. CLI sets
  `--roi-qp-threshold-kbps 1000 --roi-qp-low-anchor-kbps 2000`).
  `dl_config_validate` logs and returns -1; boot fatal.

## Sample conf and docs

- `conf/drone.conf.sample` — **no structural change.** It remains
  the source of truth for human-readable defaults and explanations.
  No comments saying "also a CLI flag"; the README covers that.
- `README.md` — new short subsection "CLI overrides" near the
  existing dl-applier invocation example:
  - State the precedence (defaults → conf → CLI).
  - Point readers at `dl-applier --help` for the full flag list.
  - One worked example, e.g.
    `dl-applier --config /etc/dynamic-link/drone.conf --safe-mcs 3 --mavlink-port 14570`.
  - State that the Phase-3 debug-suite fields have no CLI flag.

## Tests

### C unit tests — `tests/drone/test_config_cli.c`

Run via `make -C drone test`. New `DL_TEST(...)` cases:

- **Field tables are non-empty** and every name resolves through
  the corresponding `dl_config_set_*_by_name` (sanity: name + offset
  match a real field).
- **Int range enforcement:** for one representative field per
  integer type (`safe_mcs` for u8, `safe_tx_power_dBm` for i8,
  `mavlink_port` for u16, `min_idr_interval_ms` for u32), assert
  rejection at `lo - 1` and `hi + 1`, acceptance at `lo` and `hi`.
- **Bool parser:** `dl_config_set_bool_by_name(cfg, "mavlink_enable",
  true)` flips the field. The CLI surface now passes either `true`
  or `false` (see `docs/superpowers/specs/2026-05-27-bool-cli-args-design.md`);
  the helper itself was always symmetric.
- **String overflow:** value longer than `DL_CONF_MAX_STR - 1` is
  rejected.
- **`dl_config_validate`** catches `roi_qp_threshold_kbps ≤
  roi_qp_low_anchor_kbps` regardless of how the fields got there
  (set by hand to simulate either path).
- **Conf parser still works** end-to-end via the refactored table
  path (load `conf/drone.conf.sample`, assert a few sentinel
  fields match the documented defaults).

### Python e2e — `tests/test_drone_cli_overrides.py`

Run via `python3 -m pytest …`. Uses the existing `_sandbox` helper
from `tests/test_drone_e2e.py`:

- **CLI override on top of conf file.** Spawn
  `dl-applier --config conf/drone.conf.sample --safe-mcs 4 --mavlink-port 14570`,
  trip the watchdog, and verify the safe_defaults push uses MCS=4
  (visible via the mock wfb_tx control sink). Verify the mavlink
  status sink got bound on the overridden port.
- **No `--config`, defaults only.** Spawn `dl-applier --safe-mcs 4`
  with no `--config`. Trip the watchdog, verify MCS=4 and that all
  other fields match the documented defaults.
- **Bad CLI value exits non-zero.** Spawn `dl-applier --safe-mcs 9`
  (above u8 range 0..7), assert non-zero exit and a recognizable
  error line on stderr.
- **`--help` lists every in-scope field.** Run `dl-applier --help`,
  assert the output contains a flag for each field in scope and
  does *not* contain a flag for the excluded Phase-3 debug-suite
  fields.

## Implementation notes

- Keep the field tables in declaration order matching `dl_config_t`
  for readability; the linear-scan lookup is fine at boot frequency.
- Use `offsetof` plus a `char *` cast for the typed writes; no UB.
- Bit array sizing: ceil(count/8). Use a small inline helper
  (`bit_set`, `bit_test`) — no new dependency.
- The numeric short codes for generated long options must not
  collide with the existing `c`, `d`, `h`. Start at 0x100 so
  `getopt_long`'s `int` return value carries them cleanly.
- Linker/symbol surface: the new exported helpers are the only
  additions to `dl_config.h`. The internal table types stay
  private to `dl_config.c`; only the iteration accessors are
  exported.

## Acceptance criteria

- `dl-applier --help` lists every in-scope field with its range.
- `dl-applier --config sample.conf --safe-mcs 3` boots, applies
  safe_mcs=3 even though the conf file says otherwise.
- `dl-applier --safe-mcs 3` boots without `--config`, applies
  defaults + safe_mcs=3.
- All existing tests pass unmodified.
- New tests above pass.
- Phase-3 debug-suite fields are intentionally absent from `--help`
  and rejected as `unrecognized option` if passed.
