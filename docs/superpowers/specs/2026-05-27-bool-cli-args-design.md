# dl-applier boolean CLI flags: `--flag=true|false|1|0`

Status: approved
Date: 2026-05-27

## Problem

`dl-applier`'s CLI overrides cover every in-scope conf field, but
boolean flags are **on-only**. Every bool option is registered with
`getopt_long`'s `no_argument` and the handler hard-codes `true`
(`drone/src/dl_applier.c:290-291, 317-326`). Consequences:

- `interleaving_supported` defaults to `1` (custom feat/interleaving_uep
  branch). Operators on vanilla wfb-ng need it `0` — and the CLI cannot
  set it. The only path today is editing `drone.conf`.
- Same gap for `osd_enable`, `osd_debug_latency`, `mavlink_enable`
  (all default `true`).

## Goal

Let operators disable a boolean field from the CLI without editing the
conf file, while keeping the existing bare `--flag` form working.

Out of scope: GS-side flags, wire protocol, conf file parsing, the
Phase-3 debug-suite fields (which intentionally have no CLI flag).

## Design

### CLI surface

| Form | Result |
|---|---|
| `--flag` (bare) | set to `true` (back-compat) |
| `--flag=true`, `--flag=1`, `--flag=yes`, `--flag=on` | set to `true` |
| `--flag=false`, `--flag=0`, `--flag=no`, `--flag=off` | set to `false` |
| `--flag=garbage` | parse error, non-zero exit |
| `--flag false` (space) | **`false` is a positional**; flag set to `true` |

The space-form caveat is a `getopt_long(optional_argument)` limitation:
when an option's argument is optional, getopt requires the `=` form;
a space-separated token is treated as a positional, not the value.
Docs will state this explicitly; the trailing positional triggers
getopt's standard "unrecognized argument" error so the mis-use is
not silent.

The accepted value strings come from reusing the existing
`dl_parse_bool` helper (case-insensitive `true/false/1/0/yes/no/on/off`)
so CLI and conf-file behavior match exactly. Docs recommend
`true|false` as the canonical spelling.

### Code changes (all in `drone/src/dl_applier.c`)

1. **Option table** (`parse_args`, ~line 291): bool entries change from
   `no_argument` to `optional_argument`.
2. **Bool case in getopt loop** (~line 317): if `optarg == NULL`, set
   `true`; otherwise call `dl_parse_bool(optarg, &v)` and reject with
   a clear error on failure, then `dl_config_set_bool_by_name(..., v)`.
3. **Help text** (`usage`, ~line 246): change the bool section header
   from "Boolean switches (set to true when passed)" to
   "Boolean fields (--name or --name=true|false; default = true when
   no value)".

### Header change

Expose `dl_parse_bool`: drop the `static` qualifier in
`drone/src/dl_config.c:102` and add a prototype to
`drone/src/dl_config.h`. No behavior change.

### Not touched

- `dl_config.c` parsing logic
- The conf file format
- The wire protocol / `dl_wire.h`
- The GS side
- The `set_bool` bit array
- `apply_cli_overrides`

### Error handling

`--flag=garbage` prints:

```
--flag: bad value 'garbage' (expected true|false|1|0)
```

and `parse_args` returns `-1`, exiting non-zero — mirroring the
int-field error path at `dl_applier.c:308-314`.

## Tests

Extend `tests/test_drone_cli_overrides.py`:

- `--interleaving-supported=false` → drone behaves as vanilla
  (no `set_interleave_depth` emitted) when conf default is true.
- `--interleaving-supported=0` → same as above.
- Bare `--mavlink-enable` still sets `true` (regression cover for
  the existing test at line 178).
- `--osd-enable=garbage` → process exits non-zero with the error
  message.

## Docs

- `README.md` lines 187-189: replace the "on-only switches" paragraph
  with the new behavior, including the `=` caveat.
- `docs/superpowers/specs/2026-05-27-drone-cli-flags-design.md`:
  update §3 ("Booleans") to reflect the new surface.
