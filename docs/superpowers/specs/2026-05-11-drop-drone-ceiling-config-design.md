# Drop drone-side ceiling config; hardcode hardware bounds

**Status:** approved, ready for implementation plan
**Date:** 2026-05-11

## Summary

Replace the seven per-airframe ceiling knobs in `drone.conf`
(`video_k_min`, `video_k_max`, `video_n_max`, `depth_max`, `mcs_max`,
`tx_power_min_dBm`, `tx_power_max_dBm`) with `#define` constants in
`drone/src/dl_ceiling.h` derived from wfb-ng / hardware limits. The
drone applier remains an enforcer — `dl_ceiling_check` still clamps
bad decisions before backends see them — but its bounds are no longer
operator-tunable. `safe_defaults` (watchdog fallback values) stay
configurable per airframe.

## Motivation

The ceiling fields duplicate clamping that the GS policy already does
and create a second source of truth that has to be kept in sync with
the GS config. The bounds that actually matter on the drone are
firmware/hardware invariants (wfb-ng `MAX_INTERLEAVE_DEPTH`, the
`n ≤ 32 when depth > 1` limit in wfb-ng `src/tx.cpp:1183`, MCS index
range for single spatial stream, regulatory/sane TX-power range) — not
per-airframe policy. Making these constants removes the drift surface
without losing the failsafe.

## Non-goals

- Removing `safe_defaults` (`safe_k`, `safe_n`, `safe_depth`,
  `safe_mcs`, `safe_tx_power_dBm`). Different airframes have different
  safe fallback points; these stay in `drone.conf`.
- Removing `dl_ceiling.c` itself. Failsafe 4 (design doc §4) still
  applies; only its configurability goes away.
- Touching the GS policy bounds. GS keeps its own clamping.
- Backwards compatibility for old `drone.conf` files. Detect the
  removed keys and fail-load with a clear message — this project
  deploys manually, drift is worse than a loud error.

## Hardcoded bounds

In `drone/src/dl_ceiling.h`:

| `#define` | Value | Source |
|---|---|---|
| `DL_BOUND_MCS_MAX` | 7 | single spatial stream; design §4.1 policy |
| `DL_BOUND_K_MIN` | 1 | math sanity (k ≥ 1, k ≤ n) |
| `DL_BOUND_N_MAX` | 32 | wfb-ng `src/tx.cpp:1183` (covers `depth > 1`; conservative for `depth = 1`) |
| `DL_BOUND_DEPTH_MAX` | 8 | wfb-ng `MAX_INTERLEAVE_DEPTH` |
| `DL_BOUND_TX_POWER_MIN_DBM` | -10 | matches current parser sanity range |
| `DL_BOUND_TX_POWER_MAX_DBM` | 30 | same; `iw` rejects anything the radio can't do |

The previous "n ≤ 32 only when depth > 1" rule collapses into a flat
`n ≤ 32`. This is strictly more conservative; sample configs use
`n_max = 16` so no behavior change in practice.

## Code changes

### `drone/src/dl_ceiling.h` / `dl_ceiling.c`

- Add the `DL_BOUND_*` `#define` constants above to the header.
- Change the function signature:
  `dl_ceiling_check(const dl_decision_t *d, const dl_config_t *cfg)`
  → `dl_ceiling_check(const dl_decision_t *d)`.
- All comparisons inside `dl_ceiling_check` read `DL_BOUND_*` instead
  of `cfg->video_k_max` etc.

### `drone/src/dl_config.h`

- Remove the seven ceiling fields from `dl_config_t`
  (`video_k_min`, `video_k_max`, `video_n_max`, `depth_max`,
  `mcs_max`, `tx_power_min_dBm`, `tx_power_max_dBm`).
- Update the comment on `dl_config_validate()`: `safe_*` is now
  validated against `DL_BOUND_*` constants, not against sibling
  ceiling fields.

### `drone/src/dl_config.c`

- Drop the seven default assignments in the defaults init.
- Drop the seven `else if (strcmp(key, ...) == 0)` parser branches.
- Add a deprecation arm for each removed key that emits
  `dl_log_err("config: %s is no longer supported (removed 2026-05-11); bounds are now hardcoded", key)`
  and causes the load to fail. Cheaper than silent ignore.
- Replace cross-field validation: instead of
  `safe_k < cfg->video_k_min`, compare against `DL_BOUND_K_MIN` etc.
- Drop the `video_k_min > video_k_max` and
  `tx_power_min > tx_power_max` consistency checks (no longer
  applicable).
- Drop the wfb-ng-specific checks
  (`depth_max > WFB_MAX_INTERLEAVE_DEPTH`,
  `depth_max > 1 && video_n_max > WFB_N_LIMIT_WITH_DEPTH`) — these
  bounds are now `#define` constants set correctly by construction.

### `drone/src/dl_applier.c`

- Update the single `dl_ceiling_check(&d, &cfg)` call site to drop
  the `&cfg` arg.

### `drone/src/dl_inject.c`

No change. `dl_inject` is a wire-encoder CLI; it doesn't touch the
ceiling.

## Tests

### `tests/drone/test_ceiling.c` (rewrite)

Today's tests poke `c.mcs_max = 5` etc. to drive boundary conditions.
The new tests assert against the `#define` constants directly:

- `decision.k = DL_BOUND_K_MIN - 1` → rejected
- `decision.n = DL_BOUND_N_MAX + 1` → rejected
- `decision.depth = DL_BOUND_DEPTH_MAX + 1` → rejected
- `decision.mcs = DL_BOUND_MCS_MAX + 1` → rejected
- `decision.tx_power_dBm = DL_BOUND_TX_POWER_MIN_DBM - 1` → rejected
- `decision.tx_power_dBm = DL_BOUND_TX_POWER_MAX_DBM + 1` → rejected
- A representative valid decision at the constant edges → accepted.

### `tests/drone/test_config.c` (or wherever config-load tests live)

Add a case: `drone.conf` containing any of the seven removed keys
(e.g. `mcs_max = 7`) → `dl_config_load` returns failure and the log
contains the deprecation message. One key is enough; all seven take
the same path.

### `tests/test_drone_e2e.py`

Lines 246–248: remove the seven keys from the synthetic `drone.conf`
the sandbox writes.

### `tests/test_phase3_e2e.py`

Lines 111–113: same removal.

## Docs

### `conf/drone.conf.sample`

Remove the ceiling block. Add a one-line comment in its place:

```
# Bounds are hardcoded in dl_ceiling.h (wfb-ng + hardware limits).
# Per-airframe tuning is via safe_* below.
```

### `docs/dynamic-link-design.md`

- §4 failsafe 4: rewrite from "per-airframe ceiling loaded from
  drone.conf" to "hardcoded bounds from wfb-ng / hardware limits;
  see `drone/src/dl_ceiling.h`".
- §6 / config-schema sections: drop the seven keys; mention they
  were removed 2026-05-11.
- Per-airframe section: keep `safe_*` documentation; drop the
  ceiling table.
- The example `drone.conf` block (around line 1551–1568): trim to
  match the new sample.

### `docs/phase1-implementation.md`

Append a short note: "2026-05-11: ceiling knobs removed from
drone.conf; bounds hardcoded in `dl_ceiling.h`. See
`docs/superpowers/specs/2026-05-11-drop-drone-ceiling-config-design.md`."

### `CLAUDE.md`

No change required — it doesn't enumerate the ceiling keys.

### `README.md`

Quick check; remove ceiling references if any.

## Test plan

1. `make -C drone test` — all C unit tests pass, including the
   rewritten `test_ceiling.c` and the new config-deprecation case.
2. `make -C drone` — clean build, no warnings.
3. `python3 -m pytest` from repo root — all GS + Python e2e tests
   pass with the trimmed synthetic `drone.conf`.
4. Manual smoke: `dl-inject` an out-of-bounds decision (e.g.
   `--mcs 9`) against a live `dl-applier`, confirm rejection in the
   applier log with the same reason string the previous ceiling
   produced.
5. Manual smoke: load a `drone.conf` containing `mcs_max = 7`,
   confirm `dl_config_load` fails with the deprecation message.

## Out of scope / follow-ups

- Reviewing whether the GS policy's bounds (`policy.py` defaults)
  should be tightened or relaxed in light of the drone no longer
  enforcing per-airframe caps. The current GS bounds remain
  authoritative for what the GS will *propose*; this spec only
  changes what the drone will *accept*.
