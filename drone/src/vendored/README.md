# Vendored headers from wfb-ng

`tx_cmd.h` is the UDP control-socket wire format for `wfb_tx`. Our
`dl-applier` speaks this protocol verbatim — we vendor rather than
depend because the drone image should not need wfb-ng's headers or
source at install time.

## Pinned version

- Repo:    <https://github.com/wfb-ng/wfb-ng>
- SHA:     `208ec1d71e9d26bc8fbe57f5cb0903f968fafb06`
- Branch:  `feat/interleaving_uep`
- Tag:     (commit `e71b317` — "Phase 1 Step A: --interleave-depth
           flag and SESSION plumbing")
- Contract version: `WFB_IPC_CONTRACT_VERSION = 2`
  (from `wfb-ng/src/wifibroadcast.hpp`)

## Local modifications

1. Added `#include <stdbool.h>` so the header compiles standalone —
   upstream relies on includes pulled in by its C++ callers.

No other edits. Do not mutate struct layout; if wfb-ng's layout
changes we need a refresh (see below) and a matching contract-version
bump upstream.

## Refresh procedure

When wfb-ng bumps `WFB_IPC_CONTRACT_VERSION` or reshapes `cmd_req_t`
/ `cmd_resp_t`:

```
# 1. Pull the target wfb-ng revision into /workspace/wfb-ng.
cd /workspace/wfb-ng && git fetch && git checkout <new-sha>

# 2. Note the new SHA.
NEW_SHA=$(git rev-parse HEAD)

# 3. Overwrite our vendored copy, preserving the top comment block
#    and the stdbool.h include (re-apply by hand if the diff is trivial;
#    otherwise use a three-way merge).
cp /workspace/wfb-ng/src/tx_cmd.h /workspace/drone/src/vendored/tx_cmd.h
# ... then re-add our header comment and stdbool include.

# 4. Bump the SHA line in this README.

# 5. Confirm WFB_IPC_CONTRACT_VERSION matches the value the applier
#    expects (in dl_backend_tx.c or wherever we cross-check). Bump
#    that constant in lockstep if needed.

# 6. Re-run the drone test suite.
make -C drone test
pytest tests/test_drone_e2e.py
```

If wfb-ng adds new `CMD_*` opcodes that the applier doesn't dispatch,
that's fine — they're forward-compatible. Reorders or renames of
existing struct fields are breaking and require a Phase 2 wire
re-spec.
