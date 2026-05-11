# CLAUDE.md — repo conventions and gotchas for Claude

Read this before touching anything. Quick-reference notes, not a
design doc — for that see `docs/dynamic-link-design.md`, which is the
authoritative spec, and the phase-specific companions
`docs/phase0-implementation.md`, `docs/phase1-implementation.md`,
`docs/phase2-implementation.md` (write these as phases land).

## What this project is

`dynamic-link` is an adaptive link controller for wfb-ng. It's a
**separate repo** from wfb-ng and a pure consumer of wfb-ng's stable
public surfaces (JSON stats API, `tx_cmd.h` control socket, tunnel
and mavlink streams). No wfb-ng source changes are required to run.

Two components:

- **GS controller** — Python (`gs/dynamic_link/`). Subscribes to the
  wfb-ng stats feed, runs the §4 control loops, and serialises
  decisions onto the return link.
- **Drone applier** — C (`drone/src/`). Binds a UDP socket on the
  drone, validates decision packets against a local ceiling, and
  dispatches to three backends (wfb_tx via tx_cmd.h, `iw` for TX
  power, HTTP for the encoder).

Phases (as landed):
- **Phase 0** — GS observer, log-only. `enabled: false` default.
- **Phase 1** — drone `dl-applier` + `dl-inject` CLI. Driven by hand
  or by tests; not yet driven by the GS.
- **Phase 2** — GS → drone wire + drone → GS MAVLink STATUSTEXT
  status channel. Flip `enabled: true` in `gs.yaml` to activate
  end-to-end. (The Phase 2 oscillation detector was removed
  during bring-up — see design doc §5 failsafe 2.)
- **Phase 3** (pending) — airframe tuning + flight validation.

## Repo layout

```
gs/dynamic_link/    Python package (entry: service.main)
drone/src/          C sources (dl_applier, dl_inject, dl_*, vendored/)
drone/Makefile      Native build + CROSS_COMPILE passthrough
drone/build/        Build artifacts (gitignored)
conf/               gs.yaml.sample, drone.conf.sample, radios/*
packaging/          systemd + OpenRC units
tests/              pytest (GS + Python e2e)
tests/drone/        C unit tests (run via `make -C drone test`)
docs/               Design doc + per-phase implementation notes
wfb-ng/             Reference checkout (GITIGNORED — not our code)
```

## Commands

| Need | Command |
|---|---|
| Run pytest (GS unit + Python e2e) | `python3 -m pytest --ignore=tests/test_mavlink_status.py` (from repo root) |
| Run C unit tests | `make -C drone test` |
| Build C binaries | `make -C drone` → `drone/build/dl-applier`, `dl-inject` |
| Cross-compile for ARM | `make -C drone CROSS_COMPILE=arm-linux-gnueabihf-` |
| Smoke the GS service on a capture | `python3 -m dynamic_link.service --config conf/gs.yaml.sample --replay capture.jsonl --log-dir /tmp/dl-flights` |
| Smoke dl-inject | `drone/build/dl-inject --target 127.0.0.1:5800 --mcs 5 --bandwidth 20 --tx-power 18 --k 8 --n 14 --depth 2 --bitrate 12000 --fps 60` |
| Dry-run dl-inject (hex bytes) | `drone/build/dl-inject --dry-run --mcs 5 ...` |

## External dependencies

- **PyYAML** — gs.yaml parser.
- **pytest / pytest-asyncio** — dev-only.
- **wfb-ng Python package** (`wfb_ng.mavlink`, `wfb_ng.mavlink_protocol`)
  — Phase 2 `mavlink_status.py` reuses wfb-ng's bundled MAVLink codec.
  Operators running the radio link already have this installed. The
  `tests/test_mavlink_status.py` suite needs it too; the documented
  pytest command above ignores that file so a clean checkout passes
  without an install. To run those tests on this workstation:
  `VERSION=1.0.0 COMMIT=abc1234 python3 -m pip install
  --break-system-packages -e /workspace/wfb-ng` (wfb-ng's setup.py
  requires both env vars; any PEP-440-ish version string works), then
  drop the `--ignore=` flag.
- **Twisted** — pulled in transitively by the wfb-ng package. We don't
  use Twisted directly; dynamic-link's own Python code is stdlib
  asyncio.

## Key conventions

### Wire format authority

`drone/src/dl_wire.h` defines the decision-packet layout. `gs/dynamic_link/wire.py`
mirrors it byte-for-byte. The contract is anchored by
`tests/test_wire_contract.py`, which runs `dl-inject --dry-run`
against Python's encoder and diffs the hex bytes. Don't change one
side without updating the other and re-running that test.

Same pattern for the vendored wfb-ng control protocol at
`drone/src/vendored/tx_cmd.h`: verbatim from wfb-ng's
`src/tx_cmd.h`, pinned at a specific upstream SHA recorded in
`drone/src/vendored/README.md`. Upstream schema bumps `WFB_IPC_CONTRACT_VERSION`;
we re-vendor lockstep.

### Port naming caveat

Design doc §6 calls the JSON stats feed `stats_port`. In wfb-ng's
`master.cfg`, that name is the **msgpack** feed wfb-cli uses
(8002/8003). The JSON feed we subscribe to is bound to `api_port`
(8102/8103). Our config uses the endpoint-URL form
(`wfb_ng.stats_api: tcp://...:8103`) to sidestep this.

### Operational prerequisites (wfb-ng master.cfg changes)

Edits the operator must make before we run end-to-end:

1. `[common] log_interval = 100` (design doc says 10 Hz cadence).
2. `[<tx-section>] control_port = 8000` (default 0 picks a random
   port our applier can't target).
3. `/etc/wfb.yaml` must contain `wireless.mlink: <integer>` — the
   radio MTU. dl-applier reads this at boot and reports it to the
   GS via the DLHE handshake; missing key means the applier refuses
   to send HELLO and the GS stays in safe_defaults.
4. `/etc/majestic.yaml` must contain `video0.fps: <integer>` —
   same constraint as above, on the FPS side.

Call these out explicitly in any user-facing doc; they're easy to
miss and the failures are silent.

### Per-backend state tracking (Phase 1 learning)

The applier's three backends (tx, radio, encoder) each carry their
own `prev` state struct. Sharing one mutable `prev` across all three
breaks the diff-based apply path — after backend 1 updates `prev`,
backend 2 sees "no change" and skips what should have fired. See
`drone/src/dl_applier.c`'s `last_tx` / `last_radio` / `last_enc`.
Don't "simplify" these back to one.

### Tests: sandbox readiness (Phase 2 learning)

UDP `sendto()` to an unbound local port succeeds silently on Linux —
don't use it as a "did the process bind?" probe. The pattern that
works: try to bind the same port yourself; if you get `EADDRINUSE`,
the applier owns it. See `tests/test_drone_e2e.py::_sandbox`'s bind
loop.

### Git hygiene

  — **never** modify the repo's or user's global `git config`.
- Two-commit log so far (one per phase). Phase commits include a
  meaningful body citing design-doc sections. Co-author trailer
  `Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>`.
- `wfb-ng/` is gitignored — it's a reference checkout, not our
  code. Same for `drone/build/`.

### Code style

- **No comments by default.** Only WHY-not-obvious: a hidden
  constraint, a subtle invariant, a workaround for a specific bug.
  Don't narrate what self-explanatory code already does.
- Python: stdlib asyncio (not Twisted). Dataclasses for records.
  `snake_case`. Type hints.
- C: C11, `-Wall -Wextra -Wformat=2`. `snprintf` over `strncpy` for
  bounded copies (gcc's `-Wstringop-truncation` warns on the latter
  pattern even when safe). POSIX only; no libnl.
- Identifiers mirror the design-doc vocabulary where it exists
  (`residual_loss`, `fec_work`, `k_band`, etc.). Search there
  before inventing new names.

### Testing conventions

- C unit tests: `DL_TEST(name)` macro registers via a constructor;
  run through `tests/drone/test_main.c`. One assertion per specific
  invariant, not one giant test per module.
- Python unit tests: plain pytest. Use the shared `_sandbox` context
  manager from `tests/test_drone_e2e.py` when you need a live
  applier.
- E2E tests spawn the real `dl-applier` against mock `wfb_tx` / mock
  encoder HTTP / mock MAVLink sink — no live radio required. Keep
  it that way; CI runs on a workstation.

### Dynamic FEC (P4b)

`(k, n)` is computed at runtime, not pinned per-MCS in the profile:

  k = clamp(bitrate_kbps * 1000 / (fps * mtu_bytes * 8), k_min, k_max)
  n = ceil(k * (1 + base_redundancy_ratio)) + n_escalation

`mtu_bytes` and `fps` come from the P4a handshake (drone reads them
from /etc/wfb.yaml and /etc/majestic.yaml). `n_escalation` ramps on
sustained residual_loss and decays on sustained clean windows; the
EmitGate bundles (k, n) rewrites onto MCS-change ticks per
`docs/knob-cadence-bench.md`.

Bitrate uses a FIXED `k/n = 1/(1 + base_redundancy_ratio)`, not the
live (k, n), so encoder allocation stays steady when n_escalation
moves.

## How to land a new phase

1. Write a plan first (use
   plan mode). Don't start coding until the user approves.
2. Track the work via `TaskCreate` / `TaskUpdate` — keep the list
   tight, no "run tests" tasks that are just a single bash call.
3. Tests alongside code, not after. Each new module gets its own
   test file.
4. Run both test suites end-to-end (`pytest` + `make -C drone test`)
   before asking for a commit.
5. Commit message: cite the design-doc section the phase
   implements; list the main modules added; call out operational
   prerequisites and any caught-bugs. See existing commits for
   shape.
6. Update README.md if the user-visible surface changed.

## Things NOT to do

- Don't modify `git config` (global or local).
- Don't commit `wfb-ng/` or `drone/build/` — gitignored for a
  reason.
- Don't reach into `wfb_ng.` internals beyond `wfb_ng.mavlink` and
  `wfb_ng.mavlink_protocol` — those two modules are stable enough
  to lean on; the rest (services, protocols, server) is wfb-ng's
  private surface.
- Don't add libraries lightly on the drone side. The drone image is
  resource-constrained; the design explicitly avoids libcurl,
  libnl, libmavlink. Check `docs/dynamic-link-design.md` §2
  "Why the asymmetry" before adding anything.
- Don't write a shared "contract file" between C and Python wire
  encoders. The two test fixtures (static byte-layout assertions +
  dynamic dl-inject --dry-run diff) are the whole contract.
