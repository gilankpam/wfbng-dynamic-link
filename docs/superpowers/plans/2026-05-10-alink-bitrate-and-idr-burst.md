# Alink dynamic bitrate + IDR burst — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Adopt two `alink_gs` behaviors into the dynamic-link GS controller while keeping deterministic per-MCS FEC: (1) compute encoder bitrate each tick from `phy_data_rate × utilization × (k/n)` instead of reading a fixed value from the radio profile; (2) burst the IDR-flagged decision packet (4 × 20 ms) on loss so it survives lossy UDP on the return link.

**Architecture:** Two independent additive changes to `gs/dynamic_link/`. New `bitrate.py` module replaces a single `Policy.tick()` line. New `idr_burst.py` module wraps the existing `ReturnLink` + `Encoder` and is poked from the existing per-tick send path in `service.py`. Wire format unchanged. Drone code untouched.

**Tech Stack:** Python 3 stdlib (asyncio, dataclasses), pytest, PyYAML. Spec: `docs/superpowers/specs/2026-05-10-alink-bitrate-and-idr-burst-design.md`.

**Order of tasks:** Phase 1 (bitrate) is done atomically — Tasks 2 + 3 + 4 share state because dropping `bitrate_Mbps` from `FECEntry` makes `m8812eu2.yaml` and `test_policy_bitrate.py` fail until everything is updated. Each task within Phase 1 is a single commit, but the test-suite is only green again after Task 4. Phase 2 (IDR burst) is fully independent and can ship separately.

---

## Phase 1 — Dynamic bitrate

### Task 1: Add `bitrate.py` module + unit tests

**Files:**
- Create: `gs/dynamic_link/bitrate.py`
- Test: `tests/test_bitrate.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_bitrate.py`:

```python
"""Unit tests for the dynamic bitrate calculator."""
from __future__ import annotations

from pathlib import Path

import pytest

from dynamic_link.bitrate import BitrateConfig, compute_bitrate_kbps
from dynamic_link.profile import load_profile

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


def _profile():
    return load_profile("m8812eu2", [PACKAGED_DIR])


def _cfg(util=0.8, lo=1000, hi=24000):
    return BitrateConfig(
        utilization_factor=util,
        min_bitrate_kbps=lo,
        max_bitrate_kbps=hi,
    )


def test_typical_mcs_5_20mhz():
    """MCS 5 / 20 MHz: phy=52.0 Mbps, k/n=8/10=0.8, util=0.8.
    Raw = 52000 * 0.8 * 0.8 = 33280 kbps. Clamped to max=24000."""
    p = _profile()
    assert compute_bitrate_kbps(p, 20, 5, k=8, n=10, cfg=_cfg()) == 24000


def test_typical_mcs_3_20mhz_no_clamp():
    """MCS 3 / 20 MHz: phy=26.0 Mbps, k/n=4/7≈0.571, util=0.8.
    Raw = 26000 * 0.8 * 4/7 ≈ 11885 kbps. No clamp."""
    p = _profile()
    got = compute_bitrate_kbps(p, 20, 3, k=4, n=7, cfg=_cfg())
    assert 11800 <= got <= 11900


def test_clamp_to_min():
    """MCS 0 with low util: floors at min_bitrate_kbps."""
    p = _profile()
    # phy=6.5, k/n=2/5=0.4, util=0.1 → raw = 260 kbps; clamped to 1000.
    assert compute_bitrate_kbps(p, 20, 0, k=2, n=5,
                                cfg=_cfg(util=0.1)) == 1000


def test_clamp_to_max():
    """MCS 7 / 40 MHz with full util: ceilings at max_bitrate_kbps."""
    p = _profile()
    # phy=135 Mbps, k/n=12/14, util=1.0 → 115714 kbps; clamped to 24000.
    assert compute_bitrate_kbps(p, 40, 7, k=12, n=14,
                                cfg=_cfg(util=1.0)) == 24000


def test_kn_ratio_affects_result():
    """Same PHY/util, smaller k/n → smaller bitrate (more parity overhead)."""
    p = _profile()
    cfg = _cfg(util=0.5, lo=1, hi=99999)
    a = compute_bitrate_kbps(p, 20, 4, k=6, n=9, cfg=cfg)   # k/n ≈ 0.667
    b = compute_bitrate_kbps(p, 20, 4, k=6, n=12, cfg=cfg)  # k/n = 0.500
    assert a > b


def test_returns_int():
    p = _profile()
    got = compute_bitrate_kbps(p, 20, 4, k=6, n=9, cfg=_cfg())
    assert isinstance(got, int)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tests/test_bitrate.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_link.bitrate'`

- [ ] **Step 3: Write the implementation**

Create `gs/dynamic_link/bitrate.py`:

```python
"""Dynamic encoder bitrate from PHY rate × utilization × (k/n).

Replaces the per-row `bitrate_Mbps` lookup in the radio profile's
`fec_table`. Operator tunes one global `utilization_factor`
instead of a per-MCS bitrate; FEC overhead is folded in via the
actual `(k/n)` ratio of the row the leading loop selected. PHY
data rate comes from the profile's `data_rate_Mbps_LGI` table —
LGI only because we don't do short-GI selection (see design doc).

Defaults match alink_gs.conf:
  utilization_factor = 0.8   ([dynamic] utilization_factor)
  min_bitrate_kbps   = 1000  ([hardware] min_bitrate)
  max_bitrate_kbps   = 24000 ([hardware] max_bitrate)
"""
from __future__ import annotations

from dataclasses import dataclass

from .profile import RadioProfile


@dataclass(frozen=True)
class BitrateConfig:
    utilization_factor: float
    min_bitrate_kbps: int
    max_bitrate_kbps: int


def compute_bitrate_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    k: int,
    n: int,
    cfg: BitrateConfig,
) -> int:
    """Encoder kbps = PHY × utilization × (k/n), clamped to [min, max]."""
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    raw_kbps = phy_Mbps * 1000.0 * cfg.utilization_factor * (k / n)
    return int(max(cfg.min_bitrate_kbps,
                   min(cfg.max_bitrate_kbps, raw_kbps)))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tests/test_bitrate.py -v`
Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/bitrate.py tests/test_bitrate.py
git commit -m "$(cat <<'EOF'
gs: add dynamic bitrate calculator (alink port)

Single function: compute_bitrate_kbps(profile, bw, mcs, k, n, cfg)
returns phy_data_rate * utilization_factor * (k/n) in kbps, clamped
to [min_bitrate_kbps, max_bitrate_kbps]. Defaults from alink_gs.conf
(util=0.8, min=1000, max=24000) — wired up in a follow-up commit
that drops fec_table.bitrate_Mbps.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: Drop `bitrate_Mbps` from `FECEntry`/`MCSRow` + reject in validator

This task breaks `m8812eu2.yaml` loading and `test_policy_bitrate.py` until Tasks 3 + 5 land. Land Tasks 2-5 in one sitting.

**Files:**
- Modify: `gs/dynamic_link/profile.py`
- Test: `tests/test_profile.py`

- [ ] **Step 1: Update the failing test first — replace bitrate cases**

In `tests/test_profile.py`, replace the lines that assert `bitrate_Mbps` on entries (currently around lines 42 and 49-52) and update the fixture dict around lines 96-103. Then add a new test for the rejection.

Open `tests/test_profile.py` and find the test that asserts `mcs0.bitrate_Mbps == 1.5` (around line 42). Drop that line. Find the test asserting `entry.bitrate_Mbps == 18.0` (around line 52). Drop that line.

In the fixture dict (around lines 96-103), drop `, "bitrate_Mbps": <value>` from each entry, leaving e.g. `0: {"k": 2, "n": 5},`.

Update the test at line 114 (`d["fec_table"][20][8] = ...`): drop the `, "bitrate_Mbps": 30.0`.

Update the test at line 139 (`d["fec_table"][20][7] = ...`): drop the `, "bitrate_Mbps": 25.0`.

Then add this new test at the end of the file:

```python
def test_rejects_fec_table_bitrate_field(tmp_path):
    """bitrate_Mbps in fec_table is rejected with a clear error —
    bitrate is now computed from policy.bitrate."""
    d = _valid_dict()
    d["fec_table"][20][3] = {"k": 4, "n": 7, "bitrate_Mbps": 7.5}
    p = tmp_path / "p.yaml"
    p.write_text(yaml.safe_dump(d))
    with pytest.raises(
        ProfileError, match=r"bitrate_Mbps.*policy\.bitrate",
    ):
        load_profile_file(p)
```

If `_valid_dict()` doesn't already exist in the test file, locate the fixture used by the other tests (search for the `"fec_table"` literal) and lift the dict-construction into a helper named `_valid_dict()` returning a fresh deep copy. Use `copy.deepcopy` if needed.

- [ ] **Step 2: Run tests to verify the new one fails**

Run: `python3 -m pytest tests/test_profile.py::test_rejects_fec_table_bitrate_field -v`
Expected: FAIL — currently the validator silently accepts the extra field.

- [ ] **Step 3: Implement the schema change**

In `gs/dynamic_link/profile.py`:

Find `FECEntry` (around line 22):

```python
@dataclass(frozen=True)
class FECEntry:
    k: int
    n: int
    bitrate_Mbps: float
```

Replace with:

```python
@dataclass(frozen=True)
class FECEntry:
    k: int
    n: int
```

Find `MCSRow` (around line 29). Drop the `bitrate_Mbps: float` field. Update the docstring to reflect that bitrate is computed at runtime via `dynamic_link.bitrate.compute_bitrate_kbps` from `policy.bitrate`.

Find `_build_rows()` (around line 77). The `rows.append(MCSRow(...))` call constructs `bitrate_Mbps=entry.bitrate_Mbps` (around line 97) — drop that line so `MCSRow` constructor matches the new shape.

Find `_validate()`'s per-entry loop (around lines 181-211 — the `for mcs_key, entry in sub.items()` body). The current code:

```python
for f in ("k", "n", "bitrate_Mbps"):
    if f not in entry:
        raise ProfileError(...)
```

Replace with:

```python
for f in ("k", "n"):
    if f not in entry:
        raise ProfileError(
            f"{source}: fec_table[{bw}][{mcs_i}] missing {f!r}"
        )
if "bitrate_Mbps" in entry:
    raise ProfileError(
        f"{source}: fec_table[{bw}][{mcs_i}] contains bitrate_Mbps; "
        f"bitrate is computed from policy.bitrate — drop this field"
    )
```

Drop the `br = float(entry["bitrate_Mbps"])` line and the `if br <= 0.0: ...` block. Drop `bitrate_Mbps=br` from the `FECEntry(...)` constructor call.

- [ ] **Step 4: Run profile tests to verify they pass**

Run: `python3 -m pytest tests/test_profile.py -v`
Expected: all profile tests pass (the new rejection test, the modified existing tests).

- [ ] **Step 5: Do NOT commit yet** — `m8812eu2.yaml` still has `bitrate_Mbps` keys and will fail to load via the e2e/policy tests. Move on to Task 3.

---

### Task 3: Strip `bitrate_Mbps` from `m8812eu2.yaml`

**Files:**
- Modify: `conf/radios/m8812eu2.yaml`

- [ ] **Step 1: Edit the fec_table block**

Open `conf/radios/m8812eu2.yaml`. The `fec_table:` block runs from approximately line 116 to line 137. Each entry currently looks like:

```yaml
    0: {k: 2,  n: 5,  bitrate_Mbps: 1.5}    # goodput 2.6,  block-fill 14.9 ms
```

For each of the 16 entries (8 rows × 2 bandwidths), drop the `, bitrate_Mbps: <number>` portion. The result for that line:

```yaml
    0: {k: 2,  n: 5}                        # goodput 2.6,  block-fill 14.9 ms
```

Trailing inline comments may stay as-is. Also update the block's leading comment (around line 99-115) to reflect that bitrate now comes from `policy.bitrate × (k/n)` rather than being pinned per-row. One short paragraph replacing the existing:

> "Per-MCS FEC `(k, n)` and encoder bitrate. Pinned per-row..."

with:

> "Per-MCS FEC `(k, n)`. Pinned per-row, never escalated reactively
> — `docs/knob-cadence-bench.md` showed FEC `CMD_SET_FEC` reconfigs
> are the most expensive controller knob. Bitrate is computed at
> runtime as `phy_data_rate × policy.bitrate.utilization_factor ×
> (k/n)`; see `gs/dynamic_link/bitrate.py`."

- [ ] **Step 2: Verify the profile loads**

Run: `python3 -c "from dynamic_link.profile import load_profile; from pathlib import Path; p = load_profile('m8812eu2', [Path('conf/radios')]); print(p.name, len(p.fec_table[20]), len(p.fec_table[40]))"`
Expected: `BL-M8812EU2 8 8`

- [ ] **Step 3: Do NOT commit yet** — `test_policy_bitrate.py` still references `row.bitrate_Mbps` and `entry.bitrate_Mbps`. Move on to Task 4.

---

### Task 4: Wire `BitrateConfig` into `Policy` + update bitrate tests

**Files:**
- Modify: `gs/dynamic_link/policy.py`
- Test: `tests/test_policy_bitrate.py`

- [ ] **Step 1: Rewrite `tests/test_policy_bitrate.py` to cover the new behavior**

Replace the entire file contents:

```python
"""Bitrate is computed each tick via `compute_bitrate_kbps`.

Verifies the wiring inside Policy: state.bitrate_kbps reflects the
current MCS row's (k, n) and the configured BitrateConfig, and
moves when MCS changes.
"""
from __future__ import annotations

from pathlib import Path

from dynamic_link.bitrate import BitrateConfig, compute_bitrate_kbps
from dynamic_link.profile import load_profile

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


def _profile():
    return load_profile("m8812eu2", [PACKAGED_DIR])


def test_compute_per_row_matches_formula():
    """For each MCS row, compute_bitrate_kbps matches phy * util * k/n
    (clamped). Sanity check that the calculator and the profile agree
    on what (k, n) goes with each MCS."""
    prof = _profile()
    cfg = BitrateConfig(utilization_factor=0.8,
                        min_bitrate_kbps=1, max_bitrate_kbps=999_999)
    rows = prof.snr_mcs_map(bandwidth=20, snr_margin_db=0.0)
    for row in rows:
        entry = prof.fec_for(20, row.mcs)
        expected = int(prof.data_rate_Mbps_LGI[20][row.mcs] * 1000.0
                       * 0.8 * (entry.k / entry.n))
        got = compute_bitrate_kbps(prof, 20, row.mcs, entry.k, entry.n, cfg)
        assert got == expected


def test_kn_pair_carries_through_to_bitrate():
    """High-band rows use (k=12, n=14); the (k/n) factor of ~0.857
    shows up in the computed bitrate."""
    prof = _profile()
    cfg = BitrateConfig(utilization_factor=0.8,
                        min_bitrate_kbps=1, max_bitrate_kbps=999_999)
    e7 = prof.fec_for(20, 7)
    bitrate7 = compute_bitrate_kbps(prof, 20, 7, e7.k, e7.n, cfg)
    # 65.0 * 1000 * 0.8 * 12/14 ≈ 44571
    assert 44400 <= bitrate7 <= 44700
```

- [ ] **Step 2: Run the rewritten test (will fail at import — no PolicyConfig wiring yet)**

Run: `python3 -m pytest tests/test_policy_bitrate.py -v`
Expected: PASS (this test does not touch Policy yet — it exercises `compute_bitrate_kbps` directly on the profile, and the profile loads cleanly after Task 3).

- [ ] **Step 3: Wire `BitrateConfig` into `PolicyConfig` and `Policy.tick()`**

Open `gs/dynamic_link/policy.py`.

Add the import near the top, with the other module-internal imports:

```python
from .bitrate import BitrateConfig, compute_bitrate_kbps
```

Find `class SafeDefaults` (around line 136). Drop the `bitrate_kbps: int = 2000` line — cold-boot bitrate is now computed.

Find `class PolicyConfig` (around line 145). Add the `bitrate` field after `safe`:

```python
@dataclass
class PolicyConfig:
    leading: LeadingLoopConfig = field(default_factory=LeadingLoopConfig)
    gate: GateConfig = field(default_factory=GateConfig)
    selection: ProfileSelectionConfig = field(
        default_factory=ProfileSelectionConfig
    )
    cooldown: CooldownConfig = field(default_factory=CooldownConfig)
    fec: FECBounds = field(default_factory=FECBounds)
    safe: SafeDefaults = field(default_factory=SafeDefaults)
    bitrate: BitrateConfig = field(default_factory=lambda: BitrateConfig(
        utilization_factor=0.8,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    ))
    predictor: PredictorConfig = field(default_factory=PredictorConfig)
    max_latency_ms: float = 50.0
    sustained_loss_windows: int = 3
    clean_windows_for_depth_stepdown: int = 10
    starvation_windows: int = 5
```

Find `class Policy.__init__` (around line 697). Locate the cold-boot block (around lines 715-724):

```python
        row = self.leading.current_row
        self.state = PolicyState(
            mcs=row.mcs,
            bandwidth=cfg.leading.bandwidth,
            tx_power_dBm=int(self.leading.state.tx_power_dBm),
            k=row.k,
            n=row.n,
            depth=cfg.safe.depth,
            bitrate_kbps=int(row.bitrate_Mbps * 1000),
        )
```

Replace with:

```python
        row = self.leading.current_row
        self.state = PolicyState(
            mcs=row.mcs,
            bandwidth=cfg.leading.bandwidth,
            tx_power_dBm=int(self.leading.state.tx_power_dBm),
            k=row.k,
            n=row.n,
            depth=cfg.safe.depth,
            bitrate_kbps=compute_bitrate_kbps(
                profile, cfg.leading.bandwidth, row.mcs,
                row.k, row.n, cfg.bitrate,
            ),
        )
```

Find `Policy.tick()` (around line 726). Locate the `encoder_kbps = row.bitrate_Mbps * 1000.0` line (around line 760). Replace with:

```python
        new_bitrate_kbps = compute_bitrate_kbps(
            self.profile, self.state.bandwidth, row.mcs,
            row.k, row.n, self.cfg.bitrate,
        )
        encoder_kbps = float(new_bitrate_kbps)
```

Then locate `self.state.bitrate_kbps = int(row.bitrate_Mbps * 1000)` (around line 803). Replace with:

```python
        self.state.bitrate_kbps = new_bitrate_kbps
```

- [ ] **Step 4: Run the full pytest suite to catch regressions**

Run: `python3 -m pytest -v`
Expected: all tests pass. Specifically watch for any test that previously asserted a specific `bitrate_kbps` value — those values will have changed under the new formula. Update any failing assertion to match `compute_bitrate_kbps(prof, bw, mcs, k, n, BitrateConfig(0.8, 1000, 24000))` for the row in question. **Don't change the formula to match a stale test — change the test.**

If a test fixture builds a `PolicyConfig` with explicit field positions, ensure it now passes `bitrate=BitrateConfig(...)`.

- [ ] **Step 5: Commit Tasks 2 + 3 + 4 atomically**

```bash
git add gs/dynamic_link/profile.py gs/dynamic_link/policy.py \
        conf/radios/m8812eu2.yaml \
        tests/test_profile.py tests/test_policy_bitrate.py
git commit -m "$(cat <<'EOF'
gs: drop fec_table.bitrate_Mbps; wire dynamic bitrate into Policy

Replaces the per-MCS bitrate column in the radio profile with
runtime computation: phy_data_rate * utilization_factor * (k/n),
clamped to [min_bitrate_kbps, max_bitrate_kbps]. Defaults match
alink_gs.conf.

Profile validator now rejects fec_table entries containing
bitrate_Mbps with a clear error pointing to policy.bitrate.

m8812eu2.yaml fec_table entries reduced to {k, n}. Per-MCS bitrate
is no longer operator-tunable in the profile — operators tune one
utilization_factor in gs.yaml instead.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: Parse `policy.bitrate` from `gs.yaml` in `service.py`

**Files:**
- Modify: `gs/dynamic_link/service.py`
- Modify: `conf/gs.yaml.sample`

- [ ] **Step 1: Add `BitrateConfig` parsing in `_build_policy_config`**

Open `gs/dynamic_link/service.py`. Find the imports (around line 19-28). Add `BitrateConfig` to the existing import line:

```python
from .bitrate import BitrateConfig
```

Find the `SafeDefaults` import line and **drop `bitrate_kbps`** from the `SafeDefaults` constructor — but `SafeDefaults` itself is still imported because k/n/depth/mcs remain on it. Locate `safe = SafeDefaults(...)` (around line 194). Drop the `bitrate_kbps=int(safe_raw.get("bitrate_kbps", 2000)),` line. Final form:

```python
    safe = SafeDefaults(
        k=int(safe_video.get("k", 8)),
        n=int(safe_video.get("n", 12)),
        depth=int(safe_raw.get("depth", 1)),
        mcs=int(safe_raw.get("mcs", 1)),
    )
```

Find `policy_raw = raw.get("policy", {})` (around line 204). Add bitrate parsing immediately after that line:

```python
    bitrate_raw = policy_raw.get("bitrate", {})
    bitrate = BitrateConfig(
        utilization_factor=float(bitrate_raw.get("utilization_factor", 0.8)),
        min_bitrate_kbps=int(bitrate_raw.get("min_bitrate_kbps", 1000)),
        max_bitrate_kbps=int(bitrate_raw.get("max_bitrate_kbps", 24000)),
    )
    if not (0.0 < bitrate.utilization_factor <= 1.0):
        raise ValueError(
            f"policy.bitrate.utilization_factor must be in (0, 1]; "
            f"got {bitrate.utilization_factor}"
        )
    if bitrate.min_bitrate_kbps <= 0:
        raise ValueError(
            f"policy.bitrate.min_bitrate_kbps must be > 0; "
            f"got {bitrate.min_bitrate_kbps}"
        )
    if bitrate.max_bitrate_kbps < bitrate.min_bitrate_kbps:
        raise ValueError(
            f"policy.bitrate.max_bitrate_kbps ({bitrate.max_bitrate_kbps}) "
            f"< min_bitrate_kbps ({bitrate.min_bitrate_kbps})"
        )
```

Find the `return PolicyConfig(...)` (around line 205). Add `bitrate=bitrate,` to the kwargs:

```python
    return PolicyConfig(
        leading=leading,
        gate=gate,
        selection=selection,
        cooldown=cooldown,
        fec=fec,
        safe=safe,
        bitrate=bitrate,
        predictor=predictor,
        max_latency_ms=float(video_raw.get("max_latency_ms", 50.0)),
        starvation_windows=int(policy_raw.get("starvation_windows", 5)),
    )
```

- [ ] **Step 2: Add the `policy.bitrate` block to `gs.yaml.sample`**

Open `conf/gs.yaml.sample`. Find the `fec:` block. After it, add:

```yaml
# Encoder bitrate is computed each tick as
#   data_rate_Mbps_LGI[bw][mcs] * 1000 * utilization_factor * (k/n)
# clamped to [min_bitrate_kbps, max_bitrate_kbps]. Operator tunes one
# utilization_factor instead of per-MCS bitrate. Defaults from
# alink_gs.conf ([dynamic] utilization_factor, [hardware] min_bitrate,
# max_bitrate).
policy:
  bitrate:
    utilization_factor: 0.8
    min_bitrate_kbps: 1000
    max_bitrate_kbps: 24000
```

If `policy:` already exists in `conf/gs.yaml.sample` (it may carry `starvation_windows`), nest `bitrate:` under it as a sibling. Search the file with `grep -n "^policy:" conf/gs.yaml.sample` first; the file currently has `policy_raw = raw.get("policy", {})` consumers but the sample may not yet have a `policy:` top-level block. Add one if absent.

Also locate the existing `fec:` block in the sample (it carries `depth_max` and `mtu_bytes`). Update its leading comment to drop the "and bitrate" mention if present — bitrate now lives under `policy.bitrate`.

If the sample has a `safe.video.bitrate_kbps` line (it referenced `SafeDefaults.bitrate_kbps`), drop it too.

- [ ] **Step 3: Run pytest end-to-end**

Run: `python3 -m pytest -v`
Expected: all tests pass.

Also smoke-load the sample config:

Run: `python3 -m dynamic_link.service --config conf/gs.yaml.sample --replay /dev/null --log-level WARNING || true`
Expected: clean parse — process exits because `/dev/null` is not a valid replay file, but no `KeyError` / `ValueError` from config parsing. (If `--replay /dev/null` produces a noisy traceback that's hard to read, omit `--replay` and let it fail to bind the stats socket; either way, the failure must be downstream of `_build_policy_config`.)

- [ ] **Step 4: Commit**

```bash
git add gs/dynamic_link/service.py conf/gs.yaml.sample
git commit -m "$(cat <<'EOF'
gs: parse policy.bitrate from gs.yaml

Wires BitrateConfig (utilization_factor, min/max kbps) through
service.py into PolicyConfig. Defaults match alink_gs.conf.
Validates utilization_factor in (0, 1], min > 0, max >= min.

Drops SafeDefaults.bitrate_kbps; cold-boot bitrate is now derived
from the boot row's (k, n) via compute_bitrate_kbps.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 6: Update design docs

**Files:**
- Modify: `docs/dynamic-link-design.md`
- Modify: `docs/knob-cadence-bench.md`

- [ ] **Step 1: Patch `docs/dynamic-link-design.md`**

Search for "fec_table" and "bitrate" in `docs/dynamic-link-design.md` (likely §4 and §6). Wherever the doc says fec_table carries "(k, n, bitrate)" or describes bitrate as an operator-validated per-row value, replace with the new wording. Sample replacement paragraph:

> The radio profile's `fec_table[bw][mcs]` carries the operator-
> validated `(k, n)` for each MCS row. Encoder bitrate is computed
> at runtime as `data_rate_Mbps_LGI[bw][mcs] * 1000 *
> policy.bitrate.utilization_factor * (k/n)`, clamped to
> `[min_bitrate_kbps, max_bitrate_kbps]`. The `(k/n)` factor folds
> in FEC overhead so the encoder produces what the link can carry
> after parity. See `gs/dynamic_link/bitrate.py`.

Run `grep -n "bitrate_Mbps\|bitrate)" docs/dynamic-link-design.md` to locate every site; update each.

- [ ] **Step 2: Patch `docs/knob-cadence-bench.md`**

Same exercise. Search for "bitrate" and "fec_table" mentions; update wording to reflect that bitrate is computed not pinned.

Run `grep -n "bitrate_Mbps\|operator-validated.*bitrate" docs/knob-cadence-bench.md`.

- [ ] **Step 3: Commit**

```bash
git add docs/dynamic-link-design.md docs/knob-cadence-bench.md
git commit -m "$(cat <<'EOF'
docs: bitrate now computed dynamically (not in fec_table)

Updates the design doc and the knob-cadence bench writeup to
reflect that fec_table[bw][mcs] now carries (k, n) only; encoder
bitrate is computed each tick via compute_bitrate_kbps using
policy.bitrate.utilization_factor.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Phase 2 — IDR burst path

### Task 7: Add `idr_burst.py` module + unit tests

**Files:**
- Create: `gs/dynamic_link/idr_burst.py`
- Test: `tests/test_idr_burst.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_idr_burst.py`:

```python
"""Unit tests for the IDR burst sender.

Uses a fake ReturnLink (captures sent packets) and a real wire.Encoder.
Asserts: trigger fires count-1 burst packets after the regular tick,
each carries FLAG_IDR_REQUEST with monotonically increasing sequence,
re-trigger mid-burst resets _remaining, enabled=False is a no-op.
"""
from __future__ import annotations

import asyncio

import pytest

from dynamic_link.decision import Decision
from dynamic_link.idr_burst import IdrBurstConfig, IdrBurster
from dynamic_link.wire import FLAG_IDR_REQUEST, Encoder as WireEncoder


class FakeReturnLink:
    def __init__(self) -> None:
        self.packets: list[bytes] = []

    def send(self, packet: bytes) -> bool:
        self.packets.append(packet)
        return True


def _decision(idr: bool = True) -> Decision:
    return Decision(
        timestamp=1.0,
        mcs=5,
        bandwidth=20,
        tx_power_dBm=22,
        k=8,
        n=10,
        depth=1,
        bitrate_kbps=18000,
        idr_request=idr,
        reason="test",
    )


# Wire layout (gs/dynamic_link/wire.py module docstring):
#   byte 5     = flags
#   bytes 8-12 = sequence (big-endian uint32)
def _seq(packet: bytes) -> int:
    return int.from_bytes(packet[8:12], "big")


def _decoded_seqs(packets: list[bytes]) -> list[int]:
    return [_seq(p) for p in packets]


def _all_have_idr_flag(packets: list[bytes]) -> bool:
    return all(p[5] & FLAG_IDR_REQUEST for p in packets)


@pytest.mark.asyncio
async def test_burst_emits_count_minus_one_packets():
    """trigger() schedules count-1 packets at interval_ms spacing."""
    rl = FakeReturnLink()
    enc = WireEncoder(seq=100)
    cfg = IdrBurstConfig(enabled=True, count=4, interval_ms=1)
    burster = IdrBurster(cfg, rl, enc)

    burster.trigger(_decision())
    # Wait long enough for the 3 sleeps of 1ms each (give scheduler slack).
    await asyncio.sleep(0.05)

    assert len(rl.packets) == 3                              # count - 1
    assert _all_have_idr_flag(rl.packets)
    seqs = _decoded_seqs(rl.packets)
    assert seqs == sorted(seqs) and len(set(seqs)) == 3


@pytest.mark.asyncio
async def test_disabled_is_noop():
    rl = FakeReturnLink()
    enc = WireEncoder(seq=1)
    cfg = IdrBurstConfig(enabled=False, count=4, interval_ms=1)
    burster = IdrBurster(cfg, rl, enc)
    burster.trigger(_decision())
    await asyncio.sleep(0.05)
    assert rl.packets == []


@pytest.mark.asyncio
async def test_retrigger_mid_burst_resets_counter():
    """A second trigger() while a burst is in flight resets _remaining."""
    rl = FakeReturnLink()
    enc = WireEncoder(seq=1)
    cfg = IdrBurstConfig(enabled=True, count=3, interval_ms=10)
    burster = IdrBurster(cfg, rl, enc)

    burster.trigger(_decision())
    # After ~12 ms, one burst packet should have fired (one count - 1 = 2,
    # spaced 10 ms apart). Re-trigger.
    await asyncio.sleep(0.012)
    pre = len(rl.packets)
    burster.trigger(_decision())
    # Wait for the rest of the (refreshed) burst.
    await asyncio.sleep(0.05)
    # First-burst sent some packets, then re-trigger added count-1 = 2 more
    # from the reset. Total >= pre + 2, and >= 3 in absolute terms.
    assert len(rl.packets) >= pre + 1
    assert len(rl.packets) >= 3


@pytest.mark.asyncio
async def test_no_idr_when_decision_lacks_flag_is_callers_problem():
    """trigger() does NOT inspect decision.idr_request — callers gate
    on that field before invoking trigger(). The burst always sets the
    IDR flag on the wire (this is the whole point)."""
    rl = FakeReturnLink()
    enc = WireEncoder(seq=1)
    cfg = IdrBurstConfig(enabled=True, count=2, interval_ms=1)
    burster = IdrBurster(cfg, rl, enc)
    burster.trigger(_decision(idr=False))
    await asyncio.sleep(0.05)
    assert len(rl.packets) == 1                    # count - 1
    assert _all_have_idr_flag(rl.packets)
```

If `pytest-asyncio` isn't already configured for the project, add `@pytest.fixture(scope="session")` for an event loop or mark the file with `pytestmark = pytest.mark.asyncio` — check the existing `tests/test_phase2_e2e.py` or `tests/test_phase3_e2e.py` for the project's pattern. Most likely `pytest.ini` / `pyproject.toml` already configures it.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python3 -m pytest tests/test_idr_burst.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_link.idr_burst'`

- [ ] **Step 3: Write the implementation**

Create `gs/dynamic_link/idr_burst.py`:

```python
"""IDR burst sender — adopts alink_gs's keyframe-pump idea.

When the policy emits a Decision with idr_request=True, the regular
per-tick send path puts one packet on the wire. This module schedules
N-1 additional copies at interval_ms spacing, decoupled from the
10 Hz stats cadence, so a single packet loss on the lossy return
link doesn't swallow the IDR request.

Wire format unchanged: each burst packet is a normal DLK1 decision
frame with FLAG_IDR_REQUEST set and a fresh sequence number (so the
drone applier doesn't dedup it). Drone-side
dl_backend_enc_request_idr throttles HTTP calls by wall clock, so
the N attempted bursts collapse to one camera API call as long as
min_idr_interval_ms > interval_ms * (count - 1).

Defaults match alink_gs.conf [keyframe]:
  enabled     = True   (allow_idr)
  count       = 4      (idr_max_messages)
  interval_ms = 20     (idr_send_interval_ms)
"""
from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass

from .decision import Decision
from .return_link import ReturnLink
from .wire import Encoder as WireEncoder

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class IdrBurstConfig:
    enabled: bool = True
    count: int = 4
    interval_ms: int = 20


class IdrBurster:
    def __init__(
        self,
        cfg: IdrBurstConfig,
        return_link: ReturnLink,
        encoder: WireEncoder,
    ) -> None:
        self._cfg = cfg
        self._return_link = return_link
        self._encoder = encoder
        self._latest_decision: Decision | None = None
        self._remaining = 0
        self._task: asyncio.Task | None = None

    def trigger(self, decision: Decision) -> None:
        """Snapshot `decision` and schedule count-1 burst sends.
        Re-trigger during an in-flight burst overwrites both the
        snapshot and the counter (alink semantics)."""
        if not self._cfg.enabled:
            return
        self._latest_decision = decision
        self._remaining = max(0, self._cfg.count - 1)
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._pump())

    async def _pump(self) -> None:
        while self._remaining > 0:
            await asyncio.sleep(self._cfg.interval_ms / 1000.0)
            if self._latest_decision is None or self._remaining <= 0:
                break
            packet = self._encoder.encode(
                self._latest_decision, idr_request=True,
            )
            self._return_link.send(packet)
            self._remaining -= 1
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest tests/test_idr_burst.py -v`
Expected: 4 passed.

The wire module deliberately exposes no decision-packet decoder (the Python side only encodes; the C side decodes). The tests above parse the two fields they need by raw byte offset — `flags` at byte 5, `sequence` at bytes 8-12 big-endian — to avoid coupling test code to an internal decoder we don't have.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/idr_burst.py tests/test_idr_burst.py
git commit -m "$(cat <<'EOF'
gs: add IDR burst sender (alink keyframe-pump port)

When loss is detected, the regular tick-rate decision path sends
one IDR-flagged packet. IdrBurster schedules count-1 additional
copies at interval_ms spacing on asyncio so the burst survives
lossy UDP on the return link.

Wire format unchanged. Each burst packet is a fresh-sequence DLK1
decision frame with FLAG_IDR_REQUEST set; drone-side per-backend
diff apply makes the radio/encoder reapplication a no-op while
dl_backend_enc_request_idr throttles HTTP calls so the burst
collapses to one camera API call.

Defaults from alink_gs.conf [keyframe]: count=4, interval_ms=20.

Wired into service.py in a follow-up commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 8: Wire `IdrBurster` into `service.py` + add `gs.yaml` block

**Files:**
- Modify: `gs/dynamic_link/service.py`
- Modify: `conf/gs.yaml.sample`

- [ ] **Step 1: Add `IdrBurstConfig` parsing in `_build_policy_config` (or a sibling helper)**

Open `gs/dynamic_link/service.py`. Add to the imports:

```python
from .idr_burst import IdrBurstConfig, IdrBurster
```

Decision: `IdrBurstConfig` is consumed at the service layer, not inside `Policy`. Don't add it to `PolicyConfig`. Instead, parse it standalone alongside the existing config helpers.

Add a small helper near the other `_build_*` functions:

```python
def _build_idr_burst_config(raw: dict) -> IdrBurstConfig:
    policy_raw = raw.get("policy", {})
    burst_raw = policy_raw.get("idr_burst", {})
    cfg = IdrBurstConfig(
        enabled=bool(burst_raw.get("enabled", True)),
        count=int(burst_raw.get("count", 4)),
        interval_ms=int(burst_raw.get("interval_ms", 20)),
    )
    if cfg.count < 1:
        raise ValueError(
            f"policy.idr_burst.count must be >= 1; got {cfg.count}"
        )
    if cfg.interval_ms <= 0:
        raise ValueError(
            f"policy.idr_burst.interval_ms must be > 0; got {cfg.interval_ms}"
        )
    return cfg
```

- [ ] **Step 2: Construct the burster and call it from the per-tick send path**

In `_run`, find the "Phase 2 wiring" block (around lines 367-378). After `wire_encoder = WireEncoder(seq=1)` is set up, construct the burster:

```python
    idr_burst_cfg = _build_idr_burst_config(raw)
    idr_burster: IdrBurster | None = None
    if enabled and return_link is not None and wire_encoder is not None:
        idr_burster = IdrBurster(idr_burst_cfg, return_link, wire_encoder)
        log.info(
            "idr_burst: enabled=%s count=%d interval_ms=%d",
            idr_burst_cfg.enabled, idr_burst_cfg.count,
            idr_burst_cfg.interval_ms,
        )
```

The exact placement: this needs to run after `enabled`, `return_link`, and `wire_encoder` are all defined. The block after the existing `if enabled: ... else: log.info("enabled=false; ...")` is the right spot.

Find the `on_event` closure (around line 453). Locate the per-tick send (currently around lines 471-473):

```python
            if enabled and return_link is not None and wire_encoder is not None:
                packet = wire_encoder.encode(decision)
                return_link.send(packet)
```

Replace with:

```python
            if enabled and return_link is not None and wire_encoder is not None:
                packet = wire_encoder.encode(decision)
                return_link.send(packet)
                if decision.idr_request and idr_burster is not None:
                    idr_burster.trigger(decision)
```

- [ ] **Step 3: Add the `policy.idr_burst` block to `gs.yaml.sample`**

Open `conf/gs.yaml.sample`. Find the `policy:` block (added in Task 5). Add `idr_burst:` as a sibling of `bitrate:`:

```yaml
policy:
  bitrate:
    utilization_factor: 0.8
    min_bitrate_kbps: 1000
    max_bitrate_kbps: 24000
  # On any tick where the policy requests an IDR (residual_loss > 0),
  # send `count` total IDR-flagged decision packets at `interval_ms`
  # spacing — burst the regular tick packet plus count-1 fast repeats
  # so a single UDP loss on the return link doesn't swallow the IDR.
  # Drone-side dl_backend_enc throttle collapses duplicate arrivals
  # to one camera HTTP call. Defaults from alink_gs.conf [keyframe].
  idr_burst:
    enabled: true
    count: 4
    interval_ms: 20
```

- [ ] **Step 4: Run the full test suite to catch regressions**

Run: `python3 -m pytest -v`
Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/service.py conf/gs.yaml.sample
git commit -m "$(cat <<'EOF'
gs: wire IdrBurster into the service per-tick send path

When the policy emits a Decision with idr_request=True, the regular
tick-rate path sends one packet and IdrBurster.trigger() schedules
the configured number of follow-up copies on asyncio at the
configured spacing. New gs.yaml block: policy.idr_burst.{enabled,
count, interval_ms} with alink defaults (true, 4, 20).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 9: Extend `tests/test_drone_e2e.py` to assert end-to-end IDR burst

**Files:**
- Modify: `tests/test_drone_e2e.py`

- [ ] **Step 1: Find an existing loss-scenario test to extend**

Run: `grep -n "idr\|residual_loss\|FLAG_IDR" tests/test_drone_e2e.py`

Pick the test that drives a loss scenario into the real `dl-applier` (likely one that already asserts a single IDR HTTP call lands on the mock encoder). Identify it by name — call it `test_loss_triggers_idr_*` for this plan's purposes.

- [ ] **Step 2: Add a new test that asserts ≥ 2 IDR-flagged packets reach the applier**

Add this test alongside the existing one. The pattern: drive multiple consecutive ticks of loss into the policy, assert the applier's input UDP socket received at least 2 IDR-flagged decision packets within a short window (≤ 200 ms — enough for one burst plus jitter). Be tolerant — exact count depends on UDP/asyncio timing under CI.

If the existing harness already counts decision-packet arrivals, extend it to count packets-with-IDR-flag-set as a separate metric. Otherwise, in the mock applier-side socket reader, decode `flags` (byte offset 5 in the wire frame; `0x01 = FLAG_IDR_REQUEST`) and bump a counter.

Sketch:

```python
async def test_idr_burst_delivers_multiple_packets(sandbox):
    # ... existing fixture sets up the policy, return_link, mock applier
    # socket. Inject a single tick of loss to fire the burst.
    feed_loss_tick(sandbox)
    await asyncio.sleep(0.2)  # > burst window (60 ms) + slack
    idr_packets = [p for p in sandbox.received_packets
                   if p[5] & 0x01]
    assert len(idr_packets) >= 2, (
        f"expected ≥ 2 IDR-flagged packets in burst window, "
        f"got {len(idr_packets)} (total packets {len(sandbox.received_packets)})"
    )
```

The exact harness wiring will depend on what's already in `tests/test_drone_e2e.py`; adapt the fixture access pattern to match the existing tests in that file. **Do not assert exact count** — UDP loss simulation, asyncio drift, and CI load all introduce nondeterminism. `>= 2` is the meaningful assertion.

- [ ] **Step 3: Run the e2e test**

Run: `python3 -m pytest tests/test_drone_e2e.py -v -k idr`
Expected: PASS. If it flakes, increase the `asyncio.sleep` budget to 0.3 s; the burst itself completes in ~60 ms but the wfb-ng tunnel echo via the test harness can add latency.

- [ ] **Step 4: Run both full test suites end-to-end**

Run: `python3 -m pytest -v`
Run: `make -C drone test`
Expected: all green.

- [ ] **Step 5: Commit**

```bash
git add tests/test_drone_e2e.py
git commit -m "$(cat <<'EOF'
tests: assert IDR burst delivers ≥ 2 packets end-to-end

Extends the loss-scenario e2e test to verify that a single trigger
produces at least 2 IDR-flagged decision packets on the applier's
input socket within the 200 ms burst window. Tolerant assertion —
UDP timing under CI is nondeterministic.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Final verification

- [ ] **Run the full Python test suite from the repo root**

Run: `python3 -m pytest -v`
Expected: all tests pass.

- [ ] **Run the C unit tests**

Run: `make -C drone test`
Expected: all tests pass. (No drone-side changes were made — this is a regression check.)

- [ ] **Smoke-load the sample GS config**

Run: `python3 -m dynamic_link.service --config conf/gs.yaml.sample --replay /dev/null --log-level WARNING ; echo "exit=$?"`
Expected: process exits with non-zero (`/dev/null` is not a valid replay file) but no `KeyError` / `ValueError` / `ProfileError` in stderr — config parses cleanly.

- [ ] **Verify no `bitrate_Mbps` references remain in code paths**

Run: `grep -rn "bitrate_Mbps" gs/ conf/ tests/ docs/`
Expected: zero matches in `gs/` and `conf/`. In `tests/` only the rejection-test in `tests/test_profile.py` should mention it. In `docs/` the design doc may mention it in historical context — that's fine if framed as "previously".

- [ ] **Verify the spec → plan mapping**

Spec sections covered:
- §1 (dynamic bitrate) → Tasks 1-5
- §2 (IDR burst) → Tasks 7-9
- §3 (testing) → tests in Tasks 1, 2, 4, 7, 9
- §4 (migration) → Tasks 3, 5
- §6 (file list) → all tasks; matches.
