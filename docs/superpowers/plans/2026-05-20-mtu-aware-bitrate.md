# MTU-aware encoder bitrate — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `gs/dynamic_link/bitrate.py::compute_bitrate_kbps`'s raw-PHY-rate model with an effective-PHY-rate model that accounts for per-802.11-frame fixed overhead and MTU. Encoder bitrate setpoint then respects the airtime ceiling at small `mlink` instead of overshooting it.

**Architecture:** One new pure helper (`effective_phy_Mbps`) drives the model. `RadioProfile` gains an optional `preamble_us_per_frame` field (the per-chipset constant). `compute_bitrate_kbps` gains an `mtu_bytes` argument and calls the helper. `policy.py` passes `drone_config.mtu_bytes` at call time. Profiles missing the new field fall back to a conservative 200 µs default with a one-shot WARNING.

**Tech Stack:** Python 3.11 stdlib (no new deps). pytest for tests. Existing YAML profile loader.

**Spec:** `docs/superpowers/specs/2026-05-20-mtu-aware-bitrate-design.md`
**Bench (validates the model):** `docs/mlink-airtime-bench.md`

---

## File map

- **Modify** `gs/dynamic_link/profile.py` — add `preamble_us_per_frame: float | None = None` to `RadioProfile`; loader reads it if present.
- **Modify** `gs/dynamic_link/bitrate.py` — add `effective_phy_Mbps()` helper, `DEFAULT_PREAMBLE_US` constant, one-shot warning state, `mtu_bytes` parameter on `compute_bitrate_kbps`.
- **Modify** `gs/dynamic_link/policy.py` — pass `drone_config.mtu_bytes` (with 1400 fallback) at lines 760 and 833.
- **Modify** `conf/radios/m8812eu2.yaml` — add `preamble_us_per_frame: 170`.
- **Modify** `tests/test_profile.py` — add case for profile YAML missing `preamble_us_per_frame`.
- **Modify** `tests/test_bitrate.py` — pass `mtu_bytes` through existing tests; add bench-anchored numerical tests, monotonicity, clamp, default-fallback.
- **Modify** `tests/test_policy_bitrate.py` — update existing formula tests for effective-PHY math; add MCS4 mtu-comparison test.

No new files. No `gs.yaml` change. No wire-format change.

---

## Task 1: `RadioProfile` gains optional `preamble_us_per_frame`

**Files:**
- Modify: `gs/dynamic_link/profile.py` — `RadioProfile` dataclass + `load_profile_file` function
- Modify: `tests/test_profile.py` — add two tests

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_profile.py`:

```python
def test_load_profile_reads_preamble_us_per_frame(tmp_path):
    """When the YAML carries preamble_us_per_frame, it surfaces on the dataclass."""
    d = _valid_dict()
    d["preamble_us_per_frame"] = 157
    p = _write_profile(tmp_path, d)
    prof = load_profile_file(p)
    assert prof.preamble_us_per_frame == 157.0


def test_load_profile_preamble_us_per_frame_optional(tmp_path):
    """Profiles without the field load cleanly with preamble_us_per_frame=None."""
    d = _valid_dict()
    d.pop("preamble_us_per_frame", None)
    p = _write_profile(tmp_path, d)
    prof = load_profile_file(p)
    assert prof.preamble_us_per_frame is None
```

(Read the bottom of `tests/test_profile.py` to confirm `_valid_dict` and `_write_profile` helpers exist there — they're used elsewhere in the file.)

- [ ] **Step 2: Run the new tests and watch them fail**

```bash
python3 -m pytest tests/test_profile.py::test_load_profile_reads_preamble_us_per_frame \
                  tests/test_profile.py::test_load_profile_preamble_us_per_frame_optional -v
```

Expected: both FAIL — `AttributeError: 'RadioProfile' object has no attribute 'preamble_us_per_frame'`.

- [ ] **Step 3: Add the dataclass field**

In `gs/dynamic_link/profile.py`, modify the `RadioProfile` dataclass (around line 33):

```python
@dataclass(frozen=True)
class RadioProfile:
    """Parsed, validated radio profile — see design doc §6.1."""
    name: str
    chipset: str
    mcs_min: int
    mcs_max: int
    bandwidth_supported: tuple[int, ...]
    bandwidth_default: int
    tx_power_min_dBm: int
    tx_power_max_dBm: int
    snr_floor_dB: dict[int, dict[int, float]]    # bw -> mcs -> dB
    data_rate_Mbps_LGI: dict[int, dict[int, float]]
    preamble_us_per_frame: float | None = None
```

The field MUST stay at the end with a default so existing constructions without it (in tests) keep working.

- [ ] **Step 4: Wire the loader**

In `gs/dynamic_link/profile.py`, modify the `return RadioProfile(...)` call at the end of `load_profile_file` (around line 150) to pass the new field:

```python
    preamble_us_per_frame = data.get("preamble_us_per_frame")
    if preamble_us_per_frame is not None:
        preamble_us_per_frame = float(preamble_us_per_frame)

    return RadioProfile(
        name=str(req("name")),
        chipset=str(req("chipset")),
        mcs_min=mcs_min,
        mcs_max=mcs_max,
        bandwidth_supported=bandwidth_supported,
        bandwidth_default=bandwidth_default,
        tx_power_min_dBm=tx_min,
        tx_power_max_dBm=tx_max,
        snr_floor_dB=snr_floor,
        data_rate_Mbps_LGI=data_rate,
        preamble_us_per_frame=preamble_us_per_frame,
    )
```

- [ ] **Step 5: Run the new tests and verify they pass**

```bash
python3 -m pytest tests/test_profile.py -v
```

Expected: all `test_profile.py` tests pass (including the two new ones).

- [ ] **Step 6: Commit**

```bash
git add gs/dynamic_link/profile.py tests/test_profile.py
git commit -m "$(cat <<'EOF'
profile: add optional preamble_us_per_frame field

Per-chipset 802.11 per-frame fixed overhead (PHY preamble + MAC
header at PHY rate + DIFS/backoff). Sourced from the radio
profile so the bitrate model can derive effective PHY rate per
MTU. Optional with default None for backward compatibility;
bitrate.py will fall back to a conservative constant when absent.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: `effective_phy_Mbps` helper

**Files:**
- Modify: `gs/dynamic_link/bitrate.py` — add the helper
- Modify: `tests/test_bitrate.py` — add helper-only unit tests

- [ ] **Step 1: Write the failing tests**

At the top of `tests/test_bitrate.py`, add `effective_phy_Mbps` to the imports:

```python
from dynamic_link.bitrate import (
    BitrateConfig, compute_bitrate_kbps, effective_phy_Mbps,
)
```

Append to `tests/test_bitrate.py`:

```python
def test_effective_phy_Mbps_mlink_1500_mcs4():
    """Pure-math anchor: 39 Mb/s PHY, mtu=1500, preamble=170 µs
    → effective ~25.12 Mb/s. (Choice of preamble=170 is what the
    calibrated m8812eu2 profile uses; see Task 5 for why 170 vs the
    bench doc's raw 157 µs.)"""
    eff = effective_phy_Mbps(phy_Mbps=39.0, mtu_bytes=1500, preamble_us=170.0)
    assert abs(eff - 25.12) < 0.05


def test_effective_phy_Mbps_mlink_3994_mcs4():
    """Pure-math anchor: 39 Mb/s PHY, mtu=3994, preamble=170 µs
    → effective ~32.30 Mb/s."""
    eff = effective_phy_Mbps(phy_Mbps=39.0, mtu_bytes=3994, preamble_us=170.0)
    assert abs(eff - 32.30) < 0.05


def test_effective_phy_Mbps_approaches_raw_at_large_mtu():
    """As MTU → ∞, effective rate approaches raw PHY rate."""
    raw = 39.0
    eff_huge = effective_phy_Mbps(phy_Mbps=raw, mtu_bytes=64_000, preamble_us=170.0)
    assert eff_huge < raw
    assert (raw - eff_huge) / raw < 0.05   # within 5%


def test_effective_phy_Mbps_monotone_in_mtu():
    """Bigger packets are at least as efficient as smaller ones."""
    e500  = effective_phy_Mbps(39.0, 500,  170.0)
    e1500 = effective_phy_Mbps(39.0, 1500, 170.0)
    e3994 = effective_phy_Mbps(39.0, 3994, 170.0)
    assert e500 < e1500 < e3994
```

- [ ] **Step 2: Run the new tests, watch them fail**

```bash
python3 -m pytest tests/test_bitrate.py -v -k effective_phy_Mbps
```

Expected: collection error (`ImportError: cannot import name 'effective_phy_Mbps'`).

- [ ] **Step 3: Implement the helper**

In `gs/dynamic_link/bitrate.py`, **above** `compute_bitrate_kbps`, add:

```python
def effective_phy_Mbps(
    phy_Mbps: float, mtu_bytes: int, preamble_us: float,
) -> float:
    """Per-packet airtime model. Returns the wire bandwidth a
    sustained stream of `mtu_bytes` packets can actually achieve at
    this PHY rate, given `preamble_us` of fixed per-frame overhead.

    See `docs/mlink-airtime-bench.md` for derivation and calibration.
    """
    mtu_bits = mtu_bytes * 8
    preamble_s = preamble_us * 1e-6
    payload_s = mtu_bits / (phy_Mbps * 1_000_000.0)
    return mtu_bits / (preamble_s + payload_s) / 1_000_000.0
```

- [ ] **Step 4: Run the new tests, verify they pass**

```bash
python3 -m pytest tests/test_bitrate.py -v -k effective_phy_Mbps
```

Expected: all four new tests PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/bitrate.py tests/test_bitrate.py
git commit -m "$(cat <<'EOF'
bitrate: add effective_phy_Mbps helper

Pure airtime model: mtu_bits / (preamble_s + mtu_bits/phy_bps).
Bench-anchored unit tests cover the two empirical anchors from
docs/mlink-airtime-bench.md (MCS4 + mlink ∈ {1500, 3994}) plus
monotonicity and large-MTU asymptote.

Not wired into compute_bitrate_kbps yet — that's the next commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: `compute_bitrate_kbps` becomes MTU-aware

This task changes a function signature. **All call sites and tests must update in the same commit** or `pytest` won't collect. The change keeps existing behaviour by hard-coding `mtu_bytes=1400` everywhere; Task 6 swaps in the real value from `drone_config`.

**Files:**
- Modify: `gs/dynamic_link/bitrate.py`
- Modify: `gs/dynamic_link/policy.py` (two call sites, lines ~760 and ~833)
- Modify: `tests/test_bitrate.py` (four existing tests get an `mtu_bytes` arg)
- Modify: `tests/test_policy_bitrate.py` (two existing tests get an `mtu_bytes` arg + formula update)

- [ ] **Step 1: Write the failing test for default + warning**

Append to `tests/test_bitrate.py`:

```python
def test_compute_bitrate_kbps_warns_once_when_preamble_missing(caplog, tmp_path):
    """Profile without preamble_us_per_frame: WARN once, use 200 µs default."""
    import logging
    from dynamic_link.profile import load_profile_file
    # m8812eu2.yaml currently has no preamble_us_per_frame; copy minus the
    # field if/when the packaged profile gains it. For now load it directly:
    p = load_profile_file(Path("conf/radios/m8812eu2.yaml"))
    # Force the field to None even if the packaged profile has gained it,
    # to make this test stable across the rollout:
    from dataclasses import replace
    p = replace(p, preamble_us_per_frame=None, name="m8812eu2-noprmbl-1")

    cfg = _cfg(base_ratio=0.5)
    caplog.set_level(logging.WARNING, logger="dynamic_link.bitrate")
    compute_bitrate_kbps(p, 20, 4, 1400, cfg)
    compute_bitrate_kbps(p, 20, 4, 1400, cfg)   # second call — should NOT re-warn
    warnings = [r for r in caplog.records if "preamble_us_per_frame missing" in r.message]
    assert len(warnings) == 1
    assert "m8812eu2-noprmbl-1" in warnings[0].message
```

(The unique `name=` suffix avoids cross-test contamination of the module-level "warned" set; each new test that wants a fresh warn should use its own suffix.)

- [ ] **Step 2: Run that test, watch it fail**

```bash
python3 -m pytest tests/test_bitrate.py::test_compute_bitrate_kbps_warns_once_when_preamble_missing -v
```

Expected: `TypeError: compute_bitrate_kbps() takes 4 positional arguments but 5 were given` — confirms signature update needed.

- [ ] **Step 3: Update `bitrate.py` — signature, default, warn helper, formula**

Rewrite `gs/dynamic_link/bitrate.py`. Replace the module-level imports + the existing `compute_bitrate_kbps` with:

```python
"""Encoder bitrate from effective PHY rate × utilization × k_over_n.

The model accounts for per-802.11-frame fixed overhead so that
encoder bitrate scales correctly with `mtu_bytes`. See:
- docs/mlink-airtime-bench.md (model + calibration)
- docs/superpowers/specs/2026-05-20-mtu-aware-bitrate-design.md
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

from .profile import RadioProfile

_log = logging.getLogger(__name__)

# Conservative fallback when a radio profile carries no calibrated
# preamble_us_per_frame. 200 µs is ~27% higher than the measured
# 157 µs on the BL-M8812EU2, biasing toward lower encoder bitrate
# (never higher) for uncalibrated chipsets.
DEFAULT_PREAMBLE_US: float = 200.0

# Module-level set so the WARNING fires at most once per profile name
# even though compute_bitrate_kbps is called every tick (10 Hz).
_warned_missing_preamble: set[str] = set()


def _resolve_preamble_us(profile: RadioProfile) -> float:
    if profile.preamble_us_per_frame is not None:
        return profile.preamble_us_per_frame
    if profile.name not in _warned_missing_preamble:
        _warned_missing_preamble.add(profile.name)
        _log.warning(
            "preamble_us_per_frame missing from radio profile %r; "
            "using conservative default %.0f µs. "
            "See docs/mlink-airtime-bench.md.",
            profile.name, DEFAULT_PREAMBLE_US,
        )
    return DEFAULT_PREAMBLE_US


def effective_phy_Mbps(
    phy_Mbps: float, mtu_bytes: int, preamble_us: float,
) -> float:
    """Per-packet airtime model. Returns the wire bandwidth a
    sustained stream of `mtu_bytes` packets can actually achieve at
    this PHY rate, given `preamble_us` of fixed per-frame overhead.
    """
    mtu_bits = mtu_bytes * 8
    preamble_s = preamble_us * 1e-6
    payload_s = mtu_bits / (phy_Mbps * 1_000_000.0)
    return mtu_bits / (preamble_s + payload_s) / 1_000_000.0
```

(Keep the existing `BitrateConfig` dataclass and its `__post_init__` validation as-is.)

Then replace `compute_bitrate_kbps` with:

```python
def compute_bitrate_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    mtu_bytes: int,
    cfg: BitrateConfig,
) -> int:
    """Compute encoder bitrate target in kb/s for `(bandwidth, mcs, mtu_bytes)`."""
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    preamble_us = _resolve_preamble_us(profile)
    eff_phy_Mbps = effective_phy_Mbps(phy_Mbps, mtu_bytes, preamble_us)
    k_over_n = 1.0 / (1.0 + cfg.base_redundancy_ratio)
    raw_kbps = eff_phy_Mbps * 1000.0 * cfg.utilization_factor * k_over_n
    return int(max(cfg.min_bitrate_kbps,
                   min(cfg.max_bitrate_kbps, raw_kbps)))
```

- [ ] **Step 4: Update the four existing tests in `tests/test_bitrate.py`**

Pass `mtu_bytes=1400` to every existing call. The previously-asserted numeric values must change to reflect the effective-PHY math; recompute each.

With `preamble_us=DEFAULT_PREAMBLE_US=200.0` (because `conf/radios/m8812eu2.yaml` still has no field — Task 5 adds it):

```python
def test_bitrate_uses_base_redundancy_ratio_not_live_kn():
    p = _profile()
    cfg = _cfg(base_ratio=0.25)
    # MCS=5 → PHY=52 Mb/s; mtu=1400, preamble=200 µs (DEFAULT — m8812eu2
    # hasn't gained the field yet at this commit; Task 5 calibrates it).
    # eff = 11200/(200e-6 + 11200/52e6) = 11200/415.38e-6 = 26.96 Mb/s
    # bitrate = 26963 * 0.8 / 1.25 = 17256 → no clamp
    assert compute_bitrate_kbps(p, 20, 5, 1400, cfg) == 17256
```

For the other three tests:

```python
def test_bitrate_clamped_to_min():
    p = _profile()
    cfg = BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=0.5,
        min_bitrate_kbps=8000,
        max_bitrate_kbps=24000,
    )
    # MCS 0 PHY=6.5 Mb/s, mtu=1400, preamble=200 → eff ≈ 5.82, raw ≈ 3106 → clamped to 8000
    assert compute_bitrate_kbps(p, 20, 0, 1400, cfg) == 8000


def test_bitrate_changes_with_base_ratio():
    p = _profile()
    a = compute_bitrate_kbps(p, 20, 4, 1400, _cfg(base_ratio=0.25))
    b = compute_bitrate_kbps(p, 20, 4, 1400, _cfg(base_ratio=0.50))
    assert b < a


def test_bitrate_bw40_higher_than_bw20_for_same_mcs():
    p = _profile()
    cfg = _cfg()
    a = compute_bitrate_kbps(p, 20, 4, 1400, cfg)
    b = compute_bitrate_kbps(p, 40, 4, 1400, cfg)
    assert b >= a
```

- [ ] **Step 5: Update the two existing tests in `tests/test_policy_bitrate.py`**

```python
def test_bitrate_matches_formula_for_every_row(profile):
    """For each MCS row, compute_bitrate_kbps matches
    eff_phy * util * 1/(1+base_ratio)."""
    from dynamic_link.bitrate import (
        DEFAULT_PREAMBLE_US, effective_phy_Mbps,
    )
    cfg = _cfg(base_ratio=0.5)
    k_over_n = 1.0 / (1.0 + cfg.base_redundancy_ratio)
    mtu_bytes = 1400
    # Track whatever the profile carries; falls back to DEFAULT when the
    # field is None. Keeps this test stable across Task 5 (which sets the
    # packaged profile to 170 µs).
    preamble_us = (
        profile.preamble_us_per_frame
        if profile.preamble_us_per_frame is not None
        else DEFAULT_PREAMBLE_US
    )
    rows = profile.snr_mcs_map(bandwidth=20, snr_margin_db=0.0)
    for row in rows:
        phy = profile.data_rate_Mbps_LGI[20][row.mcs]
        eff = effective_phy_Mbps(phy, mtu_bytes, preamble_us)
        expected = int(max(
            cfg.min_bitrate_kbps,
            min(cfg.max_bitrate_kbps, eff * 1000 * cfg.utilization_factor * k_over_n),
        ))
        got = compute_bitrate_kbps(profile, 20, row.mcs, mtu_bytes, cfg)
        assert got == expected, (
            f"mcs={row.mcs}: expected {expected}, got {got}"
        )


def test_bitrate_higher_mcs_has_higher_bitrate(profile):
    cfg = _cfg()
    a = compute_bitrate_kbps(profile, 20, 1, 1400, cfg)
    b = compute_bitrate_kbps(profile, 20, 5, 1400, cfg)
    assert b > a
```

- [ ] **Step 6: Update the two `compute_bitrate_kbps` call sites in `policy.py`**

At policy.py:760 (`Policy.__init__`):

```python
            bitrate_kbps=compute_bitrate_kbps(
                profile, cfg.leading.bandwidth, row.mcs, 1400, cfg.bitrate,
            ),
```

At policy.py:833 (`Policy.tick`):

```python
        new_bitrate_kbps = compute_bitrate_kbps(
            self.profile, self.state.bandwidth, row.mcs, 1400, self.cfg.bitrate,
        )
```

(Hard-coded `1400` is the same fallback the predictor uses on the next line. Task 6 replaces it with `drone_config.mtu_bytes`.)

- [ ] **Step 7: Run the full GS test suite**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py -v
```

Expected: green. The previously-recorded numeric assertions in the four `test_bitrate.py` tests should match the new function output (Step 4 told you to paste real values).

- [ ] **Step 8: Commit**

```bash
git add gs/dynamic_link/bitrate.py gs/dynamic_link/policy.py \
        tests/test_bitrate.py tests/test_policy_bitrate.py
git commit -m "$(cat <<'EOF'
bitrate: compute_bitrate_kbps takes mtu_bytes

Effective PHY rate now factors per-frame overhead, computed via
the new effective_phy_Mbps helper. Profile.preamble_us_per_frame
sources the per-chipset constant; profiles without the field
warn once and fall back to a 200 µs default.

policy.py call sites pass mtu_bytes=1400 as a placeholder — the
next commit wires the real value from drone_config.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Bench-anchored numerical tests for the full formula

**Files:**
- Modify: `tests/test_bitrate.py` — add four anchored tests

- [ ] **Step 1: Write the new tests**

Append to `tests/test_bitrate.py`:

```python
def test_compute_bitrate_kbps_mcs4_mlink_1500_bench_anchor():
    """Anchored to docs/mlink-airtime-bench.md table cell:
    MCS4 HT20 + mlink=1500 + U=0.8 + n/k=1.4 → ~14400 kbps."""
    from dataclasses import replace
    p = _profile()
    p = replace(p, preamble_us_per_frame=170.0, name="m8812eu2-bench-1")
    cfg = BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=0.4,    # k/n = 1/1.4
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )
    # eff = 12000/(170e-6 + 12000/39e6) ≈ 25.12 Mb/s
    # bitrate = 25120 * 0.8 / 1.4 ≈ 14353 → within ±300 of published 14400
    got = compute_bitrate_kbps(p, 20, 4, 1500, cfg)
    assert 14100 <= got <= 14700, f"expected ~14400, got {got}"


def test_compute_bitrate_kbps_mcs4_mlink_3994_bench_anchor():
    """Anchored to docs/mlink-airtime-bench.md table cell:
    MCS4 HT20 + mlink=3994 + U=0.8 + n/k=1.4 → ~18600 kbps."""
    from dataclasses import replace
    p = _profile()
    p = replace(p, preamble_us_per_frame=170.0, name="m8812eu2-bench-2")
    cfg = BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=0.4,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )
    # eff = 31952/(170e-6 + 31952/39e6) ≈ 32.30 Mb/s
    # bitrate = 32300 * 0.8 / 1.4 ≈ 18457 → within ±300 of published 18600
    got = compute_bitrate_kbps(p, 20, 4, 3994, cfg)
    assert 18300 <= got <= 18900, f"expected ~18600, got {got}"


def test_compute_bitrate_kbps_monotone_in_mtu():
    """Larger MTU → at least as much encoder bitrate (more airtime efficiency)."""
    from dataclasses import replace
    p = _profile()
    p = replace(p, preamble_us_per_frame=170.0, name="m8812eu2-mono")
    cfg = _cfg(base_ratio=0.4)
    vals = [compute_bitrate_kbps(p, 20, 4, m, cfg) for m in (500, 1500, 3994, 8000)]
    assert vals[0] < vals[1] < vals[2] < vals[3], vals


def test_compute_bitrate_kbps_max_clamp_still_applies():
    """At high MCS + large MTU + low cap, the cap wins."""
    from dataclasses import replace
    p = _profile()
    p = replace(p, preamble_us_per_frame=170.0, name="m8812eu2-clamp")
    cfg = BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=0.4,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=12000,    # well below natural MCS5 setpoint
    )
    assert compute_bitrate_kbps(p, 20, 5, 3994, cfg) == 12000
```

- [ ] **Step 2: Run the new tests, verify they pass**

```bash
python3 -m pytest tests/test_bitrate.py -v -k "bench_anchor or monotone_in_mtu or max_clamp"
```

Expected: all four PASS. The anchor tests' assertions came directly from the published bench table, so a mismatch here means either (a) the formula is wrong, or (b) the table needs an update.

- [ ] **Step 3: Commit**

```bash
git add tests/test_bitrate.py
git commit -m "$(cat <<'EOF'
bitrate: bench-anchored tests for compute_bitrate_kbps

Pin the formula against the published table in
docs/mlink-airtime-bench.md (MCS4 + mlink ∈ {1500, 3994}),
plus monotonicity-in-MTU and the max_bitrate_kbps clamp.
Future formula edits that break the bench numbers fail here
and point at the bench doc.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Calibrate the packaged `m8812eu2` profile

**Files:**
- Modify: `conf/radios/m8812eu2.yaml`
- Modify: `tests/test_bitrate.py` (four tests' expected values shift now that the packaged profile carries 157 µs instead of 200 µs default)
- Modify: `tests/test_policy_bitrate.py` (formula test uses 157 µs now)

- [ ] **Step 1: Add the field to the packaged profile**

Edit `conf/radios/m8812eu2.yaml`. Add immediately after `data_rate_Mbps_LGI:` block (keep it next to the other RF-physics tables):

```yaml
# Per-802.11-frame fixed overhead (PHY preamble + MAC header at PHY rate
# + DIFS/backoff), broadcast/no-ACK. Bench-fitted 2026-05-20 against the
# MCS3 + mlink=1500 saturation crossover on this chipset; see
# docs/mlink-airtime-bench.md for method and refit procedure.
#
# The bench's raw physical fit is 157 µs against the on-air payload
# (naluSize = mlink - 100 = 1400 B). The implementation feeds
# `mtu_bytes = wireless.mlink` straight into compute_bitrate_kbps
# (1500 B in that example), so this constant absorbs the small
# OpenIPC framing offset between mlink and naluSize. Net: 170 µs
# here reproduces the published table cells when called with
# mtu_bytes = mlink.
#
# Re-measure if you change the wfb_tx ACK policy or move to HT40 —
# the constant is bandwidth-and-mode-dependent.
preamble_us_per_frame: 170
```

- [ ] **Step 2: Run the full suite, expect breakage**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py -v
```

Expected: `test_bitrate.py::test_bitrate_uses_base_redundancy_ratio_not_live_kn` fails with a numeric drift (was computed against 200 µs default, packaged profile now carries 170). `test_policy_bitrate.py::test_bitrate_matches_formula_for_every_row` still passes — its expected-value computation reads `profile.preamble_us_per_frame` so it tracks whatever the YAML says.

- [ ] **Step 3: Update the broken test to use the calibrated preamble**

In `tests/test_bitrate.py::test_bitrate_uses_base_redundancy_ratio_not_live_kn`, update the comment and the expected literal:

```python
def test_bitrate_uses_base_redundancy_ratio_not_live_kn():
    p = _profile()
    cfg = _cfg(base_ratio=0.25)
    # MCS=5 → PHY=52 Mb/s; mtu=1400, preamble=170 µs (now calibrated in the packaged profile).
    # eff = 11200/(170e-6 + 11200/52e6) = 11200/385.38e-6 = 29.06 Mb/s
    # bitrate = 29061 * 0.8 / 1.25 = 18599 → no clamp
    assert compute_bitrate_kbps(p, 20, 5, 1400, cfg) == 18599
```

No change needed in `tests/test_policy_bitrate.py::test_bitrate_matches_formula_for_every_row` — it already tracks `profile.preamble_us_per_frame` with a `DEFAULT_PREAMBLE_US` fallback, so it auto-picks up the new 170 µs value from the YAML.

The other three `test_bitrate.py` tests (clamp, base_ratio, bw40-vs-bw20) test relative invariants, not absolute numbers — they should still pass with no change.

- [ ] **Step 4: Run the full suite, verify green**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py -v
```

Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add conf/radios/m8812eu2.yaml tests/test_bitrate.py tests/test_policy_bitrate.py
git commit -m "$(cat <<'EOF'
radios: calibrate m8812eu2 preamble_us_per_frame to 170 µs

Bench-fitted against the MCS3 + mlink=1500 saturation crossover
captured 2026-05-20; see docs/mlink-airtime-bench.md for the data
and refit procedure. The bench's raw 157 µs fit was against the
on-air payload (naluSize = mlink - 100); the implementation feeds
mtu_bytes = mlink straight through, so this constant absorbs the
~13 µs OpenIPC framing offset and reproduces the published table
when called with mtu_bytes = mlink.

Tests pinning hand-computed numeric values shift accordingly; the
relative-invariant tests are unaffected.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Wire `policy.py` to use `drone_config.mtu_bytes`

**Files:**
- Modify: `gs/dynamic_link/policy.py` (lines ~760 and ~833)
- Modify: `tests/test_policy_bitrate.py` (add MCS4 mtu-comparison test)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_policy_bitrate.py`:

```python
def test_bitrate_drops_when_mtu_shrinks(profile):
    """At MCS4, encoder bitrate is strictly lower for mtu=1500 than
    for mtu=3994 (smaller packets eat more airtime on overhead)."""
    cfg = _cfg(base_ratio=0.4)
    br_small = compute_bitrate_kbps(profile, 20, 4, 1500, cfg)
    br_large = compute_bitrate_kbps(profile, 20, 4, 3994, cfg)
    assert br_small < br_large, (br_small, br_large)
```

(Note: this is a `bitrate.py` integration test, not a `Policy` integration test — it exercises the same call shape `policy.tick` will make once Step 3 lands. Adding a full `Policy.tick`-driven test would require building a synced `DroneConfig` fixture and feeding signals; pure-formula coverage here is sufficient given Task 4 already pins the absolute numbers.)

- [ ] **Step 2: Run the new test, verify it passes**

```bash
python3 -m pytest tests/test_policy_bitrate.py::test_bitrate_drops_when_mtu_shrinks -v
```

Expected: PASS (the function-level behaviour is already correct from Task 3; this test asserts it). If it fails, the formula is broken — debug before continuing.

- [ ] **Step 3: Replace the `1400` placeholders in `policy.py`**

At policy.py:760 (`Policy.__init__`). Need to compute `mtu_for_init` from `drone_config` if present and synced; otherwise 1400:

```python
        mtu_for_init = (
            self.drone_config.mtu_bytes
            if self.drone_config is not None and self.drone_config.is_synced()
            else 1400
        )
        self.state = PolicyState(
            mcs=row.mcs,
            bandwidth=cfg.leading.bandwidth,
            tx_power_dBm=int(self.leading.state.tx_power_dBm),
            k=cfg.safe.k,
            n=cfg.safe.n,
            depth=cfg.safe.depth,
            bitrate_kbps=compute_bitrate_kbps(
                profile, cfg.leading.bandwidth, row.mcs, mtu_for_init, cfg.bitrate,
            ),
        )
```

(Note: at `__init__`, `self.drone_config` was just assigned a few lines above. Read policy.py:740-760 to confirm the field name and place this block correctly.)

At policy.py:833 (`Policy.tick`). The existing code reads `mtu` for the `compute_k` call a few lines below. Hoist that read up to share it with the bitrate call:

```python
        # mtu/fps come from drone HELLO when available; safe fallbacks otherwise.
        mtu = self.drone_config.mtu_bytes if self.drone_config else 1400
        fps = self.drone_config.fps if self.drone_config else 60

        # Bitrate first (no FEC dependency — bitrate uses the fixed
        # `base_redundancy_ratio` from BitrateConfig, not the live k/n).
        new_bitrate_kbps = compute_bitrate_kbps(
            self.profile, self.state.bandwidth, row.mcs,
            mtu, self.cfg.bitrate,
        )

        # Dynamic FEC: k from packets-per-frame at the live bitrate,
        # n from base + escalation.
        candidate_k = compute_k(
            bitrate_kbps=new_bitrate_kbps,
            mtu_bytes=mtu,
            fps=fps,
            cfg=self.cfg.dynamic_fec,
        )
```

The previously-duplicated `mtu = … if self.drone_config else 1400` and `fps = … if self.drone_config else 60` reads below the `compute_k` call should be removed (now they're hoisted).

- [ ] **Step 4: Run the full suite**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py -v
```

Expected: green. No test should regress; the new MCS4-mtu-comparison test from Step 1 keeps passing.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/policy.py tests/test_policy_bitrate.py
git commit -m "$(cat <<'EOF'
policy: pass drone-reported mtu to compute_bitrate_kbps

Both call sites (Policy.__init__ and Policy.tick) now source MTU
from drone_config.mtu_bytes with a 1400 fallback for the pre-HELLO
window. tick() hoists the existing mtu/fps reads above the bitrate
call so both compute_bitrate_kbps and compute_k share the value.

End-to-end: when the drone advertises mlink=1500 via DLHE, the GS
no longer overshoots the airtime ceiling at high MCS — fixing the
latency symptom captured in docs/mlink-airtime-bench.md.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Full regression sweep + drone-side smoke

**Files:**
- None to modify.

- [ ] **Step 1: GS pytest suite**

```bash
cd /workspace && python3 -m pytest --ignore=tests/test_mavlink_status.py
```

Expected: 0 failures. If anything outside the bitrate/profile/policy area regressed, diagnose before claiming done.

- [ ] **Step 2: C unit tests (sanity — this change doesn't touch drone-side code, but the suite is fast)**

```bash
make -C drone test
```

Expected: green. The drone-side `dl-applier` is unchanged so this is a fast no-op verification.

- [ ] **Step 3: Replay smoke on a capture if one is available**

If `capture.jsonl` from a prior flight is on disk:

```bash
python3 -m dynamic_link.service \
    --config conf/gs.yaml.sample \
    --replay capture.jsonl \
    --log-dir /tmp/dl-flights
```

Expected: starts cleanly, no `preamble_us_per_frame missing` WARNING (sample radio profile carries the field after Task 5), bitrate decisions visible in the log. If no capture is available, skip this step.

- [ ] **Step 4: Final commit gate**

```bash
git log --oneline origin/master..HEAD
```

Expected: six commits from this plan (one per task). If anything looks off — extra commits, missing commits, unexpected file changes — investigate before declaring done.

- [ ] **Step 5: Don't push**

This plan does not push. The user reviews the local commits and pushes when satisfied.

---

## Out of scope (reminders from the spec)

- `gs/dynamic_link/predictor.py` keeps its airtime-blind model. Folding `profile.preamble_us_per_frame` into the predictor is the obvious follow-up; not done here.
- No new observability surface (no decision-packet field, no OSD, no flight-log column).
- No `gs.yaml` override for `preamble_us_per_frame`.
- No drone-side `pubq_free_page` reactive loop.
