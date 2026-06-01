"""Bitrate is computed from wire_target × k / n (live FEC params).
wire_target comes from PHY rate × utilization, MTU-corrected."""
from __future__ import annotations

from pathlib import Path

from dynamic_link.bitrate import (
    BitrateConfig, compute_bitrate_kbps, compute_wire_target_kbps, effective_phy_Mbps,
)
from dynamic_link.profile import load_profile_file


def _cfg() -> BitrateConfig:
    return BitrateConfig(
        utilization_factor=0.8,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )


def _profile():
    return load_profile_file(Path("conf/radios/m8812eu2.yaml"))



def test_effective_phy_Mbps_mlink_1500_mcs4():
    """Pure-math anchor: 39 Mb/s PHY, mtu=1500, preamble=170 µs
    → effective ~25.12 Mb/s. (Choice of preamble=170 is what the
    calibrated m8812eu2 profile uses; see Task 5 for why 170 vs the
    bench doc's raw 157 µs.)"""
    eff = effective_phy_Mbps(phy_Mbps=39.0, mtu_bytes=1500, preamble_us=170.0)
    assert abs(eff - 25.12) < 0.05


def test_effective_phy_Mbps_mlink_3994_mcs4():
    """Pure-math anchor: 39 Mb/s PHY, mtu=3994, preamble=170 µs
    → effective ~32.30 Mb/s. 3994 is the OpenIPC default
    `wireless.mlink` for this chipset (close to the wfb_tx
    compile-time Radio MTU cap of 3993)."""
    eff = effective_phy_Mbps(phy_Mbps=39.0, mtu_bytes=3994, preamble_us=170.0)
    assert abs(eff - 32.30) < 0.05


def test_effective_phy_Mbps_approaches_raw_at_large_mtu():
    """As MTU → ∞, effective rate approaches raw PHY rate."""
    raw = 39.0
    eff_huge = effective_phy_Mbps(phy_Mbps=raw, mtu_bytes=64_000, preamble_us=170.0)
    assert (raw - eff_huge) / raw < 0.05


def test_effective_phy_Mbps_monotone_in_mtu():
    """Bigger packets are at least as efficient as smaller ones."""
    e500  = effective_phy_Mbps(39.0, 500,  170.0)
    e1500 = effective_phy_Mbps(39.0, 1500, 170.0)
    e3994 = effective_phy_Mbps(39.0, 3994, 170.0)
    assert e500 < e1500 < e3994


def test_compute_wire_target_warns_once_when_preamble_missing(caplog, monkeypatch):
    """Profile without preamble_us_per_frame: WARN once, use 200 µs default."""
    from dataclasses import replace
    import logging
    monkeypatch.setattr("dynamic_link.bitrate._warned_missing_preamble", set())
    p = load_profile_file(Path("conf/radios/m8812eu2.yaml"))
    p = replace(p, preamble_us_per_frame=None, name="m8812eu2-noprmbl-1")
    caplog.set_level(logging.WARNING, logger="dynamic_link.bitrate")
    compute_wire_target_kbps(p, 20, 4, 1400, 0.8)
    compute_wire_target_kbps(p, 20, 4, 1400, 0.8)   # second call — should NOT re-warn
    warnings = [r for r in caplog.records if "preamble_us_per_frame missing" in r.message]
    assert len(warnings) == 1
    assert "m8812eu2-noprmbl-1" in warnings[0].message


def test_compute_bitrate_kbps_mcs4_mlink_1500_bench_anchor():
    """Bench-anchored end-to-end: MCS4 HT20 + mlink=1500 + U=0.8 + n/k=1.4
    → ~14400 kbps. Now via wire_target × k / n at k=10, n=14."""
    from dataclasses import replace
    p = _profile()
    p = replace(p, preamble_us_per_frame=170.0, name="m8812eu2-bench-1")
    wire = compute_wire_target_kbps(p, 20, 4, 1500, 0.8)
    got = compute_bitrate_kbps(wire, k=10, n=14, min_bitrate_kbps=1000, max_bitrate_kbps=24000)
    assert 14100 <= got <= 14700, f"expected ~14400, got {got}"


def test_compute_bitrate_kbps_mcs4_mlink_3994_bench_anchor():
    """Bench-anchored: MCS4 HT20 + mlink=3994 + U=0.8 + n/k=1.4 → ~18600 kbps."""
    from dataclasses import replace
    p = _profile()
    p = replace(p, preamble_us_per_frame=170.0, name="m8812eu2-bench-2")
    wire = compute_wire_target_kbps(p, 20, 4, 3994, 0.8)
    got = compute_bitrate_kbps(wire, k=10, n=14, min_bitrate_kbps=1000, max_bitrate_kbps=24000)
    assert 18300 <= got <= 18900, f"expected ~18600, got {got}"


def test_compute_wire_target_kbps_eff_phy_times_util():
    """wire_target = eff_phy × util × 1000 (scale to kbps)."""
    p = _profile()  # m8812eu2; preamble_us = 170 per packaged profile
    # MCS 0 HT20: phy=6.5 Mbps, mtu=1500
    # eff = 12000 / (170e-6 + 12000/6.5e6) = 12000 / (170+1846) µs ≈ 5.948 Mbps
    # wire_target = 5948 × 0.6 ≈ 3568.8 → 3568 kbps (int cast in caller)
    got = compute_wire_target_kbps(
        profile=p, bandwidth=20, mcs=0,
        mtu_bytes=1500, utilization_factor=0.6,
    )
    eff = effective_phy_Mbps(6.5, 1500, 170.0)
    expected = eff * 0.6 * 1000.0
    assert abs(got - expected) < 0.01, f"got={got} expected={expected}"


def test_compute_wire_target_kbps_is_deterministic():
    """same inputs always yield same output (no hidden state)."""
    p = _profile()
    a = compute_wire_target_kbps(p, 20, 5, 1500, 0.6)
    b = compute_wire_target_kbps(p, 20, 5, 1500, 0.6)
    assert a == b


def test_compute_wire_target_kbps_scales_with_util():
    p = _profile()
    a = compute_wire_target_kbps(p, 20, 5, 1500, 0.4)
    b = compute_wire_target_kbps(p, 20, 5, 1500, 0.8)
    assert abs(b - 2 * a) < 0.5  # exactly 2× modulo float precision


def test_compute_wire_target_kbps_rejects_invalid_util():
    """Mirrors BitrateConfig validation: util ∈ (0, 1] required."""
    import pytest
    p = _profile()
    with pytest.raises(ValueError, match="utilization_factor must be in"):
        compute_wire_target_kbps(p, 20, 5, 1500, 0.0)
    with pytest.raises(ValueError, match="utilization_factor must be in"):
        compute_wire_target_kbps(p, 20, 5, 1500, 1.5)
    with pytest.raises(ValueError, match="utilization_factor must be in"):
        compute_wire_target_kbps(p, 20, 5, 1500, -0.1)


def test_compute_bitrate_kbps_new_signature_basic():
    """bitrate = int(wire_target × k / n), clamped to [min, max]."""
    # wire=3568, k=4, n=6 → 3568*4/6 = 2378.67 → 2378
    got = compute_bitrate_kbps(
        wire_target_kbps=3568.0, k=4, n=6,
        min_bitrate_kbps=1000, max_bitrate_kbps=24000,
    )
    assert got == 2378


def test_compute_bitrate_kbps_new_signature_clamps_to_min():
    """When formula yields below floor, clamp to floor."""
    # wire=1000, k=2, n=10 → 200 < 1000 floor
    got = compute_bitrate_kbps(
        wire_target_kbps=1000.0, k=2, n=10,
        min_bitrate_kbps=1000, max_bitrate_kbps=24000,
    )
    assert got == 1000


def test_compute_bitrate_kbps_new_signature_clamps_to_max():
    """When formula yields above max, clamp to max."""
    # wire=30000, k=10, n=10 → 30000 → clamp to 24000
    got = compute_bitrate_kbps(
        wire_target_kbps=30000.0, k=10, n=10,
        min_bitrate_kbps=1000, max_bitrate_kbps=24000,
    )
    assert got == 24000


def test_compute_bitrate_kbps_shrinks_with_growing_n():
    """bitrate × n / k stays constant ≈ wire_target as n grows."""
    wire = 3568.0
    k = 4
    bitrates = [
        compute_bitrate_kbps(wire, k, n,
                             min_bitrate_kbps=1, max_bitrate_kbps=24000)
        for n in (4, 5, 6, 7, 8, 9, 10, 11, 12)
    ]
    # Strictly decreasing as n grows (k fixed)
    assert all(a > b for a, b in zip(bitrates, bitrates[1:])), bitrates
    # Wire rate (bitrate × n/k) stays ≤ wire_target for every n
    for n, br in zip(range(4, 13), bitrates):
        wire_actual = br * n / k
        assert wire_actual <= wire, f"n={n}: wire={wire_actual} > target={wire}"


def test_compute_bitrate_kbps_new_signature_rejects_invalid_inputs():
    """Defensive guards: k>0, n>=k, min_bitrate>0, max>=min."""
    import pytest
    with pytest.raises(ValueError, match="k must be > 0"):
        compute_bitrate_kbps(3568.0, k=0, n=6, min_bitrate_kbps=1000, max_bitrate_kbps=24000)
    with pytest.raises(ValueError, match=r"n \(3\) must be >= k \(4\)"):
        compute_bitrate_kbps(3568.0, k=4, n=3, min_bitrate_kbps=1000, max_bitrate_kbps=24000)
    with pytest.raises(ValueError, match="min_bitrate_kbps must be > 0"):
        compute_bitrate_kbps(3568.0, k=4, n=6, min_bitrate_kbps=0, max_bitrate_kbps=24000)
    with pytest.raises(ValueError, match="max_bitrate_kbps"):
        compute_bitrate_kbps(3568.0, k=4, n=6, min_bitrate_kbps=5000, max_bitrate_kbps=4000)
