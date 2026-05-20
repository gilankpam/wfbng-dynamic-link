"""Bitrate is computed from PHY rate × utilization × k_over_n, where
k_over_n is derived from `base_redundancy_ratio` — NOT from live
(k, n) — to keep encoder allocation steady under n_escalation."""
from __future__ import annotations

from pathlib import Path

from dynamic_link.bitrate import (
    BitrateConfig, compute_bitrate_kbps, effective_phy_Mbps,
)
from dynamic_link.profile import load_profile_file


def _cfg(base_ratio: float = 0.25) -> BitrateConfig:
    return BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=base_ratio,  # k/n = 1/(1+0.25) = 0.8
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )


def _profile():
    return load_profile_file(Path("conf/radios/m8812eu2.yaml"))


def test_bitrate_uses_base_redundancy_ratio_not_live_kn():
    """Same inputs should produce the same bitrate regardless of any
    live `(k, n)` — the function takes no `(k, n)` argument."""
    p = _profile()
    cfg = _cfg(base_ratio=0.25)
    # PHY rate for MCS 5, BW 20 in m8812eu2: 39 Mb/s
    # bitrate = 39 * 1000 * 0.8 * (1/(1+0.25)) = 39 * 1000 * 0.8 * 0.8 = 24960
    # Clamped to max_bitrate_kbps=24000.
    assert compute_bitrate_kbps(p, 20, 5, cfg) == 24000


def test_bitrate_clamped_to_min():
    p = _profile()
    cfg = BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=0.5,
        min_bitrate_kbps=8000,    # raise the floor
        max_bitrate_kbps=24000,
    )
    # MCS 0 at BW 20 has PHY ~ 6.5 Mb/s → 6500 * 0.8 * (1/1.5) = 3466
    # Clamped up to 8000.
    assert compute_bitrate_kbps(p, 20, 0, cfg) == 8000


def test_bitrate_changes_with_base_ratio():
    p = _profile()
    a = compute_bitrate_kbps(p, 20, 4, _cfg(base_ratio=0.25))  # k/n = 0.80
    b = compute_bitrate_kbps(p, 20, 4, _cfg(base_ratio=0.50))  # k/n = 0.667
    assert b < a


def test_bitrate_bw40_higher_than_bw20_for_same_mcs():
    p = _profile()
    cfg = _cfg()
    a = compute_bitrate_kbps(p, 20, 4, cfg)
    b = compute_bitrate_kbps(p, 40, 4, cfg)
    assert b >= a


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
