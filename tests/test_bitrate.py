"""Bitrate is computed from PHY rate × utilization × k_over_n, where
k_over_n is derived from `base_redundancy_ratio` — NOT from live
(k, n) — to keep encoder allocation steady under n_escalation."""
from __future__ import annotations

import logging
from dataclasses import replace
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
    p = _profile()
    cfg = _cfg(base_ratio=0.25)
    # MCS=5 → PHY=52 Mb/s; mtu=1400, preamble=200 µs (DEFAULT — m8812eu2
    # hasn't gained the field yet at this commit; Task 5 calibrates it).
    # eff = 11200/(200e-6 + 11200/52e6) = 11200/415.38e-6 = 26.96 Mb/s
    # bitrate = 26963 * 0.8 / 1.25 = 17256 → no clamp
    assert compute_bitrate_kbps(p, 20, 5, 1400, cfg) == 17256


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


def test_compute_bitrate_kbps_warns_once_when_preamble_missing(caplog, monkeypatch):
    """Profile without preamble_us_per_frame: WARN once, use 200 µs default.

    monkeypatches the module-level dedup set so this test is hermetic and
    isn't affected by other tests that may have already populated the set.
    """
    monkeypatch.setattr("dynamic_link.bitrate._warned_missing_preamble", set())
    p = load_profile_file(Path("conf/radios/m8812eu2.yaml"))
    p = replace(p, preamble_us_per_frame=None, name="m8812eu2-noprmbl-1")

    cfg = _cfg(base_ratio=0.5)
    caplog.set_level(logging.WARNING, logger="dynamic_link.bitrate")
    compute_bitrate_kbps(p, 20, 4, 1400, cfg)
    compute_bitrate_kbps(p, 20, 4, 1400, cfg)   # second call — should NOT re-warn
    warnings = [r for r in caplog.records if "preamble_us_per_frame missing" in r.message]
    assert len(warnings) == 1
    assert "m8812eu2-noprmbl-1" in warnings[0].message
