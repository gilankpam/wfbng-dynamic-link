"""Bitrate per MCS row — sanity check of the new (k, n)-free formula."""
from __future__ import annotations

from pathlib import Path

import pytest

from dynamic_link.bitrate import BitrateConfig, compute_bitrate_kbps_legacy
from dynamic_link.profile import load_profile

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


@pytest.fixture
def profile():
    return load_profile("m8812eu2", [PACKAGED_DIR])


def _cfg(base_ratio: float = 0.5) -> BitrateConfig:
    return BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=base_ratio,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )


def test_bitrate_matches_formula_for_every_row(profile):
    """For each MCS row, compute_bitrate_kbps matches
    eff_phy * util * 1/(1+base_ratio)."""
    from dynamic_link.bitrate import (
        DEFAULT_PREAMBLE_US, effective_phy_Mbps,
    )
    cfg = _cfg(base_ratio=0.5)
    k_over_n = 1.0 / (1.0 + cfg.base_redundancy_ratio)
    mtu_bytes = 1400
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
        got = compute_bitrate_kbps_legacy(profile, 20, row.mcs, mtu_bytes, cfg)
        assert got == expected, (
            f"mcs={row.mcs}: expected {expected}, got {got}"
        )


def test_bitrate_higher_mcs_has_higher_bitrate(profile):
    cfg = _cfg()
    a = compute_bitrate_kbps_legacy(profile, 20, 1, 1400, cfg)
    b = compute_bitrate_kbps_legacy(profile, 20, 5, 1400, cfg)
    assert b > a


def test_bitrate_drops_when_mtu_shrinks(profile):
    """At MCS4, encoder bitrate is strictly lower for mtu=1500 than
    for mtu=3994 (smaller packets eat more airtime on overhead)."""
    cfg = _cfg(base_ratio=0.4)
    br_small = compute_bitrate_kbps_legacy(profile, 20, 4, 1500, cfg)
    br_large = compute_bitrate_kbps_legacy(profile, 20, 4, 3994, cfg)
    assert br_small < br_large, (br_small, br_large)
