"""Bitrate per MCS row — sanity check of the new (k, n)-free formula."""
from __future__ import annotations

from pathlib import Path

import pytest

from dynamic_link.bitrate import BitrateConfig, compute_bitrate_kbps
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
    phy * util * 1/(1+base_ratio)."""
    cfg = _cfg(base_ratio=0.5)
    k_over_n = 1.0 / (1.0 + cfg.base_redundancy_ratio)
    rows = profile._build_rows(bandwidth=20, rssi_margin_db=0,
                               snr_margin_db=0)
    for row in rows:
        phy = profile.data_rate_Mbps_LGI[20][row.mcs]
        expected = int(max(
            cfg.min_bitrate_kbps,
            min(cfg.max_bitrate_kbps, phy * 1000 * cfg.utilization_factor * k_over_n),
        ))
        got = compute_bitrate_kbps(profile, 20, row.mcs, cfg)
        assert got == expected, (
            f"mcs={row.mcs}: expected {expected}, got {got}"
        )


def test_bitrate_higher_mcs_has_higher_bitrate(profile):
    cfg = _cfg()
    a = compute_bitrate_kbps(profile, 20, 1, cfg)
    b = compute_bitrate_kbps(profile, 20, 5, cfg)
    assert b > a
