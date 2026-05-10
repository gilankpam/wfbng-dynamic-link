"""Bitrate is computed each tick via `compute_bitrate_kbps`.

Verifies the calculator directly: output matches `phy *
utilization * (k/n)`, clamped, for every row in the packaged
profile.
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
    """High-band rows: the (k/n) factor shows up in the computed
    bitrate. Uses MCS 5 (k=8, n=10) on the 20 MHz path."""
    prof = _profile()
    cfg = BitrateConfig(utilization_factor=0.8,
                        min_bitrate_kbps=1, max_bitrate_kbps=999_999)
    e5 = prof.fec_for(20, 5)
    bitrate5 = compute_bitrate_kbps(prof, 20, 5, e5.k, e5.n, cfg)
    # 52.0 * 1000 * 0.8 * 8/10 = 33280
    assert bitrate5 == 33280
