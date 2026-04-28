"""Bitrate selection in the GS controller.

Bitrate is now a per-MCS lookup against the radio profile's
`fec_table`. The Policy emits the row's `bitrate_Mbps` directly —
no FEC-aware scaling, no efficiency math, no compute. See
`docs/knob-cadence-bench.md` for the rationale.
"""
from __future__ import annotations

from pathlib import Path

from dynamic_link.profile import load_profile

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


def _profile():
    return load_profile("m8812eu2", [PACKAGED_DIR])


def test_bitrate_per_row_matches_profile():
    """For each MCS, the runtime row's `bitrate_Mbps` is exactly the
    operator-validated value from `fec_table`."""
    prof = _profile()
    rows = prof.snr_mcs_map(bandwidth=20, snr_margin_db=0.0)
    for row in rows:
        entry = prof.fec_for(20, row.mcs)
        assert row.bitrate_Mbps == entry.bitrate_Mbps
        assert row.k == entry.k
        assert row.n == entry.n


def test_survival_band_bitrate_ladder_monotone():
    """MCS 0 → MCS 2 share `(k, n)` but bitrate climbs."""
    prof = _profile()
    b0 = prof.fec_for(20, 0).bitrate_Mbps
    b1 = prof.fec_for(20, 1).bitrate_Mbps
    b2 = prof.fec_for(20, 2).bitrate_Mbps
    assert b0 < b1 < b2


def test_high_band_redundancy_decreases_with_mcs():
    """Cleaner link → less FEC overhead. n/k ratio drops as MCS climbs."""
    prof = _profile()
    ratios = []
    for mcs in (3, 4, 5, 6, 7):
        e = prof.fec_for(20, mcs)
        ratios.append(e.n / e.k)
    # Strictly non-increasing.
    for prev, cur in zip(ratios, ratios[1:]):
        assert cur <= prev, f"redundancy ratio went up: {ratios}"
