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
