"""FEC-aware bitrate computation in the GS controller.

The Policy emits `bitrate_kbps` scaled so that total on-air bitrate
(`video × n/k`) stays roughly constant under FEC changes. The profile's
`bitrate_Mbps` is the operator-tuned video bitrate at the default FEC
`(safe_k, safe_n)`.
"""
from __future__ import annotations

import pytest

from dynamic_link.policy import _fec_aware_bitrate_kbps
from dynamic_link.predictor import LADDER_DROP, LADDER_STEPS


PROFILE_Mbps = 26.0   # MCS7 HT20 m8812eu2: 65 × 0.40
SAFE_K, SAFE_N = 8, 12


def test_bitrate_unscaled_at_safe_defaults():
    """At default FEC, bitrate equals the profile value — operator's
    tuning is preserved."""
    got = _fec_aware_bitrate_kbps(PROFILE_Mbps, SAFE_K, SAFE_N, SAFE_K, SAFE_N)
    assert got == 26000


def test_bitrate_scales_down_at_aggressive_fec():
    """At (k=2, n=8) with safe (8, 12): efficiency 0.25 / 0.667 = 0.375.
    Video bitrate ⇒ 26 × 0.375 = 9.75 Mbps. On-air ⇒ 9.75 × 4 = 39 Mbps,
    same as 26 Mbps × 12/8 at default — invariant under FEC."""
    got = _fec_aware_bitrate_kbps(PROFILE_Mbps, 2, 8, SAFE_K, SAFE_N)
    assert got == int(26000 * 0.375)
    # On-air invariance check.
    on_air_default = 26000 * (SAFE_N / SAFE_K)
    on_air_now     = got    * (8 / 2)
    assert abs(on_air_now - on_air_default) <= 1.0  # within rounding


def test_bitrate_capped_at_profile_for_efficient_fec():
    """A hypothetical FEC more efficient than safe defaults must NOT
    auto-boost video bitrate above the operator-validated profile.
    Defensive cap — current ladder doesn't produce this, but guards
    against future LADDER_STEPS edits."""
    # (k=8, n=10): efficiency 0.8, default 0.667 → ratio 1.2 → cap to 1.0.
    got = _fec_aware_bitrate_kbps(PROFILE_Mbps, 8, 10, SAFE_K, SAFE_N)
    assert got == 26000


@pytest.mark.parametrize("k,n", [
    *(kn for band in LADDER_STEPS.values() for kn in band),
    *LADDER_DROP.values(),
])
def test_on_air_bitrate_bounded_for_every_ladder_pair(k, n):
    """For every (k, n) the trailing loop can produce, the resulting
    on-air bitrate must not exceed the on-air bitrate at safe defaults."""
    got = _fec_aware_bitrate_kbps(PROFILE_Mbps, k, n, SAFE_K, SAFE_N)
    on_air_default = 26000 * (SAFE_N / SAFE_K)
    on_air_now     = got    * (n / k) if k > 0 else float("inf")
    # Allow 1 kbps slack for int truncation in the helper.
    assert on_air_now <= on_air_default + 1.0, (
        f"({k},{n}) → video={got} → on-air={on_air_now:.0f} > "
        f"default {on_air_default:.0f}"
    )
