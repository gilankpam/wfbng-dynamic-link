"""Encoder bitrate from effective PHY rate × utilization × k/n.

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

DEFAULT_PREAMBLE_US: float = 200.0

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


@dataclass(frozen=True)
class BitrateConfig:
    utilization_factor: float = 0.8
    min_bitrate_kbps: int = 1000
    max_bitrate_kbps: int = 24000

    def __post_init__(self) -> None:
        if not (0.0 < self.utilization_factor <= 1.0):
            raise ValueError(
                f"utilization_factor must be in (0, 1]; "
                f"got {self.utilization_factor}"
            )
        if self.min_bitrate_kbps <= 0:
            raise ValueError(
                f"min_bitrate_kbps must be > 0; "
                f"got {self.min_bitrate_kbps}"
            )
        if self.max_bitrate_kbps < self.min_bitrate_kbps:
            raise ValueError(
                f"max_bitrate_kbps ({self.max_bitrate_kbps}) "
                f"< min_bitrate_kbps ({self.min_bitrate_kbps})"
            )


def effective_phy_Mbps(
    phy_Mbps: float, mtu_bytes: int, preamble_us: float
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


def compute_wire_target_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    mtu_bytes: int,
    utilization_factor: float,
) -> float:
    """Maximum sustainable wire bitrate (kbps) at this (MCS, bandwidth, mtu).

    `wire_target = eff_phy × utilization_factor`. This is the anchor
    for the dynamic-FEC dataflow: `k` is sized for it, `n` adds
    redundancy on top of it, and `bitrate = wire_target × k / n`.

    The result depends only on the radio profile, MCS row, MTU, and
    utilization — no FEC inputs. Returned as a float; the caller is
    responsible for any int truncation.
    """
    if not (0.0 < utilization_factor <= 1.0):
        raise ValueError(
            f"utilization_factor must be in (0, 1]; got {utilization_factor}"
        )
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    preamble_us = _resolve_preamble_us(profile)
    eff_phy_Mbps = effective_phy_Mbps(phy_Mbps, mtu_bytes, preamble_us)
    return eff_phy_Mbps * 1000.0 * utilization_factor


def compute_bitrate_kbps(
    wire_target_kbps: float,
    k: int,
    n: int,
    min_bitrate_kbps: int,
    max_bitrate_kbps: int,
) -> int:
    """Encoder bitrate that keeps wire rate at `wire_target_kbps`
    with the live (k, n). Clamped to [min, max].

    Invariant when no clamp fires: `result × n / k ≤ wire_target_kbps`
    (int truncation rounds wire DOWN). When `min_bitrate_kbps` clamp
    fires — only possible when the link is too degraded to sustain
    minimum video — the invariant does not hold; callers should run
    `clamp_n_for_bitrate_floor` first to prevent this.
    """
    if k <= 0:
        raise ValueError(f"k must be > 0; got {k}")
    if n < k:
        raise ValueError(f"n ({n}) must be >= k ({k})")
    if min_bitrate_kbps <= 0:
        raise ValueError(f"min_bitrate_kbps must be > 0; got {min_bitrate_kbps}")
    if max_bitrate_kbps < min_bitrate_kbps:
        raise ValueError(
            f"max_bitrate_kbps ({max_bitrate_kbps}) "
            f"< min_bitrate_kbps ({min_bitrate_kbps})"
        )
    raw_kbps = wire_target_kbps * k / n
    return int(max(min_bitrate_kbps, min(max_bitrate_kbps, raw_kbps)))


