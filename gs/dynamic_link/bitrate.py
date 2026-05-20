"""Encoder bitrate from PHY rate × utilization × k_over_n.

`k_over_n` is derived from `base_redundancy_ratio` (a fixed
operator-set ratio), NOT the live `(k, n)` of the policy state. This
decouples encoder bitrate from the dynamic `n`-escalation loop —
escalation reserves additional airtime for parity out of the
utilization headroom, not out of the encoder's allocation.

See `docs/superpowers/specs/2026-05-11-drone-config-handshake-and-dynamic-fec-design.md` §"Dynamic FEC algorithm".
"""
from __future__ import annotations

from dataclasses import dataclass

from .profile import RadioProfile


@dataclass(frozen=True)
class BitrateConfig:
    utilization_factor: float = 0.8
    base_redundancy_ratio: float = 0.5   # k/n = 1/(1+ratio) = 0.667
    min_bitrate_kbps: int = 1000
    max_bitrate_kbps: int = 24000

    def __post_init__(self) -> None:
        if not (0.0 < self.utilization_factor <= 1.0):
            raise ValueError(
                f"utilization_factor must be in (0, 1]; "
                f"got {self.utilization_factor}"
            )
        if self.base_redundancy_ratio < 0.0:
            raise ValueError(
                f"base_redundancy_ratio must be >= 0; "
                f"got {self.base_redundancy_ratio}"
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


def compute_bitrate_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    cfg: BitrateConfig,
) -> int:
    """Compute encoder bitrate target in kb/s for `(bandwidth, mcs)`."""
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    k_over_n = 1.0 / (1.0 + cfg.base_redundancy_ratio)
    raw_kbps = phy_Mbps * 1000.0 * cfg.utilization_factor * k_over_n
    return int(max(cfg.min_bitrate_kbps,
                   min(cfg.max_bitrate_kbps, raw_kbps)))
