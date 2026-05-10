"""Dynamic encoder bitrate from PHY rate × utilization × (k/n).

Operator tunes one global `utilization_factor`; FEC overhead is
folded in via the actual `(k/n)` ratio of the row the leading
loop selected. PHY data rate comes from the profile's
`data_rate_Mbps_LGI` table — LGI only because we don't do
short-GI selection (see design doc).

Defaults match alink_gs.conf:
  utilization_factor = 0.8   ([dynamic] utilization_factor)
  min_bitrate_kbps   = 1000  ([hardware] min_bitrate)
  max_bitrate_kbps   = 24000 ([hardware] max_bitrate)
"""
from __future__ import annotations

from dataclasses import dataclass

from .profile import RadioProfile


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


def compute_bitrate_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    k: int,
    n: int,
    cfg: BitrateConfig,
) -> int:
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    raw_kbps = phy_Mbps * 1000.0 * cfg.utilization_factor * (k / n)
    return int(max(cfg.min_bitrate_kbps,
                   min(cfg.max_bitrate_kbps, raw_kbps)))
