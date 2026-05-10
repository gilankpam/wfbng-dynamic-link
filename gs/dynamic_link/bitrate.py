"""Dynamic encoder bitrate from PHY rate × utilization × (k/n).

Replaces the per-row `bitrate_Mbps` lookup in the radio profile's
`fec_table`. Operator tunes one global `utilization_factor`
instead of a per-MCS bitrate; FEC overhead is folded in via the
actual `(k/n)` ratio of the row the leading loop selected. PHY
data rate comes from the profile's `data_rate_Mbps_LGI` table —
LGI only because we don't do short-GI selection (see design doc).

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
    utilization_factor: float
    min_bitrate_kbps: int
    max_bitrate_kbps: int


def compute_bitrate_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    k: int,
    n: int,
    cfg: BitrateConfig,
) -> int:
    """Encoder kbps = PHY × utilization × (k/n), clamped to [min, max]."""
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    raw_kbps = phy_Mbps * 1000.0 * cfg.utilization_factor * (k / n)
    return int(max(cfg.min_bitrate_kbps,
                   min(cfg.max_bitrate_kbps, raw_kbps)))
