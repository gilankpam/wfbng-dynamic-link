"""Radio-profile YAML loader and runtime `rssi_mcs_map` builder (§6.1)."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


class ProfileError(ValueError):
    """Raised on malformed or semantically invalid radio profile."""


_ALLOWED_K = frozenset({2, 4, 6, 8})


@dataclass(frozen=True)
class MCSRow:
    """One row of the runtime RSSI → (MCS, bitrate, k) map."""
    mcs: int
    rssi_floor_dBm: float  # sensitivity_dBm[mcs] + rssi_margin_db
    bitrate_Mbps: float    # data_rate * encoder_bitrate_frac
    preferred_k: int


@dataclass(frozen=True)
class RadioProfile:
    """Parsed, validated radio profile — see design doc §6.1."""
    name: str
    chipset: str
    mcs_min: int
    mcs_max: int
    bandwidth_supported: tuple[int, ...]
    bandwidth_default: int
    tx_power_min_dBm: int
    tx_power_max_dBm: int
    sensitivity_dBm: dict[int, dict[int, int]]   # bw -> mcs -> dBm
    data_rate_Mbps_LGI: dict[int, dict[int, float]]
    preferred_k: dict[int, int]
    encoder_bitrate_frac: float

    def rssi_mcs_map(self, bandwidth: int, rssi_margin_db: float) -> list[MCSRow]:
        """Build the runtime row table (§4.1), high MCS first."""
        if bandwidth not in self.bandwidth_supported:
            raise ProfileError(
                f"bandwidth {bandwidth} not in supported {self.bandwidth_supported}"
            )
        sens = self.sensitivity_dBm[bandwidth]
        rates = self.data_rate_Mbps_LGI[bandwidth]
        rows = []
        for mcs in range(self.mcs_max, self.mcs_min - 1, -1):
            rows.append(MCSRow(
                mcs=mcs,
                rssi_floor_dBm=sens[mcs] + rssi_margin_db,
                bitrate_Mbps=rates[mcs] * self.encoder_bitrate_frac,
                preferred_k=self.preferred_k[mcs],
            ))
        return rows


def load_profile(name: str, search_dirs: list[Path]) -> RadioProfile:
    """Resolve `<dir>/<name>.yaml` in order; load and validate the first hit."""
    for d in search_dirs:
        candidate = Path(d) / f"{name}.yaml"
        if candidate.is_file():
            return load_profile_file(candidate)
    searched = ", ".join(str(p) for p in search_dirs)
    raise ProfileError(f"radio profile {name!r} not found in: {searched}")


def load_profile_file(path: Path) -> RadioProfile:
    with open(path, "r") as fd:
        data = yaml.safe_load(fd)
    if not isinstance(data, dict):
        raise ProfileError(f"{path}: top-level must be a mapping")
    return _validate(data, source=str(path))


def _validate(data: dict, source: str) -> RadioProfile:
    def req(key: str):
        if key not in data:
            raise ProfileError(f"{source}: missing required key {key!r}")
        return data[key]

    mcs_min = int(req("mcs_min"))
    mcs_max = int(req("mcs_max"))
    if mcs_max > 7:
        raise ProfileError(
            f"{source}: mcs_max={mcs_max} > 7 violates design policy (1T1R SISO)"
        )
    if mcs_min < 0 or mcs_min > mcs_max:
        raise ProfileError(f"{source}: invalid mcs_min={mcs_min} mcs_max={mcs_max}")

    bandwidth_supported = tuple(int(b) for b in req("bandwidth_supported"))
    bandwidth_default = int(req("bandwidth_default"))
    if bandwidth_default not in bandwidth_supported:
        raise ProfileError(
            f"{source}: bandwidth_default={bandwidth_default} not in supported list"
        )

    tx_min = int(req("tx_power_min_dBm"))
    tx_max = int(req("tx_power_max_dBm"))
    if tx_min > tx_max:
        raise ProfileError(f"{source}: tx_power_min_dBm > tx_power_max_dBm")

    sensitivity = {int(bw): {int(m): int(s) for m, s in sub.items()}
                   for bw, sub in req("sensitivity_dBm").items()}
    data_rate = {int(bw): {int(m): float(s) for m, s in sub.items()}
                 for bw, sub in req("data_rate_Mbps_LGI").items()}

    for bw in bandwidth_supported:
        if bw not in sensitivity:
            raise ProfileError(f"{source}: sensitivity_dBm missing bw={bw}")
        if bw not in data_rate:
            raise ProfileError(f"{source}: data_rate_Mbps_LGI missing bw={bw}")
        for mcs in range(mcs_min, mcs_max + 1):
            if mcs not in sensitivity[bw]:
                raise ProfileError(
                    f"{source}: sensitivity_dBm[{bw}] missing mcs={mcs}"
                )
            if mcs not in data_rate[bw]:
                raise ProfileError(
                    f"{source}: data_rate_Mbps_LGI[{bw}] missing mcs={mcs}"
                )

    preferred_k_raw = req("preferred_k")
    preferred_k = {int(m): int(k) for m, k in preferred_k_raw.items()}
    for mcs in range(mcs_min, mcs_max + 1):
        if mcs not in preferred_k:
            raise ProfileError(f"{source}: preferred_k missing mcs={mcs}")
        if preferred_k[mcs] not in _ALLOWED_K:
            raise ProfileError(
                f"{source}: preferred_k[{mcs}]={preferred_k[mcs]} not in {{2,4,6,8}}"
            )
    # Monotone non-increasing from mcs_max down to mcs_min (i.e. k decreases as
    # MCS decreases — high MCS uses bigger k blocks).
    prev = preferred_k[mcs_max]
    for mcs in range(mcs_max - 1, mcs_min - 1, -1):
        cur = preferred_k[mcs]
        if cur > prev:
            raise ProfileError(
                f"{source}: preferred_k not monotone non-increasing from "
                f"mcs_max down (mcs={mcs} k={cur} > prev k={prev})"
            )
        prev = cur

    encoder_bitrate_frac = float(req("encoder_bitrate_frac"))
    if not 0.0 < encoder_bitrate_frac <= 1.0:
        raise ProfileError(
            f"{source}: encoder_bitrate_frac={encoder_bitrate_frac} must be in (0, 1]"
        )

    return RadioProfile(
        name=str(req("name")),
        chipset=str(req("chipset")),
        mcs_min=mcs_min,
        mcs_max=mcs_max,
        bandwidth_supported=bandwidth_supported,
        bandwidth_default=bandwidth_default,
        tx_power_min_dBm=tx_min,
        tx_power_max_dBm=tx_max,
        sensitivity_dBm=sensitivity,
        data_rate_Mbps_LGI=data_rate,
        preferred_k=preferred_k,
        encoder_bitrate_frac=encoder_bitrate_frac,
    )
