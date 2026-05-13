"""Radio-profile YAML loader and runtime `rssi_mcs_map` builder (§6.1).

Bitrate is computed at runtime via
`dynamic_link.bitrate.compute_bitrate_kbps` keyed off `policy.bitrate`,
and `(k, n)` are computed at runtime by `dynamic_link.dynamic_fec`
from the drone-reported `(mtu_bytes, fps)` and the live MCS-driven
bitrate. See `docs/knob-cadence-bench.md` for why FEC reconfigs are
debounced via the EmitGate.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


class ProfileError(ValueError):
    """Raised on malformed or semantically invalid radio profile."""


@dataclass(frozen=True)
class MCSRow:
    """One row of the runtime SNR → MCS map.

    `(k, n)` are no longer per-row — they're computed at runtime by
    `dynamic_fec.compute_k` / `compute_n`. See P4b spec.
    """
    mcs: int
    snr_floor_dB: float    # snr_floor_dB[mcs] + snr_margin_db


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
    snr_floor_dB: dict[int, dict[int, float]]    # bw -> mcs -> dB
    data_rate_Mbps_LGI: dict[int, dict[int, float]]

    def snr_mcs_map(
        self,
        bandwidth: int,
        snr_margin_db: float,
    ) -> list[MCSRow]:
        """Build the runtime row table for SNR-driven MCS selection
        (§4.1). High MCS first."""
        if bandwidth not in self.bandwidth_supported:
            raise ProfileError(
                f"bandwidth {bandwidth} not in supported {self.bandwidth_supported}"
            )
        snr = self.snr_floor_dB[bandwidth]
        return [
            MCSRow(mcs=mcs, snr_floor_dB=snr[mcs] + snr_margin_db)
            for mcs in range(self.mcs_max, self.mcs_min - 1, -1)
        ]


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

    # fec_table was removed in P4b. Reject legacy YAMLs explicitly so
    # operators know to delete the section rather than silently ignoring it.
    if "fec_table" in data:
        raise ProfileError(
            f"{source}: fec_table is no longer supported — (k, n) are "
            f"computed at runtime by dynamic_fec from the drone-reported "
            f"(mtu, fps). Remove the fec_table section."
        )

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

    if "sensitivity_dBm" in data:
        raise ProfileError(
            f"{source}: sensitivity_dBm is no longer used — RSSI floors "
            f"never fed the MCS selector (snr_floor_dB drives selection). "
            f"Remove the sensitivity_dBm section."
        )

    snr_floor = {int(bw): {int(m): float(s) for m, s in sub.items()}
                 for bw, sub in req("snr_floor_dB").items()}
    data_rate = {int(bw): {int(m): float(s) for m, s in sub.items()}
                 for bw, sub in req("data_rate_Mbps_LGI").items()}

    for bw in bandwidth_supported:
        if bw not in snr_floor:
            raise ProfileError(f"{source}: snr_floor_dB missing bw={bw}")
        if bw not in data_rate:
            raise ProfileError(f"{source}: data_rate_Mbps_LGI missing bw={bw}")
        for mcs in range(mcs_min, mcs_max + 1):
            if mcs not in snr_floor[bw]:
                raise ProfileError(
                    f"{source}: snr_floor_dB[{bw}] missing mcs={mcs}"
                )
            if mcs not in data_rate[bw]:
                raise ProfileError(
                    f"{source}: data_rate_Mbps_LGI[{bw}] missing mcs={mcs}"
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
        snr_floor_dB=snr_floor,
        data_rate_Mbps_LGI=data_rate,
    )
