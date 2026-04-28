"""Radio-profile YAML loader and runtime `rssi_mcs_map` builder (§6.1).

Each profile carries a per-(bandwidth, MCS) `fec_table` — the
`(k, n, bitrate_Mbps)` triple to use at that row. The table is
operator-validated per airframe; the controller never moves
`(k, n)` reactively. See `docs/knob-cadence-bench.md` for why
FEC reconfigs are pinned per-MCS instead of escalated on loss.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


class ProfileError(ValueError):
    """Raised on malformed or semantically invalid radio profile."""


@dataclass(frozen=True)
class FECEntry:
    k: int
    n: int
    bitrate_Mbps: float


@dataclass(frozen=True)
class MCSRow:
    """One row of the runtime RSSI/SNR → (MCS, k, n, bitrate) map.

    Both `rssi_floor_dBm` and `snr_floor_dB` are populated by both
    `rssi_mcs_map` and `snr_mcs_map`. The leading loop's MCS hysteresis
    runs on `snr_floor_dB`; `rssi_floor_dBm` is retained for diagnostic
    visibility and any future RSSI-based safety check. `(k, n)` and
    `bitrate_Mbps` come straight from the profile's `fec_table` row —
    they are not computed at runtime.
    """
    mcs: int
    rssi_floor_dBm: float  # sensitivity_dBm[mcs] + rssi_margin_db
    snr_floor_dB: float    # snr_floor_dB[mcs] + snr_margin_db
    bitrate_Mbps: float    # fec_table[bw][mcs].bitrate_Mbps
    k: int                 # fec_table[bw][mcs].k
    n: int                 # fec_table[bw][mcs].n


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
    snr_floor_dB: dict[int, dict[int, float]]    # bw -> mcs -> dB
    data_rate_Mbps_LGI: dict[int, dict[int, float]]
    fec_table: dict[int, dict[int, FECEntry]]    # bw -> mcs -> FECEntry

    def fec_for(self, bandwidth: int, mcs: int) -> FECEntry:
        """Look up the operator-validated `(k, n, bitrate_Mbps)` for
        a (bandwidth, mcs) pair. Raises `ProfileError` if missing."""
        if bandwidth not in self.fec_table:
            raise ProfileError(
                f"fec_table missing bandwidth {bandwidth}"
            )
        row = self.fec_table[bandwidth]
        if mcs not in row:
            raise ProfileError(
                f"fec_table[{bandwidth}] missing mcs {mcs}"
            )
        return row[mcs]

    def _build_rows(
        self,
        bandwidth: int,
        rssi_margin_db: float,
        snr_margin_db: float,
    ) -> list[MCSRow]:
        if bandwidth not in self.bandwidth_supported:
            raise ProfileError(
                f"bandwidth {bandwidth} not in supported {self.bandwidth_supported}"
            )
        sens = self.sensitivity_dBm[bandwidth]
        snr = self.snr_floor_dB[bandwidth]
        fec = self.fec_table[bandwidth]
        rows = []
        for mcs in range(self.mcs_max, self.mcs_min - 1, -1):
            entry = fec[mcs]
            rows.append(MCSRow(
                mcs=mcs,
                rssi_floor_dBm=sens[mcs] + rssi_margin_db,
                snr_floor_dB=snr[mcs] + snr_margin_db,
                bitrate_Mbps=entry.bitrate_Mbps,
                k=entry.k,
                n=entry.n,
            ))
        return rows

    def rssi_mcs_map(self, bandwidth: int, rssi_margin_db: float) -> list[MCSRow]:
        """Legacy entry point — leaves snr_floor_dB at 0 margin."""
        return self._build_rows(bandwidth, rssi_margin_db, 0.0)

    def snr_mcs_map(
        self,
        bandwidth: int,
        snr_margin_db: float,
        rssi_margin_db: float = 0.0,
    ) -> list[MCSRow]:
        """Build the runtime row table for SNR-driven MCS selection
        (§4.1). High MCS first."""
        return self._build_rows(bandwidth, rssi_margin_db, snr_margin_db)


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
    snr_floor = {int(bw): {int(m): float(s) for m, s in sub.items()}
                 for bw, sub in req("snr_floor_dB").items()}
    data_rate = {int(bw): {int(m): float(s) for m, s in sub.items()}
                 for bw, sub in req("data_rate_Mbps_LGI").items()}

    fec_raw = req("fec_table")
    if not isinstance(fec_raw, dict):
        raise ProfileError(f"{source}: fec_table must be a mapping")
    fec_table: dict[int, dict[int, FECEntry]] = {}
    for bw_key, sub in fec_raw.items():
        bw = int(bw_key)
        if not isinstance(sub, dict):
            raise ProfileError(
                f"{source}: fec_table[{bw}] must be a mapping"
            )
        per_bw: dict[int, FECEntry] = {}
        for mcs_key, entry in sub.items():
            mcs_i = int(mcs_key)
            if not isinstance(entry, dict):
                raise ProfileError(
                    f"{source}: fec_table[{bw}][{mcs_i}] must be a mapping"
                )
            for f in ("k", "n", "bitrate_Mbps"):
                if f not in entry:
                    raise ProfileError(
                        f"{source}: fec_table[{bw}][{mcs_i}] missing {f!r}"
                    )
            k = int(entry["k"])
            n = int(entry["n"])
            br = float(entry["bitrate_Mbps"])
            if k < 1 or k > 12:
                raise ProfileError(
                    f"{source}: fec_table[{bw}][{mcs_i}] k={k} out of range [1, 12]"
                )
            if n <= k:
                raise ProfileError(
                    f"{source}: fec_table[{bw}][{mcs_i}] n={n} must be > k={k}"
                )
            if n > 32:
                raise ProfileError(
                    f"{source}: fec_table[{bw}][{mcs_i}] n={n} exceeds wfb-ng cap (32)"
                )
            if br <= 0.0:
                raise ProfileError(
                    f"{source}: fec_table[{bw}][{mcs_i}] bitrate_Mbps={br} must be > 0"
                )
            per_bw[mcs_i] = FECEntry(k=k, n=n, bitrate_Mbps=br)
        fec_table[bw] = per_bw

    for bw in bandwidth_supported:
        if bw not in sensitivity:
            raise ProfileError(f"{source}: sensitivity_dBm missing bw={bw}")
        if bw not in snr_floor:
            raise ProfileError(f"{source}: snr_floor_dB missing bw={bw}")
        if bw not in data_rate:
            raise ProfileError(f"{source}: data_rate_Mbps_LGI missing bw={bw}")
        if bw not in fec_table:
            raise ProfileError(f"{source}: fec_table missing bw={bw}")
        for mcs in range(mcs_min, mcs_max + 1):
            if mcs not in sensitivity[bw]:
                raise ProfileError(
                    f"{source}: sensitivity_dBm[{bw}] missing mcs={mcs}"
                )
            if mcs not in snr_floor[bw]:
                raise ProfileError(
                    f"{source}: snr_floor_dB[{bw}] missing mcs={mcs}"
                )
            if mcs not in data_rate[bw]:
                raise ProfileError(
                    f"{source}: data_rate_Mbps_LGI[{bw}] missing mcs={mcs}"
                )
            if mcs not in fec_table[bw]:
                raise ProfileError(
                    f"{source}: fec_table[{bw}] missing mcs={mcs}"
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
        snr_floor_dB=snr_floor,
        data_rate_Mbps_LGI=data_rate,
        fec_table=fec_table,
    )
