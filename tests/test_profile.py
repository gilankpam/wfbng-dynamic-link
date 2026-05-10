"""Tests for the radio-profile YAML loader and §6.1 validation."""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from dynamic_link.profile import (
    ProfileError,
    load_profile,
    load_profile_file,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


def test_load_packaged_m8812eu2():
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    assert prof.name == "BL-M8812EU2"
    assert prof.chipset == "RTL8812EU"
    # Operator-tunable; just verify the field is in a sensible range.
    assert 0 <= prof.mcs_min <= prof.mcs_max <= 7
    assert 20 in prof.bandwidth_supported
    assert 40 in prof.bandwidth_supported


def test_rssi_mcs_map_floor_matches_spec():
    prof = load_packaged()
    rows = prof.rssi_mcs_map(bandwidth=20, rssi_margin_db=8.0)
    # Highest mcs row first; the test asserts against whatever
    # mcs_max the operator has currently configured.
    top = rows[0]
    assert top.mcs == prof.mcs_max
    expected_floor = prof.sensitivity_dBm[20][prof.mcs_max] + 8
    assert top.rssi_floor_dBm == expected_floor
    mcs0 = rows[-1]
    assert mcs0.mcs == 0
    assert mcs0.k == 2 and mcs0.n == 5


def test_fec_for_mcs_returns_table_entry():
    prof = load_packaged()
    entry = prof.fec_for(bandwidth=20, mcs=5)
    assert entry.k == 8
    assert entry.n == 10


def test_fec_for_mcs_survival_band_shares_kn():
    """MCS 0/1/2 share `(k, n)` so an MCS bounce in the survival
    band never fires a `CMD_SET_FEC`."""
    prof = load_packaged()
    e0 = prof.fec_for(bandwidth=20, mcs=0)
    e1 = prof.fec_for(bandwidth=20, mcs=1)
    e2 = prof.fec_for(bandwidth=20, mcs=2)
    assert (e0.k, e0.n) == (e1.k, e1.n) == (e2.k, e2.n) == (2, 5)


def load_packaged():
    return load_profile("m8812eu2", [PACKAGED_DIR])


def test_rssi_mcs_map_rejects_unsupported_bandwidth():
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    with pytest.raises(ProfileError):
        prof.rssi_mcs_map(bandwidth=80, rssi_margin_db=8.0)


def _write_profile(tmp_path: Path, data: dict) -> Path:
    p = tmp_path / "test.yaml"
    p.write_text(yaml.safe_dump(data))
    return p


def _valid_dict() -> dict:
    return {
        "name": "test",
        "chipset": "test",
        "mcs_min": 0,
        "mcs_max": 7,
        "bandwidth_supported": [20],
        "bandwidth_default": 20,
        "tx_power_min_dBm": 0,
        "tx_power_max_dBm": 20,
        "sensitivity_dBm": {20: {i: -90 + i for i in range(8)}},
        "snr_floor_dB": {20: {i: 5 + 3 * i for i in range(8)}},
        "data_rate_Mbps_LGI": {20: {i: 6.5 * (i + 1) for i in range(8)}},
        "fec_table": {
            20: {
                0: {"k": 2, "n": 5},
                1: {"k": 2, "n": 5},
                2: {"k": 2, "n": 5},
                3: {"k": 4, "n": 7},
                4: {"k": 6, "n": 9},
                5: {"k": 8, "n": 10},
                6: {"k": 10, "n": 12},
                7: {"k": 12, "n": 14},
            },
        },
    }


def test_rejects_mcs_max_above_7(tmp_path):
    d = _valid_dict()
    d["mcs_max"] = 8
    d["sensitivity_dBm"][20][8] = -70
    d["data_rate_Mbps_LGI"][20][8] = 80.0
    d["fec_table"][20][8] = {"k": 12, "n": 14}
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="mcs_max"):
        load_profile_file(p)


def test_rejects_missing_bandwidth_entry(tmp_path):
    d = _valid_dict()
    d["bandwidth_supported"] = [20, 40]
    # Missing 40 in sensitivity / data_rate.
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="bw=40"):
        load_profile_file(p)


def test_rejects_fec_table_n_le_k(tmp_path):
    d = _valid_dict()
    d["fec_table"][20][5]["n"] = 8  # k=8, n=8 → invalid (n must be > k)
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="must be > k"):
        load_profile_file(p)


def test_rejects_fec_table_k_above_max(tmp_path):
    d = _valid_dict()
    d["fec_table"][20][7] = {"k": 13, "n": 15}
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="out of range"):
        load_profile_file(p)


def test_rejects_fec_table_missing_mcs(tmp_path):
    d = _valid_dict()
    del d["fec_table"][20][3]
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match=r"fec_table\[20\] missing mcs=3"):
        load_profile_file(p)


def test_snr_mcs_map_floor_includes_margin():
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    rows = prof.snr_mcs_map(bandwidth=20, snr_margin_db=3.0)
    # Highest configured mcs first.
    top = rows[0]
    assert top.mcs == prof.mcs_max
    assert top.snr_floor_dB == prof.snr_floor_dB[20][prof.mcs_max] + 3
    # MCS0 floor 5 + margin 3 = 8 dB
    assert rows[-1].mcs == 0
    assert rows[-1].snr_floor_dB == 5 + 3


def test_rejects_missing_snr_floor_bw_entry(tmp_path):
    d = _valid_dict()
    d["bandwidth_supported"] = [20, 40]
    d["sensitivity_dBm"][40] = {i: -90 + i for i in range(8)}
    d["data_rate_Mbps_LGI"][40] = {i: 6.5 * (i + 1) for i in range(8)}
    d["fec_table"][40] = dict(d["fec_table"][20])
    # snr_floor_dB still missing 40 → should fail with snr_floor_dB error
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="snr_floor_dB missing bw=40"):
        load_profile_file(p)


def test_rejects_missing_snr_floor_mcs_entry(tmp_path):
    d = _valid_dict()
    del d["snr_floor_dB"][20][3]
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="snr_floor_dB.*mcs=3"):
        load_profile_file(p)


def test_search_dir_ordering(tmp_path):
    override_dir = tmp_path / "override"
    override_dir.mkdir()
    custom = _valid_dict()
    custom["name"] = "override_wins"
    (override_dir / "m8812eu2.yaml").write_text(yaml.safe_dump(custom))

    prof = load_profile("m8812eu2", [override_dir, PACKAGED_DIR])
    assert prof.name == "override_wins"


def test_rejects_fec_table_bitrate_field(tmp_path):
    """bitrate_Mbps in fec_table is rejected with a clear error —
    bitrate is now computed from policy.bitrate."""
    d = _valid_dict()
    d["fec_table"][20][3] = {"k": 4, "n": 7, "bitrate_Mbps": 7.5}
    p = tmp_path / "p.yaml"
    p.write_text(yaml.safe_dump(d))
    with pytest.raises(
        ProfileError, match=r"bitrate_Mbps.*policy\.bitrate",
    ):
        load_profile_file(p)
