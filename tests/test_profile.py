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
    assert prof.mcs_max == 7
    assert 20 in prof.bandwidth_supported
    assert 40 in prof.bandwidth_supported


def test_rssi_mcs_map_mcs7_floor_matches_spec():
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    rows = prof.rssi_mcs_map(bandwidth=20, rssi_margin_db=8.0)
    # Highest row first.
    assert rows[0].mcs == 7
    assert rows[0].rssi_floor_dBm == -77 + 8  # -69 dBm from §4.1 table
    assert rows[0].preferred_k == 8
    # MCS0 row bitrate = 6.5 Mbps * 0.40 = 2.6 Mbps
    mcs0 = rows[-1]
    assert mcs0.mcs == 0
    assert abs(mcs0.bitrate_Mbps - 2.6) < 1e-6
    assert mcs0.preferred_k == 2


def test_rssi_mcs_map_rejects_unsupported_bandwidth():
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    with pytest.raises(ProfileError):
        prof.rssi_mcs_map(bandwidth=80, rssi_margin_db=8.0)


def _write_profile(tmp_path: Path, data: dict) -> Path:
    p = tmp_path / "test.yaml"
    p.write_text(yaml.safe_dump(data))
    return p


def _base_profile() -> dict:
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
        "preferred_k": {7: 8, 6: 8, 5: 6, 4: 6, 3: 4, 2: 4, 1: 2, 0: 2},
        "encoder_bitrate_frac": 0.40,
    }


def test_rejects_mcs_max_above_7(tmp_path):
    d = _base_profile()
    d["mcs_max"] = 8
    d["sensitivity_dBm"][20][8] = -70
    d["data_rate_Mbps_LGI"][20][8] = 80.0
    d["preferred_k"][8] = 8
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="mcs_max"):
        load_profile_file(p)


def test_rejects_missing_bandwidth_entry(tmp_path):
    d = _base_profile()
    d["bandwidth_supported"] = [20, 40]
    # Missing 40 in sensitivity / data_rate.
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="bw=40"):
        load_profile_file(p)


def test_rejects_non_monotone_preferred_k(tmp_path):
    d = _base_profile()
    d["preferred_k"][3] = 8  # jumps back up at mcs=3
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="monotone"):
        load_profile_file(p)


def test_rejects_preferred_k_out_of_set(tmp_path):
    d = _base_profile()
    d["preferred_k"][7] = 3  # not in {2,4,6,8}
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match=r"\{2,4,6,8\}"):
        load_profile_file(p)


def test_rejects_missing_preferred_k_row(tmp_path):
    d = _base_profile()
    del d["preferred_k"][4]
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="preferred_k missing"):
        load_profile_file(p)


def test_rejects_encoder_bitrate_frac_zero(tmp_path):
    d = _base_profile()
    d["encoder_bitrate_frac"] = 0.0
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="encoder_bitrate_frac"):
        load_profile_file(p)


def test_snr_mcs_map_floor_includes_margin():
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    rows = prof.snr_mcs_map(bandwidth=20, snr_margin_db=3.0)
    # Highest row first (MCS7 floor 25 + margin 3 = 28 dB)
    assert rows[0].mcs == 7
    assert rows[0].snr_floor_dB == 25 + 3
    # MCS0 floor 5 + margin 3 = 8 dB
    assert rows[-1].mcs == 0
    assert rows[-1].snr_floor_dB == 5 + 3


def test_rejects_missing_snr_floor_bw_entry(tmp_path):
    d = _base_profile()
    d["bandwidth_supported"] = [20, 40]
    d["sensitivity_dBm"][40] = {i: -90 + i for i in range(8)}
    d["data_rate_Mbps_LGI"][40] = {i: 6.5 * (i + 1) for i in range(8)}
    # snr_floor_dB still missing 40 → should fail with snr_floor_dB error
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="snr_floor_dB missing bw=40"):
        load_profile_file(p)


def test_rejects_missing_snr_floor_mcs_entry(tmp_path):
    d = _base_profile()
    del d["snr_floor_dB"][20][3]
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="snr_floor_dB.*mcs=3"):
        load_profile_file(p)


def test_search_dir_ordering(tmp_path):
    override_dir = tmp_path / "override"
    override_dir.mkdir()
    custom = _base_profile()
    custom["name"] = "override_wins"
    (override_dir / "m8812eu2.yaml").write_text(yaml.safe_dump(custom))

    prof = load_profile("m8812eu2", [override_dir, PACKAGED_DIR])
    assert prof.name == "override_wins"
