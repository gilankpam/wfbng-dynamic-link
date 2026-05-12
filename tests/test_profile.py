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


def load_packaged():
    return load_profile("m8812eu2", [PACKAGED_DIR])


def test_snr_mcs_map_rejects_unsupported_bandwidth():
    prof = load_packaged()
    with pytest.raises(ProfileError):
        prof.snr_mcs_map(bandwidth=80, snr_margin_db=0.0)


def test_profile_loader_rejects_legacy_sensitivity_dBm_key(tmp_path):
    """sensitivity_dBm was removed; loading a YAML that still has it
    should fail loudly so operators clean up."""
    d = _valid_dict()
    d["sensitivity_dBm"] = {20: {i: -90 for i in range(8)}}
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="sensitivity_dBm"):
        load_profile_file(p)


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
        "snr_floor_dB": {20: {i: 5 + 3 * i for i in range(8)}},
        "data_rate_Mbps_LGI": {20: {i: 6.5 * (i + 1) for i in range(8)}},
    }


def test_rejects_mcs_max_above_7(tmp_path):
    d = _valid_dict()
    d["mcs_max"] = 8
    d["snr_floor_dB"][20][8] = 30.0
    d["data_rate_Mbps_LGI"][20][8] = 80.0
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="mcs_max"):
        load_profile_file(p)


def test_rejects_missing_bandwidth_entry(tmp_path):
    d = _valid_dict()
    d["bandwidth_supported"] = [20, 40]
    # Missing 40 in snr_floor_dB / data_rate_Mbps_LGI.
    p = _write_profile(tmp_path, d)
    with pytest.raises(ProfileError, match="bw=40"):
        load_profile_file(p)


def test_profile_loader_rejects_legacy_fec_table_key(tmp_path):
    """A profile YAML that still has the removed fec_table section
    is an operator error — fail loudly rather than silently ignore."""
    p = tmp_path / "bad.yaml"
    p.write_text("""
name: T
chipset: T
mcs_min: 0
mcs_max: 1
bandwidth_supported: [20]
bandwidth_default: 20
tx_power_min_dBm: 0
tx_power_max_dBm: 20
snr_floor_dB:
  20:
    0: 0
    1: 5
data_rate_Mbps_LGI:
  20:
    0: 6.5
    1: 13.0
fec_table:
  20:
    0: {k: 2, n: 5}
""")
    try:
        load_profile_file(p)
    except ProfileError as e:
        assert "fec_table" in str(e)
    else:
        raise AssertionError("expected ProfileError")


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
    d["data_rate_Mbps_LGI"][40] = {i: 6.5 * (i + 1) for i in range(8)}
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


