"""Tests for the leading loop: MCS hysteresis + TX-power closed loop (§4.1)."""
from __future__ import annotations

from pathlib import Path

import pytest

from dynamic_link.policy import LeadingLoop, LeadingLoopConfig
from dynamic_link.profile import load_profile

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


def _loop(**overrides) -> LeadingLoop:
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    cfg = LeadingLoopConfig(**overrides)
    return LeadingLoop(cfg, prof)


def test_starts_at_safe_default_mcs1():
    ll = _loop()
    assert ll.current_row.mcs == 1


def test_mcs_down_holds_for_rssi_down_hold_ms():
    """RSSI below current row for < hold_ms does not drop the row."""
    ll = _loop(rssi_down_hold_ms=500.0)
    # Force starting at a high MCS so we have room to drop.
    # Current row is MCS1 (floor -85 at margin 8). Drop it to MCS0 by
    # pushing rssi below -85.
    ll.tick(rssi_smooth=-90.0, ts_ms=0)
    # 100ms later, still below — but < 500 ms hold → no drop yet
    row, _, changed = ll.tick(rssi_smooth=-90.0, ts_ms=100.0)
    assert not changed
    # >500 ms sustained → drop
    row, _, changed = ll.tick(rssi_smooth=-90.0, ts_ms=600.0)
    assert changed
    assert row.mcs == 0


def test_mcs_up_holds_for_rssi_up_hold_ms_with_up_guard():
    ll = _loop(rssi_up_hold_ms=2000.0, rssi_up_guard_db=3.0)
    # Start at MCS1. HT20 MCS2 sensitivity = -91, floor at margin 8 = -83.
    # With up_guard 3, rssi must clear -80 dBm sustained for 2 s.
    ll.tick(rssi_smooth=-60.0, ts_ms=0)  # well above MCS2+guard
    _, _, changed = ll.tick(rssi_smooth=-60.0, ts_ms=1500.0)
    assert not changed  # still under 2 s
    _, _, changed = ll.tick(rssi_smooth=-60.0, ts_ms=2100.0)
    assert changed
    assert ll.current_row.mcs == 2


def test_mcs_up_requires_guard_band_not_just_floor():
    ll = _loop(rssi_up_hold_ms=2000.0, rssi_up_guard_db=3.0)
    # MCS2 floor is -83; up_guard 3 → need rssi >= -80.
    # At rssi=-82 (above floor, below guard), no upgrade candidate.
    ll.tick(rssi_smooth=-82.0, ts_ms=0)
    _, _, changed = ll.tick(rssi_smooth=-82.0, ts_ms=3000.0)
    assert not changed
    assert ll.current_row.mcs == 1


def test_tx_power_deadband_ignores_small_jitter():
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=0.0, tx_power_cooldown_ms=0.0,
    )
    initial_power = ll.state.tx_power_dBm
    # Jitter ±2 dB around target — never breaks the 3 dB deadband.
    for i, r in enumerate([-58, -61, -59, -62, -58]):
        ll.tick(rssi_smooth=float(r), ts_ms=1000.0 + i * 100.0)
    assert ll.state.tx_power_dBm == initial_power


def test_tx_power_cooldown_limits_step_rate():
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=0.0,
        tx_power_cooldown_ms=1000.0,
        tx_power_step_max_db=3.0,
        tx_power_min_dBm=0.0, tx_power_max_dBm=30.0,
    )
    # Start from initial tx power, force rssi way below target.
    # Seed state_tx so we have room to increase.
    ll.state.tx_power_dBm = 10.0
    # Must skip first row-hysteresis tick which may mark a candidate.
    ll.tick(rssi_smooth=-80.0, ts_ms=1000.0)
    p1 = ll.state.tx_power_dBm
    # 500 ms later — inside the 1 s cooldown. No change.
    ll.tick(rssi_smooth=-80.0, ts_ms=1500.0)
    assert ll.state.tx_power_dBm == p1
    # 1100 ms later — outside cooldown. Should step.
    ll.tick(rssi_smooth=-80.0, ts_ms=2100.0)
    assert ll.state.tx_power_dBm > p1


def test_tx_power_step_clamped_to_step_max():
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=0.0, tx_power_cooldown_ms=0.0,
        tx_power_step_max_db=3.0,
        tx_power_min_dBm=0.0, tx_power_max_dBm=30.0,
    )
    ll.state.tx_power_dBm = 10.0
    # 20 dB below target → naive step is +20, but clamp is ±3.
    ll.tick(rssi_smooth=-80.0, ts_ms=1000.0)
    step = ll.state.tx_power_dBm - 10.0
    assert step <= 3.0 + 1e-9


def test_tx_power_freezes_after_mcs_change():
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=2000.0, tx_power_cooldown_ms=0.0,
        rssi_down_hold_ms=0.0,
        tx_power_min_dBm=0.0, tx_power_max_dBm=30.0,
    )
    ll.state.tx_power_dBm = 10.0
    # Force an MCS drop — rssi well below current row floor.
    _, _, changed = ll.tick(rssi_smooth=-95.0, ts_ms=0.0)
    assert changed  # MCS moved
    # Next tick 500 ms later — MCS-freeze blocks power change.
    pre = ll.state.tx_power_dBm
    ll.tick(rssi_smooth=-95.0, ts_ms=500.0)
    assert ll.state.tx_power_dBm == pre


def test_forced_mcs_drop_from_trailing_loop():
    ll = _loop()
    ll.tick(rssi_smooth=-60.0, ts_ms=0.0)  # steady
    start_mcs = ll.current_row.mcs
    _, _, changed = ll.tick(
        rssi_smooth=-60.0, ts_ms=100.0, forced_mcs_drop=True
    )
    assert changed
    assert ll.current_row.mcs < start_mcs


def test_cfg_mcs_max_caps_row_table():
    """leading_loop.mcs_max from gs.yaml must clamp the runtime row
    table — even when the radio profile permits higher MCS."""
    ll = _loop(mcs_max=5, rssi_up_hold_ms=0.0, rssi_up_guard_db=0.0)
    assert max(r.mcs for r in ll.rows) == 5
    # Drive RSSI well above MCS7 floor; the loop must not climb past 5.
    for ts in range(0, 5000, 100):
        ll.tick(rssi_smooth=-30.0, ts_ms=float(ts))
    assert ll.current_row.mcs == 5


def test_cfg_mcs_max_too_low_raises():
    with pytest.raises(ValueError, match="mcs_max"):
        _loop(mcs_max=-1)
