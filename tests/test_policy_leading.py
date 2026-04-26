"""Tests for the leading loop: SNR-driven MCS hysteresis + RSSI-driven
TX-power closed loop (§4.1)."""
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


# Convenience: drive only the MCS hysteresis (snr) or only the TX power
# loop (rssi). The other signal is parked at a "fine" value so it
# doesn't interfere.
_NEUTRAL_RSSI = -60.0   # at rssi_target by default → TX loop deadband
_NEUTRAL_SNR = 30.0     # well above any per-MCS SNR floor in the profile


def _tick(ll, *, snr=_NEUTRAL_SNR, rssi=_NEUTRAL_RSSI, ts_ms=0.0,
          forced_mcs_drop=False, force_tx_power_to=None):
    return ll.tick(
        rssi_smooth=rssi, snr_smooth=snr, ts_ms=ts_ms,
        forced_mcs_drop=forced_mcs_drop,
        force_tx_power_to=force_tx_power_to,
    )


def test_starts_at_safe_default_mcs1():
    ll = _loop()
    assert ll.current_row.mcs == 1


def test_mcs_down_holds_for_snr_down_hold_ms():
    """SNR below current row floor for < hold_ms does not drop the row."""
    ll = _loop(snr_down_hold_ms=500.0, snr_margin_db=3.0)
    # MCS1 SNR floor = profile snr_floor[20][1] (8) + margin 3 = 11.
    # SNR=5 is well below — should drop, but only after hold expires.
    _tick(ll, snr=5.0, ts_ms=0)
    _, _, changed = _tick(ll, snr=5.0, ts_ms=100.0)
    assert not changed  # < 500 ms hold
    _, _, changed = _tick(ll, snr=5.0, ts_ms=600.0)
    assert changed
    assert ll.current_row.mcs == 0


def test_mcs_up_holds_for_snr_up_hold_ms_with_up_guard():
    ll = _loop(snr_up_hold_ms=2000.0, snr_up_guard_db=2.0,
               snr_margin_db=3.0, forced_drop_inhibit_ms=0.0)
    # Start at MCS1 (floor 11). MCS2 floor = 11+3=14; need snr >= 14+2 = 16.
    _tick(ll, snr=20.0, ts_ms=0)  # well above
    _, _, changed = _tick(ll, snr=20.0, ts_ms=1500.0)
    assert not changed  # still under 2 s
    _, _, changed = _tick(ll, snr=20.0, ts_ms=2100.0)
    assert changed
    assert ll.current_row.mcs == 2


def test_mcs_up_requires_guard_band_not_just_floor():
    ll = _loop(snr_up_hold_ms=2000.0, snr_up_guard_db=2.0,
               snr_margin_db=3.0, forced_drop_inhibit_ms=0.0)
    # MCS2 floor 14; up_guard 2 → need snr >= 16.
    # snr=15 (above floor, below guard) should not upgrade.
    _tick(ll, snr=15.0, ts_ms=0)
    _, _, changed = _tick(ll, snr=15.0, ts_ms=3000.0)
    assert not changed
    assert ll.current_row.mcs == 1


def test_post_drop_inhibit_blocks_climb_for_inhibit_ms():
    """After a forced drop, no MCS up-climb until inhibit window expires."""
    ll = _loop(snr_up_hold_ms=0.0, snr_up_guard_db=0.0,
               forced_drop_inhibit_ms=5000.0, snr_margin_db=3.0)
    _tick(ll, snr=30.0, ts_ms=0)
    # Force a drop at t=100. Now in MCS0.
    _, _, changed = _tick(ll, snr=30.0, ts_ms=100.0, forced_mcs_drop=True)
    assert changed
    dropped_mcs = ll.current_row.mcs
    # Even with great SNR for hours, no climb until inhibit expires.
    for ts in range(200, 5000, 100):
        _tick(ll, snr=40.0, ts_ms=float(ts))
    assert ll.current_row.mcs == dropped_mcs


def test_post_drop_inhibit_expires_then_climb_allowed():
    ll = _loop(snr_up_hold_ms=0.0, snr_up_guard_db=0.0,
               forced_drop_inhibit_ms=1000.0, snr_margin_db=3.0)
    _tick(ll, snr=30.0, ts_ms=0)
    _, _, _ = _tick(ll, snr=30.0, ts_ms=100.0, forced_mcs_drop=True)
    pre_climb_mcs = ll.current_row.mcs
    # Past the 1 s inhibit, SNR clears the up trigger → climb fires.
    _, _, _ = _tick(ll, snr=40.0, ts_ms=1200.0)
    _, _, changed = _tick(ll, snr=40.0, ts_ms=1300.0)
    assert changed
    assert ll.current_row.mcs > pre_climb_mcs


def test_tx_power_deadband_ignores_small_jitter():
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=0.0, tx_power_cooldown_ms=0.0,
    )
    initial_power = ll.state.tx_power_dBm
    # Jitter ±2 dB around target — never breaks the 3 dB deadband.
    for i, r in enumerate([-58, -61, -59, -62, -58]):
        _tick(ll, rssi=float(r), ts_ms=1000.0 + i * 100.0)
    assert ll.state.tx_power_dBm == initial_power


def test_tx_power_cooldown_limits_step_rate():
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=0.0,
        tx_power_cooldown_ms=1000.0,
        tx_power_step_max_db=3.0,
        tx_power_min_dBm=0.0, tx_power_max_dBm=30.0,
    )
    ll.state.tx_power_dBm = 10.0
    _tick(ll, rssi=-80.0, ts_ms=1000.0)
    p1 = ll.state.tx_power_dBm
    _tick(ll, rssi=-80.0, ts_ms=1500.0)
    assert ll.state.tx_power_dBm == p1
    _tick(ll, rssi=-80.0, ts_ms=2100.0)
    assert ll.state.tx_power_dBm > p1


def test_tx_power_step_clamped_to_step_max():
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=0.0, tx_power_cooldown_ms=0.0,
        tx_power_step_max_db=3.0,
        tx_power_min_dBm=0.0, tx_power_max_dBm=30.0,
    )
    ll.state.tx_power_dBm = 10.0
    _tick(ll, rssi=-80.0, ts_ms=1000.0)
    step = ll.state.tx_power_dBm - 10.0
    assert step <= 3.0 + 1e-9


def test_tx_power_freezes_after_mcs_change():
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=2000.0, tx_power_cooldown_ms=0.0,
        snr_down_hold_ms=0.0, snr_margin_db=3.0,
        tx_power_min_dBm=0.0, tx_power_max_dBm=30.0,
    )
    ll.state.tx_power_dBm = 10.0
    # Force MCS drop via SNR below MCS1 floor (11). RSSI -80 would normally
    # bump TX power up, but the post-MCS freeze should hold it steady.
    _, _, changed = _tick(ll, snr=2.0, rssi=-80.0, ts_ms=0.0)
    assert changed
    pre = ll.state.tx_power_dBm
    _tick(ll, snr=2.0, rssi=-80.0, ts_ms=500.0)
    assert ll.state.tx_power_dBm == pre


def test_forced_mcs_drop_from_trailing_loop():
    ll = _loop()
    _tick(ll, snr=30.0, ts_ms=0.0)  # steady
    start_mcs = ll.current_row.mcs
    _, _, changed = _tick(ll, snr=30.0, ts_ms=100.0, forced_mcs_drop=True)
    assert changed
    assert ll.current_row.mcs < start_mcs


def test_force_tx_power_to_pins_power_immediately():
    """Starvation path uses force_tx_power_to to push to max regardless
    of cooldown / freeze."""
    ll = _loop(
        rssi_target_dBm=-60.0, rssi_deadband_db=3.0,
        tx_power_freeze_after_mcs_ms=2000.0, tx_power_cooldown_ms=1000.0,
        tx_power_min_dBm=0.0, tx_power_max_dBm=30.0,
    )
    ll.state.tx_power_dBm = 10.0
    _tick(ll, snr=30.0, ts_ms=0.0, force_tx_power_to=30.0)
    assert ll.state.tx_power_dBm == 30.0


def test_cfg_mcs_max_caps_row_table():
    """leading_loop.mcs_max from gs.yaml must clamp the runtime row
    table — even when the radio profile permits higher MCS."""
    ll = _loop(mcs_max=5, snr_up_hold_ms=0.0, snr_up_guard_db=0.0,
               forced_drop_inhibit_ms=0.0)
    assert max(r.mcs for r in ll.rows) == 5
    # Drive SNR way above MCS7 floor; the loop must not climb past 5.
    for ts in range(0, 5000, 100):
        _tick(ll, snr=40.0, ts_ms=float(ts))
    assert ll.current_row.mcs == 5


def test_cfg_mcs_max_too_low_raises():
    with pytest.raises(ValueError, match="mcs_max"):
        _loop(mcs_max=-1)
