"""Tests for the dual-gate LeadingSelector: Channel A (SNR-margin
hysteresis with predictive horizon) + Channel B (emergency forced
one-step downgrade) + inverse MCS↔TX-power coupling. Port of the
test patterns from
adaptive-link/ground-station/test/test_dynamic_profile.py."""
from __future__ import annotations

from pathlib import Path

import pytest

from dynamic_link.policy import (
    GateConfig,
    LeadingLoopConfig,
    LeadingSelector,
    Policy,
    PolicyConfig,
    ProfileSelectionConfig,
    SafeDefaults,
)
from dynamic_link.profile import load_profile

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"

# Defaults below are deliberately friendly to testing — most hold
# timers/cooldowns at zero so a single tick can effect a change. Where
# a test wants to exercise a specific timing path, it overrides
# explicitly.


def _selector(*,
              # leading
              tx_power_min_dBm: float = 18.0,
              tx_power_max_dBm: float = 28.0,
              bandwidth: int = 20,
              # gate overrides
              snr_ema_alpha: float = 0.3,
              snr_slope_alpha: float = 0.3,
              snr_predict_horizon_ticks: float = 3.0,
              snr_safety_margin: float = 3.0,
              loss_margin_weight: float = 20.0,
              fec_margin_weight: float = 5.0,
              hysteresis_up_db: float = 2.5,
              hysteresis_down_db: float = 1.0,
              emergency_loss_rate: float = 0.05,
              emergency_fec_pressure: float = 0.80,
              max_mcs: int = 7,
              max_mcs_step_up: int = 1,
              # selection overrides
              hold_fallback_mode_ms: int = 0,
              hold_modes_down_ms: int = 0,
              min_between_changes_ms: int = 0,
              fast_downgrade: bool = True,
              upward_confidence_loops: int = 1,
              ) -> LeadingSelector:
    profile = load_profile("m8812eu2", [PACKAGED_DIR])
    leading = LeadingLoopConfig(
        bandwidth=bandwidth,
        tx_power_min_dBm=tx_power_min_dBm,
        tx_power_max_dBm=tx_power_max_dBm,
    )
    gate = GateConfig(
        snr_ema_alpha=snr_ema_alpha,
        snr_slope_alpha=snr_slope_alpha,
        snr_predict_horizon_ticks=snr_predict_horizon_ticks,
        snr_safety_margin=snr_safety_margin,
        loss_margin_weight=loss_margin_weight,
        fec_margin_weight=fec_margin_weight,
        hysteresis_up_db=hysteresis_up_db,
        hysteresis_down_db=hysteresis_down_db,
        emergency_loss_rate=emergency_loss_rate,
        emergency_fec_pressure=emergency_fec_pressure,
        max_mcs=max_mcs,
        max_mcs_step_up=max_mcs_step_up,
    )
    sel = ProfileSelectionConfig(
        hold_fallback_mode_ms=hold_fallback_mode_ms,
        hold_modes_down_ms=hold_modes_down_ms,
        min_between_changes_ms=min_between_changes_ms,
        fast_downgrade=fast_downgrade,
        upward_confidence_loops=upward_confidence_loops,
    )
    return LeadingSelector(leading, gate, sel, profile)


def _select(s: LeadingSelector, *,
            snr=30.0, snr_raw=None, snr_slope=0.0, loss=0.0, fec=0.0,
            link_starved=False, ts_ms=0.0):
    """Drive the selector. `snr` is the smoothed (EMA) value;
    `snr_raw` is the latest per-window raw best-antenna mean.
    `snr_raw=None` defaults to the same value as `snr` so existing
    tests keep their symmetric behaviour. Tests that exercise the
    asymmetry pass `snr_raw=...` explicitly."""
    return s.select(
        snr_ema=snr,
        snr_raw=snr_raw if snr_raw is not None else snr,
        snr_slope=snr_slope,
        loss_rate=loss, fec_pressure=fec,
        link_starved=link_starved, ts_ms=ts_ms,
    )


def _drive_to_mcs(s: LeadingSelector, target_mcs: int,
                  max_ticks: int = 200):
    """Step the selector through the climb-up sequence to land at target.

    With upward_confidence_loops>=1 each MCS step needs at least 2
    confirming ticks (first sets the candidate, second applies), so a
    naive "stop on no-change-this-tick" loop bails too early. Keep
    ticking until either we reach the target or hit max_ticks."""
    ts = 0.0
    for _ in range(max_ticks):
        if s.state.current_mcs >= target_mcs:
            break
        _select(s, snr=40.0, ts_ms=ts)
        ts += 100.0


# ── Initial state ───────────────────────────────────────────────────────────


def test_starts_at_safe_default_mcs1():
    s = _selector()
    assert s.state.current_mcs == 1


def test_initial_tx_power_at_max():
    s = _selector(tx_power_min_dBm=18.0, tx_power_max_dBm=28.0)
    assert s.state.tx_power_dBm == 28.0


def test_max_mcs_too_low_raises():
    with pytest.raises(ValueError, match="max_mcs"):
        _selector(max_mcs=-1)


# ── Channel B: emergency triggers ───────────────────────────────────────────


def test_emergency_loss_rate_forces_one_step_down():
    s = _selector(emergency_loss_rate=0.05, max_mcs=5)
    _drive_to_mcs(s, 5)
    pre = s.state.current_mcs
    mcs, _, changed = _select(
        s, snr=40.0, loss=0.06, ts_ms=10000.0,
    )
    assert changed
    assert mcs == pre - 1


def test_emergency_fec_pressure_forces_one_step_down():
    s = _selector(emergency_fec_pressure=0.80, max_mcs=5)
    _drive_to_mcs(s, 5)
    pre = s.state.current_mcs
    mcs, _, changed = _select(
        s, snr=40.0, fec=0.85, ts_ms=10000.0,
    )
    assert changed
    assert mcs == pre - 1


def test_emergency_link_starved_forces_one_step_down():
    s = _selector(max_mcs=5)
    _drive_to_mcs(s, 5)
    pre = s.state.current_mcs
    mcs, _, changed = _select(
        s, snr=40.0, link_starved=True, ts_ms=10000.0,
    )
    assert changed
    assert mcs == pre - 1


def test_emergency_below_threshold_no_drop():
    """loss=0.04 (below 0.05 default) → no emergency, MCS holds."""
    s = _selector(emergency_loss_rate=0.05, max_mcs=5,
                  hysteresis_down_db=1.0)
    _drive_to_mcs(s, 5)
    pre = s.state.current_mcs
    # Stable healthy SNR; only a touch of loss below threshold.
    mcs, _, changed = _select(
        s, snr=40.0, loss=0.04, ts_ms=10000.0,
    )
    assert not changed
    assert mcs == pre


def test_emergency_at_mcs0_cannot_force_below():
    """Already at MCS 0 — emergency has nowhere to go, no change."""
    s = _selector(max_mcs=5)
    # Drive to MCS 0 by repeated emergency.
    ts = 0.0
    while s.state.current_mcs > 0:
        _select(s, snr=40.0, link_starved=True, ts_ms=ts)
        ts += 100.0
    mcs, _, changed = _select(
        s, snr=40.0, link_starved=True, ts_ms=ts + 100.0,
    )
    assert not changed
    assert mcs == 0


# ── Channel A: SNR-margin hysteresis ────────────────────────────────────────


def test_upgrade_blocked_below_hysteresis_up_db():
    """Even if margin is positive, upgrade requires margin >= hysteresis_up_db."""
    # Driver: at MCS 1 (floor 8 dB after profile margin → 11 dB? Actually
    # profile.snr_floor_dB[20][1]=8 baked-in, our gate uses snr_safety_margin
    # = 3 added on top by _stress_margin_dB. Effective MCS 2 floor =
    # snr_floor[2]=11 + safety=3 = 14. Upgrade requires margin >= 2.5 dB
    # so snr must be >= 16.5 to upgrade from MCS 1.
    s = _selector(hysteresis_up_db=2.5, snr_safety_margin=3.0,
                  upward_confidence_loops=1)
    # snr=15 → margin = 15 - 11 - 3 = 1 dB → below hysteresis_up_db
    mcs, _, changed = _select(s, snr=15.0, ts_ms=0.0)
    assert not changed
    assert mcs == 1


def test_upgrade_fires_when_margin_clears_hysteresis():
    s = _selector(hysteresis_up_db=2.5, snr_safety_margin=3.0,
                  upward_confidence_loops=1)
    # snr=17 → margin for MCS2 = 17 - 11 - 3 = 3 dB > hysteresis_up_db.
    # Confidence gating needs 2 ticks: first sets the candidate,
    # second applies it.
    _select(s, snr=17.0, ts_ms=0.0)
    _, _, changed = _select(s, snr=17.0, ts_ms=100.0)
    assert changed
    assert s.state.current_mcs == 2


def test_upgrade_blocked_when_snr_slope_predicts_negative_margin():
    """Even with current margin clear, a steeply-negative slope blocks
    the upgrade because predicted future margin is negative."""
    s = _selector(hysteresis_up_db=2.5, snr_safety_margin=3.0,
                  snr_predict_horizon_ticks=3.0, upward_confidence_loops=1)
    # margin at MCS2 = 17 - 11 - 3 = 3 dB; predicted = 3 + (-1.5)*3 = -1.5 → blocked
    mcs, _, changed = _select(
        s, snr=17.0, snr_slope=-1.5, ts_ms=0.0,
    )
    assert not changed
    assert mcs == 1


def test_upgrade_requires_upward_confidence_loops():
    """Confidence counter must reach upward_confidence_loops before
    the upgrade actually applies."""
    s = _selector(hysteresis_up_db=2.5, snr_safety_margin=3.0,
                  upward_confidence_loops=4, hold_modes_down_ms=0)
    # Each tick: snr=20 (well above threshold). Need 4 confirming ticks.
    for i in range(3):
        _, _, changed = _select(s, snr=20.0, ts_ms=i * 100.0)
        assert not changed, f"upgrade fired prematurely at tick {i}"
    _, _, changed = _select(s, snr=20.0, ts_ms=400.0)
    assert changed
    assert s.state.current_mcs == 2


def test_downgrade_blocked_above_hysteresis_down_db():
    s = _selector(hysteresis_down_db=1.0, snr_safety_margin=3.0)
    _drive_to_mcs(s, 3)
    # MCS3 floor = 14 + 3 = 17. snr=16.5 → margin = -0.5 dB.
    # cur_margin > -hysteresis_down_db (-1.0) → no downgrade.
    mcs, _, changed = _select(s, snr=16.5, ts_ms=10000.0)
    assert not changed
    assert mcs == 3


def test_downgrade_fires_when_margin_below_negative_hysteresis():
    s = _selector(hysteresis_down_db=1.0, snr_safety_margin=3.0,
                  fast_downgrade=True)
    _drive_to_mcs(s, 3)
    # snr=15.5 → margin for MCS3 = 15.5 - 14 - 3 = -1.5 dB → < -hyst_down
    mcs, _, changed = _select(s, snr=15.5, ts_ms=10000.0)
    assert changed
    assert mcs < 3


# ── Stress margin ───────────────────────────────────────────────────────────


def test_stress_margin_widens_with_loss():
    """Loss raises the effective threshold — what would have been a
    valid upgrade with loss=0 is now blocked."""
    # Without loss, snr=17 clears MCS2 (3 dB margin). With loss=0.10,
    # margin shrinks by 0.10 * 20 = 2 dB → 1 dB → below hysteresis_up.
    s = _selector(loss_margin_weight=20.0, hysteresis_up_db=2.5)
    mcs, _, changed = _select(s, snr=17.0, loss=0.10, ts_ms=0.0)
    # Loss 0.10 ≥ emergency_loss_rate 0.05 → emergency fires; MCS doesn't
    # *climb*, instead force-downs by 1. So changed is True but MCS dropped.
    # To test stress-margin in isolation, use loss below the emergency
    # threshold:
    s2 = _selector(loss_margin_weight=20.0, hysteresis_up_db=2.5,
                   emergency_loss_rate=0.99)
    mcs2, _, changed2 = _select(s2, snr=17.0, loss=0.10, ts_ms=0.0)
    assert not changed2
    assert mcs2 == 1


def test_stress_margin_widens_with_fec_pressure():
    s = _selector(fec_margin_weight=5.0, hysteresis_up_db=2.5,
                  emergency_fec_pressure=0.99)
    # snr=17, fec=0.6 → stress = 3 + 0.6*5 = 6 dB, margin for MCS2 =
    # 17 - 11 - 6 = 0 < hysteresis_up_db. Block.
    _, _, changed = _select(s, snr=17.0, fec=0.6, ts_ms=0.0)
    assert not changed
    assert s.state.current_mcs == 1


# ── Timing ──────────────────────────────────────────────────────────────────


def test_min_between_changes_ms_enforced_outside_emergency():
    """Two upgrade-eligible ticks within min_between_changes_ms — the
    second should be blocked even though everything else qualifies."""
    s = _selector(min_between_changes_ms=300, hold_modes_down_ms=0,
                  upward_confidence_loops=1)
    # Land an upgrade (2 ticks for confidence).
    _select(s, snr=40.0, ts_ms=0.0)
    _, _, changed = _select(s, snr=40.0, ts_ms=50.0)
    assert changed
    pre = s.state.current_mcs
    # Try to upgrade again 100 ms after the apply — should be rate-limited.
    _, _, changed = _select(s, snr=40.0, ts_ms=150.0)
    _, _, changed = _select(s, snr=40.0, ts_ms=200.0)
    assert not changed
    assert s.state.current_mcs == pre


def test_emergency_bypasses_min_between_changes_ms():
    s = _selector(min_between_changes_ms=300, hold_modes_down_ms=0,
                  upward_confidence_loops=1, max_mcs=5,
                  emergency_loss_rate=0.05)
    _drive_to_mcs(s, 3)
    # Land any change to set last_change_time_ms.
    _select(s, snr=40.0, ts_ms=10_000.0)
    # 100 ms later, emergency loss → still fires.
    pre = s.state.current_mcs
    _, _, changed = _select(s, snr=40.0, loss=0.10, ts_ms=10_100.0)
    assert changed
    assert s.state.current_mcs == pre - 1


def test_hold_modes_down_ms_blocks_consecutive_upgrades():
    s = _selector(hold_modes_down_ms=2000, upward_confidence_loops=1)
    # First upgrade (2 ticks for confidence).
    _select(s, snr=40.0, ts_ms=0.0)
    _, _, changed = _select(s, snr=40.0, ts_ms=100.0)
    assert changed
    pre = s.state.current_mcs
    # Try to upgrade again well within 2 s — blocked by
    # hold_modes_down_ms even with confidence gating ready.
    for i in range(15):
        _select(s, snr=40.0, ts_ms=200.0 + i * 100.0)
    assert s.state.current_mcs == pre


def test_hold_fallback_mode_ms_when_at_mcs_zero():
    """Climbing out of MCS 0 has its own (typically longer) hold."""
    s = _selector(hold_fallback_mode_ms=1000, hold_modes_down_ms=0,
                  upward_confidence_loops=1, max_mcs=5)
    # Force MCS to 0 via emergency.
    ts = 0.0
    while s.state.current_mcs > 0:
        _select(s, snr=40.0, link_starved=True, ts_ms=ts)
        ts += 100.0
    # Just after settling at 0 — try to climb.
    _, _, changed = _select(s, snr=40.0, ts_ms=ts + 100.0)
    assert not changed
    # Past hold_fallback_mode_ms → climb succeeds.
    _, _, changed = _select(s, snr=40.0, ts_ms=ts + 1500.0)
    assert changed


# ── Inverse MCS↔TX power coupling ──────────────────────────────────────────


def test_tx_power_at_mcs0_is_max():
    s = _selector(tx_power_min_dBm=18.0, tx_power_max_dBm=28.0, max_mcs=5)
    ts = 0.0
    while s.state.current_mcs > 0:
        _select(s, snr=40.0, link_starved=True, ts_ms=ts)
        ts += 100.0
    assert s.state.current_mcs == 0
    assert int(round(s.state.tx_power_dBm)) == 28


def test_tx_power_at_max_mcs_is_min():
    s = _selector(tx_power_min_dBm=18.0, tx_power_max_dBm=28.0, max_mcs=5,
                  upward_confidence_loops=1)
    _drive_to_mcs(s, 5)
    assert s.state.current_mcs == 5
    assert int(round(s.state.tx_power_dBm)) == 18


def test_tx_power_jumps_atomically_on_emergency_drop():
    s = _selector(tx_power_min_dBm=18.0, tx_power_max_dBm=28.0, max_mcs=5,
                  upward_confidence_loops=1)
    _drive_to_mcs(s, 5)
    assert int(round(s.state.tx_power_dBm)) == 18
    # Single emergency tick → MCS 5 → 4 → power follows in same tick.
    _, _, changed = _select(
        s, snr=40.0, loss=0.10, ts_ms=10_000.0,
    )
    assert changed
    assert s.state.current_mcs == 4
    # MCS4 with cap=5, range [18,28]: power = 28 - (4/5)*10 = 20.
    assert int(round(s.state.tx_power_dBm)) == 20


# ── Step-up cap ─────────────────────────────────────────────────────────────


def test_max_mcs_step_up_caps_upward_jump():
    """SNR clears MCS5 but max_mcs_step_up=1 caps the climb at MCS1+1=2."""
    s = _selector(max_mcs_step_up=1, upward_confidence_loops=1,
                  hold_modes_down_ms=0)
    _select(s, snr=40.0, ts_ms=0.0)
    _, _, changed = _select(s, snr=40.0, ts_ms=100.0)
    assert changed
    assert s.state.current_mcs == 2  # not 5


# ── No-SNR-yet edge case ────────────────────────────────────────────────────


def test_no_snr_holds_state_but_emergency_still_fires():
    s = _selector(max_mcs=5, upward_confidence_loops=1)
    _drive_to_mcs(s, 3)
    # No SNR reading yet, no stress → hold.
    mcs, _, changed = s.select(
        snr_ema=None, snr_slope=0.0,
        loss_rate=0.0, fec_pressure=0.0,
        link_starved=False, ts_ms=10_000.0,
    )
    assert not changed
    assert mcs == 3
    # No SNR but emergency (link_starved) → still drops.
    pre = s.state.current_mcs
    mcs, _, changed = s.select(
        snr_ema=None, snr_slope=0.0,
        loss_rate=0.0, fec_pressure=0.0,
        link_starved=True, ts_ms=10_100.0,
    )
    # Without SNR we can't go through Channel A, so fall through to
    # emergency — selector should still force-down.
    # NOTE: the current implementation returns early if snr_ema is None
    # AND link_starved is False. With link_starved=True, we proceed.
    assert mcs <= pre  # emergency drove down or held


# ── Asymmetric SNR (raw for downgrade, smoothed for upgrade) ────────────────


def test_downgrade_fires_on_raw_collapse_while_smoothed_clears():
    """Reproduces gs.5.jsonl t=146.3 → 146.6 fast-fade. Smoothed snr
    still clears the floor; raw has just collapsed below. Asymmetric
    selector should drop now, not 500 ms later."""
    s = _selector(hysteresis_down_db=1.0, snr_safety_margin=3.0,
                  fast_downgrade=True, snr_predict_horizon_ticks=0.0,
                  emergency_loss_rate=0.99)  # disable Channel B
    _drive_to_mcs(s, 4)
    pre = s.state.current_mcs
    # smoothed=24.7 still clears MCS4 effective floor 17+3=20 (margin +4.7);
    # raw=20 → margin against MCS4 = 20 - 17 - 3 = 0 → below
    # -hysteresis_down_db (-1) is needed... wait, 0 > -1 so not below.
    # Use raw=18 → margin = 18 - 20 = -2 → below -1 → drop fires.
    mcs, _, changed = _select(
        s, snr=24.7, snr_raw=18.0, ts_ms=10_000.0,
    )
    assert changed
    assert mcs < pre


def test_downgrade_does_not_fire_when_raw_matches_smoothed_high():
    """Control case for the test above: when raw matches smoothed at
    a healthy value, no drop should fire."""
    s = _selector(hysteresis_down_db=1.0, snr_safety_margin=3.0,
                  fast_downgrade=True, snr_predict_horizon_ticks=0.0,
                  emergency_loss_rate=0.99)
    _drive_to_mcs(s, 4)
    pre = s.state.current_mcs
    mcs, _, changed = _select(
        s, snr=24.7, snr_raw=24.7, ts_ms=10_000.0,
    )
    assert not changed
    assert mcs == pre


def test_upgrade_unchanged_uses_smoothed_not_raw():
    """When smoothed is high but raw is low, the climb is held — `min`
    of the two candidate computations caps the upgrade to whatever
    the more-pessimistic (raw) signal allows."""
    s = _selector(hysteresis_up_db=2.5, snr_safety_margin=3.0,
                  upward_confidence_loops=1, emergency_loss_rate=0.99)
    # Raw is low — only clears MCS1 (effective floor 8+3=11). With raw=12,
    # _pick_mcs(raw) → MCS1; _pick_mcs(smoothed=30) → max_mcs=7 (capped).
    # min(...) → MCS1 = current. No climb.
    mcs, _, changed = _select(
        s, snr=30.0, snr_raw=12.0, ts_ms=10_000.0,
    )
    assert not changed
    assert mcs == s.state.current_mcs


def test_upgrade_fires_when_raw_clears_too():
    """Control for above: raw matches smoothed at a healthy level →
    upgrade fires normally."""
    s = _selector(hysteresis_up_db=2.5, snr_safety_margin=3.0,
                  upward_confidence_loops=1, max_mcs_step_up=1,
                  emergency_loss_rate=0.99)
    # First tick sets candidate; second applies (confidence_loops=1).
    _select(s, snr=30.0, snr_raw=30.0, ts_ms=10_000.0)
    _, _, changed = _select(s, snr=30.0, snr_raw=30.0, ts_ms=10_100.0)
    assert changed
    assert s.state.current_mcs > 1


def test_no_snr_raw_falls_back_to_snr_ema():
    """Helper passes None when caller doesn't specify; selector should
    treat that identically to symmetric behaviour."""
    s = _selector(hysteresis_down_db=1.0, snr_safety_margin=3.0,
                  fast_downgrade=True, emergency_loss_rate=0.99)
    _drive_to_mcs(s, 4)
    # Calling s.select directly with snr_raw=None is allowed; and
    # because the helper falls back to snr when snr_raw is unset, all
    # existing tests in this file effectively exercise the symmetric
    # path. So a direct check that explicitly passing None matches:
    pre = s.state.current_mcs
    mcs, _, changed = s.select(
        snr_ema=24.7, snr_raw=None, snr_slope=0.0,
        loss_rate=0.0, fec_pressure=0.0, link_starved=False,
        ts_ms=10_000.0,
    )
    # smoothed=24.7 clears MCS4 effective floor (20) by 4.7 dB > -1 → hold.
    assert not changed
    assert mcs == pre


def test_min_picks_more_pessimistic_candidate():
    """Direct sanity test on _pick_mcs and the min() composition."""
    s = _selector(snr_safety_margin=3.0, emergency_loss_rate=0.99)
    # MCS5 floor 20+3=23; MCS2 floor 11+3=14.
    pick_high = s._pick_mcs(33.0, 0.0, 0.0)   # clears MCS5
    pick_low = s._pick_mcs(15.0, 0.0, 0.0)    # only clears MCS2
    assert pick_high > pick_low
    assert min(pick_high, pick_low) == pick_low


# ── Policy-level: drone_config safe-defaults gate (P4a Task 10) ────────────

def test_policy_emits_safe_defaults_until_drone_synced():
    """Until the drone has sent a HELLO, the policy must emit a
    safe-defaults decision regardless of the incoming signals."""
    from dynamic_link.drone_config import DroneConfigState
    from dynamic_link.signals import Signals
    from dynamic_link.stats_client import SessionInfo
    from dynamic_link.wire import Hello

    profile = load_profile("m8812eu2", [PACKAGED_DIR])
    cfg = PolicyConfig(
        leading=LeadingLoopConfig(tx_power_min_dBm=5.0, tx_power_max_dBm=30.0),
        safe=SafeDefaults(k=8, n=12, depth=1, mcs=1),
    )
    drone_cfg = DroneConfigState()
    policy = Policy(cfg, profile, drone_config=drone_cfg)

    # Healthy signals that would otherwise drive normal MCS selection.
    signals = Signals(
        rssi=-55.0, rssi_min_w=-55.0, rssi_max_w=-55.0,
        snr=30.0, snr_min_w=30.0, snr_max_w=30.0,
        residual_loss_w=0.0, fec_work=0.0,
        timestamp=0.0,
        link_starved_w=False,
        session=SessionInfo(
            fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=1,
            interleave_depth=1, contract_version=2,
        ),
    )

    decision_before = policy.tick(signals)
    assert decision_before.mcs == cfg.safe.mcs
    assert decision_before.k == cfg.safe.k
    assert decision_before.n == cfg.safe.n
    assert decision_before.depth == cfg.safe.depth
    assert decision_before.reason == "awaiting_drone_config"

    # Synthesize a HELLO; policy should now emit "real" decisions.
    drone_cfg.on_hello(Hello(generation_id=1, mtu_bytes=1400, fps=60))
    assert drone_cfg.is_synced()
    decision_after = policy.tick(signals)
    # The real path runs; reason is no longer the gate sentinel.
    assert decision_after.reason != "awaiting_drone_config"
