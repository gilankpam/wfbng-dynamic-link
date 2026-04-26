"""Tests for the trailing loop: loss-driven escalation (§4.2)."""
from __future__ import annotations

from pathlib import Path

from dynamic_link.policy import (
    LeadingLoopConfig,
    Policy,
    PolicyConfig,
    SafeDefaults,
    TrailingLoop,
    _ladder_step_down,
    _ladder_step_up,
)
from dynamic_link.predictor import LADDER_CLIMB, LADDER_STEPS
from dynamic_link.profile import load_profile
from dynamic_link.signals import Signals
from dynamic_link.stats_client import SessionInfo

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


def _sigs(
    residual_loss_w: float = 0.0,
    fec_work: float = 0.0,
    burst_rate: float = 0.0,
    holdoff_rate: float = 0.0,
    ts: float = 0.0,
) -> Signals:
    return Signals(
        residual_loss_w=residual_loss_w,
        fec_work=fec_work,
        burst_rate=burst_rate,
        holdoff_rate=holdoff_rate,
        timestamp=ts,
        session=SessionInfo(
            fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=1,
            interleave_depth=1, contract_version=2,
        ),
    )


def test_residual_loss_triggers_same_tick_no_cooldown():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    k, n, depth, idr = tl.tick(
        _sigs(residual_loss_w=0.05, ts=0.0),
        current_k=8, current_n=12, current_depth=1, ts_ms=0.0,
    )
    # Step up within k=8 band: (8,12) -> (8,14)
    assert (k, n) == (8, 14)
    assert idr is True


def test_ladder_k8_progression():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    k, n = 8, 12
    # First loss window — 200 ms cooldown is bypassed on residual_loss.
    k, n, _, _ = tl.tick(_sigs(residual_loss_w=0.05, ts=0.0),
                         k, n, 1, ts_ms=0.0)
    assert (k, n) == (8, 14)
    k, n, _, _ = tl.tick(_sigs(residual_loss_w=0.05, ts=0.3),
                         k, n, 1, ts_ms=300.0)
    assert (k, n) == (8, 16)
    k, n, _, _ = tl.tick(_sigs(residual_loss_w=0.05, ts=0.6),
                         k, n, 1, ts_ms=600.0)
    # Step 3: drop to (6, 12) per LADDER_DROP.
    assert (k, n) == (6, 12)


def test_ladder_steps_match_design_table():
    """§4.2 table rows vs implementation constants."""
    assert LADDER_STEPS[8] == [(8, 12), (8, 14), (8, 16)]
    assert LADDER_STEPS[6] == [(6, 10), (6, 12), (6, 14)]
    assert LADDER_STEPS[4] == [(4, 8), (4, 10), (4, 12)]
    assert LADDER_STEPS[2] == [(2, 4), (2, 6), (2, 8)]


def test_fec_work_rising_preemptive_step_respects_cooldown():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    # High fec_work but no loss. First tick at t=0 is inside the
    # 200 ms cooldown window from the last_fec_change_ts=0 initial —
    # so nothing fires this tick.
    k, n, _, idr = tl.tick(
        _sigs(fec_work=0.2, ts=0.0),
        current_k=8, current_n=12, current_depth=1, ts_ms=0.0,
    )
    assert (k, n) == (8, 12)
    # Past the cooldown → preemptive raise fires; no IDR.
    k, n, _, idr = tl.tick(
        _sigs(fec_work=0.2, ts=0.3),
        current_k=8, current_n=12, current_depth=1, ts_ms=300.0,
    )
    assert (k, n) == (8, 14)
    assert idr is False


def test_no_escalation_when_clean():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    k, n, d, idr = tl.tick(
        _sigs(residual_loss_w=0.0, fec_work=0.0, ts=0.0),
        current_k=8, current_n=12, current_depth=1, ts_ms=0.0,
    )
    assert (k, n, d) == (8, 12, 1)
    assert idr is False


def test_sustained_loss_detection():
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    for t in range(3):
        tl.tick(
            _sigs(residual_loss_w=0.01, ts=t * 0.1),
            current_k=8, current_n=14, current_depth=1,
            ts_ms=t * 100.0,
        )
    assert tl.sustained_loss() is True


def test_depth_raised_when_burst_and_holdoff_together():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    # Tick past the 200 ms depth cooldown from bootstrap.
    _, _, depth, _ = tl.tick(
        _sigs(residual_loss_w=0.02, burst_rate=5.0, holdoff_rate=2.0, ts=0.3),
        current_k=8, current_n=12, current_depth=1, ts_ms=300.0,
    )
    assert depth == 2


def test_ladder_holds_at_k2_floor_does_not_emit_k1():
    """At the top of the k=2 band (=last entry of LADDER_STEPS[2]),
    sustained loss must NOT step the ladder to (1, 4). k=1 is below
    every airframe's ceiling and would be rejected by the drone."""
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    # Walk to (2, 8): the k=2 band's top. Bypass cooldown via residual_loss.
    pairs = [(2, 4), (2, 6), (2, 8)]
    k, n = 2, 4
    for i, expected in enumerate(pairs[1:], start=1):
        k, n, _, _ = tl.tick(
            _sigs(residual_loss_w=0.05, ts=i * 0.3),
            current_k=k, current_n=n, current_depth=1, ts_ms=i * 300.0,
        )
        assert (k, n) == expected, f"step {i}"
    # One more loss tick at (2, 8) must NOT drop to (1, 4).
    k, n, _, _ = tl.tick(
        _sigs(residual_loss_w=0.05, ts=1.2),
        current_k=2, current_n=8, current_depth=1, ts_ms=1200.0,
    )
    assert (k, n) == (2, 8)


def test_depth_not_raised_on_burst_alone():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    _, _, depth, _ = tl.tick(
        _sigs(residual_loss_w=0.02, burst_rate=5.0, holdoff_rate=0.0, ts=0.3),
        current_k=8, current_n=12, current_depth=1, ts_ms=300.0,
    )
    assert depth == 1


def test_depth_bootstrap_fires_on_sustained_loss_plus_busy_fec():
    """At depth=1 wfb-ng's interleaver code path is bypassed
    (rx.cpp:357), so burst_rate/holdoff_rate are structurally zero
    and can't trigger the first depth raise. The bootstrap branch uses
    sustained loss + fec_work as a proxy. Once depth>=2 the interleaver
    is engaged and the original burst+holdoff trigger takes over."""
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    # Build sustained loss across 3 consecutive windows. Depth stays at
    # 1 until the third tick because sustained_loss requires all N
    # windows in history to have loss.
    depths = []
    for i in range(3):
        _, _, depth, _ = tl.tick(
            _sigs(residual_loss_w=0.05, fec_work=0.20, ts=(i + 1) * 0.3),
            current_k=8, current_n=12, current_depth=1, ts_ms=(i + 1) * 300.0,
        )
        depths.append(depth)
    # First two ticks: not enough history for sustained_loss → depth=1.
    # Third tick: history full + still lossy + busy FEC → bootstrap fires.
    assert depths[-1] == 2, f"expected bootstrap to depth=2, got {depths}"


def test_depth_bootstrap_requires_busy_fec_not_just_sustained_loss():
    """Sustained loss with only marginal FEC work shouldn't bootstrap —
    the link might just be poor on RSSI; depth=1 is fine until FEC is
    actually being exercised (which is the signal that loss is bursty
    enough that interleaving will help)."""
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    for i in range(3):
        _, _, depth, _ = tl.tick(
            _sigs(residual_loss_w=0.05, fec_work=0.05, ts=(i + 1) * 0.3),
            current_k=8, current_n=12, current_depth=1, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 1


def test_depth_steps_down_after_sustained_clean_link():
    """After enough consecutive zero-loss windows, depth reclaims one
    step. Walks down one step per threshold period, not all at once,
    so the link can confirm it tolerates the lower depth before
    further reductions."""
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=5)
    tl = TrailingLoop(cfg)
    # Start at depth=3. Feed clean windows; depth should drop after 5.
    depth = 3
    last_depth = depth
    for i in range(7):
        _, _, depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_k=8, current_n=12, current_depth=depth,
            ts_ms=(i + 1) * 300.0,
        )
    # After 5 clean windows we should have stepped down once (3 → 2).
    assert depth == 2, f"expected step-down to 2, got {depth}"


def test_depth_walks_down_one_step_at_a_time():
    """Two thresholds met → two step-downs, not one big jump."""
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=3)
    tl = TrailingLoop(cfg)
    depth = 3
    seen = []
    for i in range(10):
        _, _, depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_k=8, current_n=12, current_depth=depth,
            ts_ms=(i + 1) * 300.0,
        )
        seen.append(depth)
    # 3 clean → step to 2; counter resets; 3 more clean → step to 1.
    # Should never reach 0.
    assert min(seen) == 1
    assert seen.count(2) >= 1, f"never hit depth=2: {seen}"


def test_depth_does_not_step_down_with_recent_loss():
    """A single loss tick resets the clean-window counter, so
    subsequent clean ticks must rebuild before step-down can fire."""
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=5)
    tl = TrailingLoop(cfg)
    # 4 clean ticks, then a loss, then 3 more clean — total 7 ticks
    # but the loss reset means we never accumulate 5 consecutive clean.
    depth = 3
    for i in range(4):
        _, _, depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_k=8, current_n=12, current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    _, _, depth, _ = tl.tick(
        _sigs(residual_loss_w=0.05, ts=1.5),
        current_k=8, current_n=12, current_depth=depth, ts_ms=1500.0,
    )
    for i in range(3):
        _, _, depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=1.8 + i * 0.3),
            current_k=8, current_n=12, current_depth=depth,
            ts_ms=1800.0 + i * 300.0,
        )
    assert depth == 3


def _policy(**overrides) -> Policy:
    """Build a Policy backed by the packaged radio profile."""
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    leading_overrides = overrides.pop("leading", {})
    leading = LeadingLoopConfig(
        tx_power_min_dBm=5.0,
        tx_power_max_dBm=30.0,
        **leading_overrides,
    )
    cfg = PolicyConfig(
        leading=leading,
        safe=SafeDefaults(k=8, n=12, depth=1, mcs=1, bitrate_kbps=2000),
        **overrides,
    )
    return Policy(cfg, prof)


def _starved_sigs(ts_ms: float, *, link_starved: bool) -> Signals:
    """Build a Signals snapshot driving the starvation path. snr=30 keeps
    the leading loop's MCS hysteresis quiet; only link_starved_w moves."""
    return Signals(
        rssi=-55.0, rssi_min_w=-55.0, rssi_max_w=-55.0,
        snr=30.0, snr_min_w=30.0, snr_max_w=30.0,
        residual_loss_w=0.0,
        timestamp=ts_ms / 1000.0,
        link_starved_w=link_starved,
        session=SessionInfo(
            fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=1,
            interleave_depth=1, contract_version=2,
        ),
    )


def _settle_at_max_mcs(p, base_ts: float = 0.0) -> float:
    """Drive the policy up to its MCS cap with healthy non-starved signals.
    Returns the next ts to use."""
    ts = base_ts
    for _ in range(200):
        if p.state.mcs >= p.cfg.gate.max_mcs:
            break
        p.tick(_starved_sigs(ts, link_starved=False))
        ts += 100.0
    return ts


def test_starvation_needs_n_consecutive_windows():
    """Per-tick link_starved_w is too noisy on bursty video traffic
    to trigger emergency directly — need N consecutive starved
    windows. starvation_windows=5 is the default."""
    p = _policy(starvation_windows=5)
    ts = _settle_at_max_mcs(p)
    start_mcs = p.state.mcs
    # 4 starved ticks → still no drop.
    for _ in range(4):
        d = p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
    assert d.mcs == start_mcs
    # 5th consecutive starved → emergency fires.
    d = p.tick(_starved_sigs(ts, link_starved=True))
    assert d.mcs < start_mcs
    assert "emergency" in d.reason and "starved" in d.reason


def test_starvation_resets_on_recovery():
    """A non-starved window in the middle resets the accumulator."""
    p = _policy(starvation_windows=5)
    ts = _settle_at_max_mcs(p)
    start_mcs = p.state.mcs
    # 3 starved
    for _ in range(3):
        p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
    # 1 healthy → resets
    p.tick(_starved_sigs(ts, link_starved=False))
    ts += 100.0
    # 4 more starved → only 4 in a row, threshold 5, no drop.
    for _ in range(4):
        d = p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
    assert d.mcs == start_mcs


def test_starvation_pins_tx_power_to_max():
    """Inverse coupling: when sustained starvation drives MCS to 0,
    TX power follows to max via inverse coupling."""
    p = _policy(starvation_windows=5)
    p.leading.state.tx_power_dBm = 10.0
    ts = 10_000.0
    for _ in range(50):
        d = p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
        if d.mcs == 0:
            break
    assert d.mcs == 0
    assert d.tx_power_dBm == 30


def test_starvation_holds_at_floor_no_climb():
    """Continued sustained starvation past the floor doesn't climb
    back up — the emergency-at-floor guard pins MCS at the bottom."""
    p = _policy(starvation_windows=5)
    ts = 10_000.0
    for _ in range(50):
        p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
    # 50 more starved ticks — should stay pinned at the floor.
    for _ in range(50):
        d = p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
        assert d.mcs == 0, f"climbed to {d.mcs} during sustained starvation"


def test_depth_bootstrap_is_one_shot_at_depth_1():
    """Bootstrap branch only fires at current_depth==1. At depth>=2 we
    rely on the interleaver-internal signals (burst_rate + holdoff_rate),
    which are now valid because the interleaver is on."""
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    # Seed sustained loss + busy FEC, but start at depth=2 so the
    # bootstrap gate (current_depth==1) doesn't apply. Without the
    # interleaver-internal trigger (burst/holdoff), depth stays put.
    for i in range(3):
        _, _, depth, _ = tl.tick(
            _sigs(residual_loss_w=0.05, fec_work=0.20, ts=(i + 1) * 0.3),
            current_k=8, current_n=12, current_depth=2, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 2


# ── FEC ladder step-down ─────────────────────────────────────────────────────


# Pure helper tests on _ladder_step_down + LADDER_CLIMB.

def test_ladder_step_down_within_band_decrements_n():
    assert _ladder_step_down(8, 16) == (8, 14)
    assert _ladder_step_down(8, 14) == (8, 12)
    assert _ladder_step_down(6, 14) == (6, 12)
    assert _ladder_step_down(6, 12) == (6, 10)
    assert _ladder_step_down(4, 12) == (4, 10)
    assert _ladder_step_down(4, 10) == (4, 8)
    assert _ladder_step_down(2, 8) == (2, 6)
    assert _ladder_step_down(2, 6) == (2, 4)


def test_ladder_step_down_at_band_floor_climbs_to_previous_band_top():
    assert _ladder_step_down(6, 10) == (8, 16)
    assert _ladder_step_down(4, 8) == (6, 14)
    assert _ladder_step_down(2, 4) == (4, 12)


def test_ladder_step_down_at_band_8_floor_holds():
    """No band above k=8 — step-down at (8, 12) holds in place."""
    assert _ladder_step_down(8, 12) == (8, 12)


def test_ladder_step_down_off_ladder_holds():
    """Inputs not in any LADDER_STEPS rung return unchanged."""
    assert _ladder_step_down(8, 11) == (8, 11)   # not a real rung
    assert _ladder_step_down(7, 12) == (7, 12)   # k=7 not a band
    assert _ladder_step_down(2, 5) == (2, 5)     # not a rung


def test_ladder_full_traversal_to_band_8_floor():
    """Walking step-down from (2, 8) hits every LADDER_STEPS rung in
    reverse and lands at (8, 12) after 11 steps."""
    expected = [
        (2, 8), (2, 6), (2, 4),
        (4, 12), (4, 10), (4, 8),
        (6, 14), (6, 12), (6, 10),
        (8, 16), (8, 14), (8, 12),
    ]
    trace = [(2, 8)]
    for _ in range(11):
        trace.append(_ladder_step_down(*trace[-1]))
    assert trace == expected


def test_ladder_climb_inverts_band_floor_transitions():
    """LADDER_CLIMB keys are band-floor rungs; values are the top
    rung of the previous (higher-k) band."""
    for floor, top in LADDER_CLIMB.items():
        assert floor in (band[0] for band in LADDER_STEPS.values())
        # top is the highest-n rung of the band above
        floor_band_idx = sorted(LADDER_STEPS.keys(), reverse=True).index(floor[0])
        higher_k = sorted(LADDER_STEPS.keys(), reverse=True)[floor_band_idx - 1]
        assert top == LADDER_STEPS[higher_k][-1]


# Trailing-loop integration tests.

def _idle_sigs(ts_ms: float) -> Signals:
    """Clean & FEC-idle window."""
    return _sigs(residual_loss_w=0.0, fec_work=0.0, ts=ts_ms / 1000.0)


def test_fec_stepdown_after_clean_windows():
    """10 consecutive clean+idle windows → one rung of step-down."""
    cfg = PolicyConfig(clean_windows_for_fec_stepdown=10,
                       fec_work_idle_threshold=0.02)
    cfg.cooldown.min_change_interval_ms_fec = 0
    tl = TrailingLoop(cfg)
    k, n, ts = 2, 8, 0.0
    # 9 clean windows: counter at 9, no step yet.
    for _ in range(9):
        ts += 100.0
        k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                             stepdown_floor=(8, 12))
    assert (k, n) == (2, 8)
    # 10th tick → step-down fires.
    ts += 100.0
    k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                         stepdown_floor=(8, 12))
    assert (k, n) == (2, 6)


def test_fec_stepdown_blocked_by_loss_in_window():
    """A single lossy tick mid-clean resets the counter; step-down
    doesn't fire."""
    cfg = PolicyConfig(clean_windows_for_fec_stepdown=10,
                       fec_work_idle_threshold=0.02)
    cfg.cooldown.min_change_interval_ms_fec = 0
    tl = TrailingLoop(cfg)
    k, n, ts = 2, 8, 0.0
    # 5 clean
    for _ in range(5):
        ts += 100.0
        k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                             stepdown_floor=(8, 12))
    # 1 lossy tick — counter resets (and step-up fires)
    ts += 100.0
    k, n, _, _ = tl.tick(
        _sigs(residual_loss_w=0.10, ts=ts / 1000.0),
        k, n, 1, ts, stepdown_floor=(8, 12),
    )
    # 5 more clean — total 10 clean post-loss but counter was reset
    # at tick 6, so we're at counter=5, no step-down yet.
    ts0 = ts
    for _ in range(5):
        ts += 100.0
        k_after, n_after, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                                         stepdown_floor=(8, 12))
        # State should hold — k and n shouldn't be changing.
        k, n = k_after, n_after
    # The lossy tick walked the ladder up first, so we're not at (2,8) anymore.
    # The point is that no FEC step-DOWN happened during the post-loss window.
    # After loss step-up, k goes from (2,8) → (1,4) refused → holds at (2,8).
    # So we should still be at (2, 8) here, not (2, 6).
    assert (k, n) == (2, 8)


def test_fec_stepdown_blocked_by_fec_work_above_threshold():
    """fec_work=0.04 > idle_threshold=0.02 → counter never increments,
    no step-down."""
    cfg = PolicyConfig(clean_windows_for_fec_stepdown=10,
                       fec_work_idle_threshold=0.02)
    cfg.cooldown.min_change_interval_ms_fec = 0
    tl = TrailingLoop(cfg)
    k, n, ts = 2, 8, 0.0
    for _ in range(20):
        ts += 100.0
        k, n, _, _ = tl.tick(
            _sigs(residual_loss_w=0.0, fec_work=0.04, ts=ts / 1000.0),
            k, n, 1, ts, stepdown_floor=(8, 12),
        )
    assert (k, n) == (2, 8)


def test_fec_stepdown_stops_at_floor():
    """Once at stepdown_floor, no further step-downs fire."""
    cfg = PolicyConfig(clean_windows_for_fec_stepdown=10,
                       fec_work_idle_threshold=0.02)
    cfg.cooldown.min_change_interval_ms_fec = 0
    tl = TrailingLoop(cfg)
    k, n, ts = 8, 14, 0.0
    # First 10 clean → step (8, 14) → (8, 12) (the floor).
    for _ in range(10):
        ts += 100.0
        k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                             stepdown_floor=(8, 12))
    assert (k, n) == (8, 12)
    # 30 more clean — should hold at (8, 12).
    for _ in range(30):
        ts += 100.0
        k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                             stepdown_floor=(8, 12))
    assert (k, n) == (8, 12)


def test_fec_stepdown_walks_full_ladder_to_floor():
    """Continuous clean+idle from (2, 8) lands at (8, 12) in 11 steps,
    each separated by ~10 ticks of clean window."""
    cfg = PolicyConfig(clean_windows_for_fec_stepdown=10,
                       fec_work_idle_threshold=0.02)
    cfg.cooldown.min_change_interval_ms_fec = 0
    tl = TrailingLoop(cfg)
    k, n, ts = 2, 8, 0.0
    # 11 step-downs * 10 windows = 110 ticks
    for _ in range(120):
        ts += 100.0
        k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                             stepdown_floor=(8, 12))
    assert (k, n) == (8, 12)


def test_fec_stepdown_respects_cooldown():
    """Even with the clean-window threshold met, the FEC cooldown
    blocks back-to-back steps."""
    cfg = PolicyConfig(clean_windows_for_fec_stepdown=10,
                       fec_work_idle_threshold=0.02)
    cfg.cooldown.min_change_interval_ms_fec = 500.0
    tl = TrailingLoop(cfg)
    k, n, ts = 2, 8, 0.0
    # 10 windows clean → first step-down at ts=1000.
    for _ in range(10):
        ts += 100.0
        k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                             stepdown_floor=(8, 12))
    assert (k, n) == (2, 6)
    first_step_ts = ts
    # 10 more clean windows immediately after — would normally step,
    # but the 500 ms cooldown is still active for the first ~5 ticks.
    # Continue feeding clean until cooldown expires AND counter
    # rebuilds to 10 again.
    for _ in range(20):
        ts += 100.0
        k_new, n_new, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                                     stepdown_floor=(8, 12))
        if (k_new, n_new) != (k, n):
            assert ts - first_step_ts >= 500.0
            assert (k_new, n_new) == (2, 4)
            return
        k, n = k_new, n_new
    assert False, "Step-down never fired after cooldown"


def test_fec_stepdown_no_floor_passed_no_step():
    """stepdown_floor=None → step-down never fires (back-compat)."""
    cfg = PolicyConfig(clean_windows_for_fec_stepdown=10,
                       fec_work_idle_threshold=0.02)
    cfg.cooldown.min_change_interval_ms_fec = 0
    tl = TrailingLoop(cfg)
    k, n, ts = 2, 8, 0.0
    for _ in range(50):
        ts += 100.0
        k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                             stepdown_floor=None)
    assert (k, n) == (2, 8)


def test_fec_stepdown_does_not_fire_during_step_up_tick():
    """A tick that triggers preemptive step-up shouldn't also step
    down — the fec_work signal that drives step-up is above the idle
    threshold so the step-down counter is reset."""
    cfg = PolicyConfig(clean_windows_for_fec_stepdown=10,
                       fec_work_idle_threshold=0.02)
    cfg.cooldown.min_change_interval_ms_fec = 0
    tl = TrailingLoop(cfg)
    k, n, ts = 6, 10, 0.0
    # Build up clean counter first.
    for _ in range(9):
        ts += 100.0
        k, n, _, _ = tl.tick(_idle_sigs(ts), k, n, 1, ts,
                             stepdown_floor=(8, 12))
    # Tick with fec_work=0.10 — preemptive step-up fires; step-down
    # counter resets to 0; no step-down.
    ts += 100.0
    k_after, n_after, _, _ = tl.tick(
        _sigs(residual_loss_w=0.0, fec_work=0.10, ts=ts / 1000.0),
        k, n, 1, ts, stepdown_floor=(8, 12),
    )
    # Ladder went UP, not down.
    assert (k_after, n_after) != (k, n)
    # Step-up of (6, 10) → (6, 12).
    assert (k_after, n_after) == (6, 12)


# Policy-level integration.

def _settled_clean_sigs(ts_ms: float, snr: float = 30.0) -> Signals:
    """A Signals that the dual-gate selector treats as 'fine, hold MCS'
    while leaving FEC idle."""
    return Signals(
        rssi=-55.0, rssi_min_w=-55.0, rssi_max_w=-55.0,
        snr=snr, snr_min_w=snr, snr_max_w=snr,
        residual_loss_w=0.0, fec_work=0.0,
        timestamp=ts_ms / 1000.0,
        link_starved_w=False,
        session=SessionInfo(
            fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=1,
            interleave_depth=1, contract_version=2,
        ),
    )


def _policy_at_mcs5() -> Policy:
    """Build a policy capped at gate.max_mcs=5 so _settle_at_max_mcs
    lands us at MCS 5 (preferred_k=6, band floor (6, 10))."""
    from dynamic_link.policy import GateConfig
    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    cfg = PolicyConfig(
        leading=LeadingLoopConfig(tx_power_min_dBm=5.0, tx_power_max_dBm=30.0),
        gate=GateConfig(max_mcs=5),
        safe=SafeDefaults(k=8, n=12, depth=1, mcs=1, bitrate_kbps=2000),
        clean_windows_for_fec_stepdown=10,
        fec_work_idle_threshold=0.02,
    )
    cfg.cooldown.min_change_interval_ms_fec = 0
    return Policy(cfg, prof)


def test_policy_passes_correct_stepdown_floor_for_current_mcs():
    """At MCS 5 (preferred_k=6), step-down should walk from (2, 8)
    toward (6, 10) — the band-6 floor."""
    p = _policy_at_mcs5()
    ts = _settle_at_max_mcs(p)
    assert p.state.mcs == 5
    # Force (k, n) to (2, 8). This bypasses the rebase that would
    # have happened on MCS climb.
    p.state.k, p.state.n = 2, 8
    # Feed enough clean ticks to walk all 8 step-downs from (2, 8) to (6, 10):
    # (2, 8) → (2, 6) → (2, 4) → (4, 12) → (4, 10) → (4, 8) → (6, 14) → (6, 12) → (6, 10)
    # = 8 step-downs. Need 8 * 10 = 80 windows.
    for _ in range(120):
        ts += 100.0
        p.tick(_settled_clean_sigs(ts))
    assert (p.state.k, p.state.n) == (6, 10)


def test_policy_stepdown_holds_at_per_mcs_floor():
    """Once recovery walks (k, n) to the per-MCS floor, further clean
    windows at the same MCS don't change (k, n)."""
    p = _policy_at_mcs5()
    ts = _settle_at_max_mcs(p)
    assert p.state.mcs == 5
    # Force (k, n) to (6, 10) — already at the per-MCS floor for MCS 5.
    p.state.k, p.state.n = 6, 10
    for _ in range(100):
        ts += 100.0
        p.tick(_settled_clean_sigs(ts))
    assert (p.state.k, p.state.n) == (6, 10)
