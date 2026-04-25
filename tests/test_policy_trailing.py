"""Tests for the trailing loop: loss-driven escalation (§4.2)."""
from __future__ import annotations

from dynamic_link.policy import PolicyConfig, TrailingLoop
from dynamic_link.predictor import LADDER_STEPS
from dynamic_link.signals import Signals
from dynamic_link.stats_client import SessionInfo


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
