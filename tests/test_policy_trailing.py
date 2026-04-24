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


def test_depth_not_raised_on_burst_alone():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    _, _, depth, _ = tl.tick(
        _sigs(residual_loss_w=0.02, burst_rate=5.0, holdoff_rate=0.0, ts=0.3),
        current_k=8, current_n=12, current_depth=1, ts_ms=300.0,
    )
    assert depth == 1
