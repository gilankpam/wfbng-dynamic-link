"""Tests for the trailing loop and Policy-level FEC computation.

The controller computes (k, n_lower, n_upper) deterministically from
MCS conditions; the trailing loop only moves `n` within [n_lower,
n_upper] under signal-driven adjustments.
"""
from __future__ import annotations

from pathlib import Path

from dynamic_link.policy import (
    FECBounds,
    LeadingLoopConfig,
    Policy,
    PolicyConfig,
    SafeDefaults,
    TrailingLoop,
    compute_k,
    compute_n_bounds,
    _ipi_ms_for_encoder,
)
from dynamic_link.predictor import PredictorConfig
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


def _idle_sigs(ts_ms: float) -> Signals:
    return _sigs(residual_loss_w=0.0, fec_work=0.0, ts=ts_ms / 1000.0)


# ── compute_k ───────────────────────────────────────────────────────────────

def test_compute_k_fast_ipi_clamps_at_k_max():
    """High encoder rate → small ipi → ceil(target/ipi) ≫ k_max → clamp."""
    # ipi=0.43, target=12 → raw=27.9 → ceil=28 → clamped to k_max=12.
    assert compute_k(ipi_ms=0.43, target_fill_ms=12.0, k_min=2, k_max=12) == 12


def test_compute_k_slow_ipi_picks_k_min():
    """Very slow encoder → ipi_ms ≫ target → ceil(small) → clamped to k_min."""
    # ipi=10 ms, target=12 ms → raw=1.2 → ceil=2 → k=2.
    assert compute_k(ipi_ms=10.0, target_fill_ms=12.0, k_min=2, k_max=12) == 2


def test_compute_k_mid_ipi_uses_ceil():
    """Mid IPI → exact integer k from ceil()."""
    # ipi=2.5 ms, target=12 ms → raw = 4.8 → ceil = 5.
    assert compute_k(ipi_ms=2.5, target_fill_ms=12.0, k_min=2, k_max=12) == 5
    # ipi=2.15 ms (≈ MCS1 m8812eu2) → raw=5.58 → ceil=6.
    assert compute_k(ipi_ms=2.15, target_fill_ms=12.0, k_min=2, k_max=12) == 6
    # ipi=4.31 ms (≈ MCS0 m8812eu2) → raw=2.78 → ceil=3.
    assert compute_k(ipi_ms=4.31, target_fill_ms=12.0, k_min=2, k_max=12) == 3


# ── compute_n_bounds ────────────────────────────────────────────────────────

def test_n_upper_clamped_by_bandwidth():
    """With redundancy cap disabled, bandwidth alone bounds n_upper:
    encoder=10 Mbps in a 25 Mbps link → n/k ≤ 2.5 → max redundancy
    150% → for k=8, n_upper = round(8 * 2.5) = 20."""
    bounds = FECBounds(max_redundancy_ratio=None)
    p_cfg = PredictorConfig(per_packet_airtime_us=80.0,
                            inter_packet_interval_ms=1.4)
    n_lower, n_upper = compute_n_bounds(
        k=8, encoder_kbps=10_000, bw_Mbps=25.0, bounds=bounds,
        max_latency_ms=1000.0, depth=1, predictor_cfg=p_cfg,
    )
    assert n_upper == 20


def test_n_upper_clamped_by_redundancy_cap():
    """When the redundancy cap bites first, n_upper = round(k*(1+cap))."""
    bounds = FECBounds(max_redundancy_ratio=1.0)  # 100% cap
    p_cfg = PredictorConfig(per_packet_airtime_us=80.0,
                            inter_packet_interval_ms=1.4)
    # encoder=5 Mbps, bw=25 Mbps → bw allows redundancy=4.0
    # cap=1.0 wins → n_upper = round(8 * 2.0) = 16, but for k=4: round(4*2)=8.
    n_lower, n_upper = compute_n_bounds(
        k=4, encoder_kbps=5_000, bw_Mbps=25.0, bounds=bounds,
        max_latency_ms=1000.0, depth=1, predictor_cfg=p_cfg,
    )
    assert n_upper == 8


def test_n_upper_clamped_by_latency():
    """With short latency cap, latency wins over bandwidth + cap."""
    bounds = FECBounds(max_redundancy_ratio=None)
    p_cfg = PredictorConfig(per_packet_airtime_us=80.0,
                            inter_packet_interval_ms=1.4)
    # cap=10 ms, k=8, depth=3, ipi=1.4 → block_fill=11.2 ms already overrun.
    n_lower, n_upper = compute_n_bounds(
        k=8, encoder_kbps=10_000, bw_Mbps=25.0, bounds=bounds,
        max_latency_ms=10.0, depth=3, predictor_cfg=p_cfg,
    )
    # n_upper would be 0 from latency, but compute_n_bounds floors at k.
    assert n_upper == 8  # the k floor — no parity fits


def test_n_lower_uses_base_redundancy():
    """n_lower = round(k * (1 + base_redundancy_ratio)), clamped to n_min."""
    bounds = FECBounds(base_redundancy_ratio=0.5, n_min=4)
    p_cfg = PredictorConfig(per_packet_airtime_us=80.0,
                            inter_packet_interval_ms=1.4)
    # k=8 → round(8 * 1.5) = 12.
    n_lower, n_upper = compute_n_bounds(
        k=8, encoder_kbps=10_000, bw_Mbps=25.0, bounds=bounds,
        max_latency_ms=1000.0, depth=1, predictor_cfg=p_cfg,
    )
    assert n_lower == 12


def test_n_lower_clamped_by_n_upper_under_latency():
    """If latency squeezes n_upper below the base-redundancy floor,
    n_lower is pulled down to match (latency wins over a clean-link
    floor)."""
    bounds = FECBounds(base_redundancy_ratio=0.5)
    p_cfg = PredictorConfig(per_packet_airtime_us=80.0,
                            inter_packet_interval_ms=1.4)
    n_lower, n_upper = compute_n_bounds(
        k=8, encoder_kbps=10_000, bw_Mbps=25.0, bounds=bounds,
        max_latency_ms=10.0, depth=3, predictor_cfg=p_cfg,
    )
    assert n_lower <= n_upper


# ── TrailingLoop signal-driven n adjustment ─────────────────────────────────

def _trailing(**fec_overrides) -> tuple[TrailingLoop, PolicyConfig]:
    cfg = PolicyConfig(fec=FECBounds(**fec_overrides))
    cfg.cooldown.min_change_interval_ms_fec = 200.0
    return TrailingLoop(cfg), cfg


def test_residual_loss_bumps_n_by_loss_step_immediately():
    """Loss bypasses cooldown; n grows by n_loss_step. IDR requested."""
    tl, _ = _trailing(n_loss_step=2)
    n, depth, idr = tl.tick(
        _sigs(residual_loss_w=0.05, ts=0.0),
        current_n=12, current_depth=1, ts_ms=0.0,
        n_lower=12, n_upper=16,
    )
    assert n == 14
    assert idr is True


def test_residual_loss_clamped_at_n_upper():
    """Bump can't exceed n_upper."""
    tl, _ = _trailing(n_loss_step=4)
    n, _, _ = tl.tick(
        _sigs(residual_loss_w=0.05, ts=0.0),
        current_n=14, current_depth=1, ts_ms=0.0,
        n_lower=12, n_upper=15,
    )
    assert n == 15


def test_fec_work_preemptive_step_after_cooldown():
    """fec_work above 0.05 → bump by n_preempt_step once FEC cooldown
    has elapsed. No IDR."""
    tl, _ = _trailing(n_preempt_step=1)
    # First tick at t=0 is inside the 200 ms initial cooldown window.
    n, _, idr = tl.tick(
        _sigs(fec_work=0.20, ts=0.0),
        current_n=12, current_depth=1, ts_ms=0.0,
        n_lower=12, n_upper=16,
    )
    assert n == 12
    assert idr is False
    # After cooldown elapses, preempt fires.
    n, _, idr = tl.tick(
        _sigs(fec_work=0.20, ts=0.3),
        current_n=12, current_depth=1, ts_ms=300.0,
        n_lower=12, n_upper=16,
    )
    assert n == 13
    assert idr is False


def test_no_change_when_clean():
    tl, _ = _trailing()
    n, depth, idr = tl.tick(
        _idle_sigs(0.0),
        current_n=12, current_depth=1, ts_ms=0.0,
        n_lower=12, n_upper=16,
    )
    assert (n, depth, idr) == (12, 1, False)


def test_n_walks_down_on_sustained_clean():
    """After clean_windows_for_fec_stepdown clean+idle windows, n drops
    by n_recover_step."""
    tl, cfg = _trailing(n_recover_step=1)
    cfg.clean_windows_for_fec_stepdown = 5
    cfg.fec_work_idle_threshold = 0.02
    cfg.cooldown.min_change_interval_ms_fec = 0
    n, ts = 16, 0.0
    for _ in range(4):
        ts += 100.0
        n, _, _ = tl.tick(_idle_sigs(ts), current_n=n, current_depth=1,
                          ts_ms=ts, n_lower=12, n_upper=16)
    assert n == 16  # not enough clean windows yet
    ts += 100.0
    n, _, _ = tl.tick(_idle_sigs(ts), current_n=n, current_depth=1,
                      ts_ms=ts, n_lower=12, n_upper=16)
    assert n == 15


def test_n_stepdown_stops_at_n_lower():
    """n step-down doesn't go below the floor."""
    tl, cfg = _trailing(n_recover_step=1)
    cfg.clean_windows_for_fec_stepdown = 5
    cfg.fec_work_idle_threshold = 0.02
    cfg.cooldown.min_change_interval_ms_fec = 0
    n, ts = 12, 0.0
    for _ in range(20):
        ts += 100.0
        n, _, _ = tl.tick(_idle_sigs(ts), current_n=n, current_depth=1,
                          ts_ms=ts, n_lower=12, n_upper=16)
    assert n == 12


def test_n_clamped_to_envelope_each_tick():
    """If current_n is below n_lower entering the tick, it gets pulled
    up to the floor regardless of signals."""
    tl, _ = _trailing()
    n, _, _ = tl.tick(
        _idle_sigs(0.0),
        current_n=8, current_depth=1, ts_ms=0.0,
        n_lower=12, n_upper=16,
    )
    assert n == 12


def test_n_clamped_down_when_n_upper_drops():
    """If current_n is above n_upper (e.g., bw shrank after MCS drop),
    it gets pulled down."""
    tl, _ = _trailing()
    n, _, _ = tl.tick(
        _idle_sigs(0.0),
        current_n=20, current_depth=1, ts_ms=0.0,
        n_lower=12, n_upper=16,
    )
    assert n == 16


def test_loss_step_blocked_when_already_at_n_upper():
    """At n_upper, residual_loss can't push higher (clamp wins)."""
    tl, _ = _trailing(n_loss_step=2)
    n, _, idr = tl.tick(
        _sigs(residual_loss_w=0.05, ts=0.0),
        current_n=16, current_depth=1, ts_ms=0.0,
        n_lower=12, n_upper=16,
    )
    assert n == 16
    # IDR still requested — the controller wanted to escalate but couldn't.
    assert idr is True


# ── Depth handling (unchanged logic, new signature) ─────────────────────────

def _tick_depth(tl, signals, current_depth, ts_ms,
                current_n=12, n_lower=12, n_upper=16):
    return tl.tick(signals, current_n=current_n, current_depth=current_depth,
                   ts_ms=ts_ms, n_lower=n_lower, n_upper=n_upper)


def test_depth_raised_when_burst_and_holdoff_together():
    tl, _ = _trailing()
    _, depth, _ = _tick_depth(
        tl, _sigs(residual_loss_w=0.02, burst_rate=5.0,
                  holdoff_rate=2.0, ts=0.3),
        current_depth=1, ts_ms=300.0,
    )
    assert depth == 2


def test_depth_not_raised_on_burst_alone():
    tl, _ = _trailing()
    _, depth, _ = _tick_depth(
        tl, _sigs(residual_loss_w=0.02, burst_rate=5.0,
                  holdoff_rate=0.0, ts=0.3),
        current_depth=1, ts_ms=300.0,
    )
    assert depth == 1


def test_depth_bootstrap_fires_on_sustained_loss_plus_busy_fec():
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    depth = 1
    for i in range(3):
        _, depth, _ = _tick_depth(
            tl, _sigs(residual_loss_w=0.05, fec_work=0.20,
                      ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
            n_lower=12, n_upper=16,
        )
    assert depth == 2


def test_depth_bootstrap_requires_busy_fec_not_just_sustained_loss():
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    depth = 1
    for i in range(3):
        _, depth, _ = _tick_depth(
            tl, _sigs(residual_loss_w=0.05, fec_work=0.05,
                      ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 1


def test_depth_steps_down_after_sustained_clean_link():
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=5)
    tl = TrailingLoop(cfg)
    depth = 3
    for i in range(5):
        _, depth, _ = _tick_depth(
            tl, _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 2


def test_depth_walks_down_one_step_at_a_time():
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=5)
    tl = TrailingLoop(cfg)
    depth = 3
    for i in range(5):
        _, depth, _ = _tick_depth(
            tl, _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 2
    for i in range(5):
        _, depth, _ = _tick_depth(
            tl, _sigs(residual_loss_w=0.0, ts=(i + 6) * 0.3),
            current_depth=depth, ts_ms=(i + 6) * 300.0,
        )
    assert depth == 1


def test_depth_does_not_step_down_with_recent_loss():
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=5)
    tl = TrailingLoop(cfg)
    depth = 3
    for i in range(4):
        _, depth, _ = _tick_depth(
            tl, _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    _, depth, _ = _tick_depth(
        tl, _sigs(residual_loss_w=0.05, ts=1.5),
        current_depth=depth, ts_ms=1500.0,
    )
    for i in range(3):
        _, depth, _ = _tick_depth(
            tl, _sigs(residual_loss_w=0.0, ts=1.8 + i * 0.3),
            current_depth=depth, ts_ms=1800.0 + i * 300.0,
        )
    assert depth == 3


def test_depth_bootstrap_is_one_shot_at_depth_1():
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    depth = 2
    for i in range(3):
        _, depth, _ = _tick_depth(
            tl, _sigs(residual_loss_w=0.05, fec_work=0.20,
                      ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 2


def test_sustained_loss_detection():
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    for t in range(3):
        _tick_depth(tl, _sigs(residual_loss_w=0.01, ts=t * 0.1),
                    current_depth=1, ts_ms=t * 100.0)
    assert tl.sustained_loss() is True


# ── Policy-level integration ────────────────────────────────────────────────

def _policy(**overrides) -> Policy:
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
    ts = base_ts
    for _ in range(200):
        if p.state.mcs >= p.cfg.gate.max_mcs:
            break
        p.tick(_starved_sigs(ts, link_starved=False))
        ts += 100.0
    return ts


def test_starvation_needs_n_consecutive_windows():
    p = _policy(starvation_windows=5)
    ts = _settle_at_max_mcs(p)
    start_mcs = p.state.mcs
    for _ in range(4):
        d = p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
    assert d.mcs == start_mcs
    d = p.tick(_starved_sigs(ts, link_starved=True))
    assert d.mcs < start_mcs
    assert "emergency" in d.reason and "starved" in d.reason


def test_starvation_resets_on_recovery():
    p = _policy(starvation_windows=5)
    ts = _settle_at_max_mcs(p)
    start_mcs = p.state.mcs
    for _ in range(3):
        p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
    p.tick(_starved_sigs(ts, link_starved=False))
    ts += 100.0
    for _ in range(4):
        d = p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
    assert d.mcs == start_mcs


def test_starvation_pins_tx_power_to_max():
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
    p = _policy(starvation_windows=5)
    ts = 10_000.0
    for _ in range(50):
        p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
    for _ in range(50):
        d = p.tick(_starved_sigs(ts, link_starved=True))
        ts += 100.0
        assert d.mcs == 0


def _settled_clean_sigs(ts_ms: float, snr: float = 30.0) -> Signals:
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


def test_policy_settles_to_n_lower_on_clean_link():
    """After settling, (k, n) should land at the floor for the chosen MCS."""
    p = _policy()
    p.cfg.cooldown.min_change_interval_ms_fec = 0
    p.cfg.clean_windows_for_fec_stepdown = 5
    ts = _settle_at_max_mcs(p)
    # Feed clean signals long enough for n to walk down to the floor.
    for _ in range(60):
        ts += 100.0
        p.tick(_settled_clean_sigs(ts))
    # k computed from MCS via compute_k; any integer in [k_min, k_max].
    assert p.cfg.fec.k_min <= p.state.k <= p.cfg.fec.k_max
    assert p.state.n >= p.cfg.fec.n_min
    # On a clean link, n should be at or near the redundancy floor —
    # round(k * (1 + base_redundancy_ratio)).
    expected_floor = round(p.state.k * (1 + p.cfg.fec.base_redundancy_ratio))
    assert p.state.n <= expected_floor + 1


def test_policy_default_uses_packaged_profile_without_preferred_k():
    """The packaged radio profile no longer has preferred_k; loader
    must accept that."""
    p = _policy()
    assert p.profile.encoder_bitrate_frac == 0.40
    # No AttributeError on profile.preferred_k — it's gone.
    assert not hasattr(p.profile, "preferred_k")
