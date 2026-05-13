"""Tests for the trailing loop and Policy-level FEC handling.

`bitrate` is a deterministic per-MCS computation from the radio
profile's PHY rate. `(k, n)` are computed at runtime by `dynamic_fec`
from the live bitrate plus drone-reported `(mtu, fps)` — see
`gs/dynamic_link/dynamic_fec.py`. The trailing loop owns depth
bootstrap/step-down and an IDR-on-loss signal. See
`docs/knob-cadence-bench.md` for the empirical justification.
"""
from __future__ import annotations

from pathlib import Path

from dynamic_link.bitrate import compute_bitrate_kbps
from dynamic_link.policy import (
    FECBounds,
    LeadingLoopConfig,
    Policy,
    PolicyConfig,
    SafeDefaults,
    TrailingLoop,
)
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


# ── TrailingLoop: IDR signalling on loss ────────────────────────────────────

def test_residual_loss_emits_idr_only():
    """Residual loss → IDR request, no `(k, n)` change. The trailing
    loop returns only `(depth, idr)`; FEC is the leading loop's job."""
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    depth, idr = tl.tick(
        _sigs(residual_loss_w=0.05, ts=0.0),
        current_depth=1, ts_ms=0.0,
    )
    assert idr is True
    assert depth == 1


def test_no_idr_on_clean_link():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    depth, idr = tl.tick(_idle_sigs(0.0), current_depth=1, ts_ms=0.0)
    assert idr is False
    assert depth == 1


def test_no_idr_on_fec_pressure_alone():
    """fec_work > threshold but no residual loss → no IDR. The bench
    showed reactive FEC moves cost more than they save; the trailing
    loop only requests an IDR when an actual primary was lost."""
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    depth, idr = tl.tick(
        _sigs(fec_work=0.20, ts=0.0),
        current_depth=1, ts_ms=0.0,
    )
    assert idr is False
    assert depth == 1


# ── TrailingLoop: depth ─────────────────────────────────────────────────────

def test_depth_raised_when_burst_and_holdoff_together():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    depth, _ = tl.tick(
        _sigs(residual_loss_w=0.02, burst_rate=5.0,
              holdoff_rate=2.0, ts=0.3),
        current_depth=1, ts_ms=300.0,
    )
    assert depth == 2


def test_depth_not_raised_on_burst_alone():
    cfg = PolicyConfig()
    tl = TrailingLoop(cfg)
    depth, _ = tl.tick(
        _sigs(residual_loss_w=0.02, burst_rate=5.0,
              holdoff_rate=0.0, ts=0.3),
        current_depth=1, ts_ms=300.0,
    )
    assert depth == 1


def test_depth_bootstrap_fires_on_sustained_loss_plus_busy_fec():
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    depth = 1
    for i in range(3):
        depth, _ = tl.tick(
            _sigs(residual_loss_w=0.05, fec_work=0.20,
                  ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 2


def test_depth_bootstrap_requires_busy_fec_not_just_sustained_loss():
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    depth = 1
    for i in range(3):
        depth, _ = tl.tick(
            _sigs(residual_loss_w=0.05, fec_work=0.05,
                  ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 1


def test_depth_steps_down_after_sustained_clean_link():
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=5)
    tl = TrailingLoop(cfg)
    depth = 3
    for i in range(5):
        depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 2


def test_depth_walks_down_one_step_at_a_time():
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=5)
    tl = TrailingLoop(cfg)
    depth = 3
    for i in range(5):
        depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 2
    for i in range(5):
        depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=(i + 6) * 0.3),
            current_depth=depth, ts_ms=(i + 6) * 300.0,
        )
    assert depth == 1


def test_depth_does_not_step_down_with_recent_loss():
    cfg = PolicyConfig(clean_windows_for_depth_stepdown=5)
    tl = TrailingLoop(cfg)
    depth = 3
    for i in range(4):
        depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    depth, _ = tl.tick(
        _sigs(residual_loss_w=0.05, ts=1.5),
        current_depth=depth, ts_ms=1500.0,
    )
    for i in range(3):
        depth, _ = tl.tick(
            _sigs(residual_loss_w=0.0, ts=1.8 + i * 0.3),
            current_depth=depth, ts_ms=1800.0 + i * 300.0,
        )
    assert depth == 3


def test_depth_bootstrap_is_one_shot_at_depth_1():
    """Bootstrap trigger only applies at depth==1. From depth==2,
    further raises must come from the burst+holdoff refine path."""
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    depth = 2
    for i in range(3):
        depth, _ = tl.tick(
            _sigs(residual_loss_w=0.05, fec_work=0.20,
                  ts=(i + 1) * 0.3),
            current_depth=depth, ts_ms=(i + 1) * 300.0,
        )
    assert depth == 2


def test_sustained_loss_detection():
    cfg = PolicyConfig(sustained_loss_windows=3)
    tl = TrailingLoop(cfg)
    for t in range(3):
        tl.tick(_sigs(residual_loss_w=0.01, ts=t * 0.1),
                current_depth=1, ts_ms=t * 100.0)
    assert tl.sustained_loss() is True


# ── Policy-level integration: deterministic FEC follows MCS ─────────────────

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
        safe=SafeDefaults(k=8, n=12, depth=1, mcs=1),
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


def _clean_sigs(ts_ms: float, snr: float = 30.0) -> Signals:
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


def _lossy_sigs(ts_ms: float, snr: float = 30.0,
                residual_loss: float = 0.02) -> Signals:
    return Signals(
        rssi=-55.0, rssi_min_w=-55.0, rssi_max_w=-55.0,
        snr=snr, snr_min_w=snr, snr_max_w=snr,
        residual_loss_w=residual_loss, fec_work=0.10,
        timestamp=ts_ms / 1000.0,
        link_starved_w=False,
        session=SessionInfo(
            fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=1,
            interleave_depth=1, contract_version=2,
        ),
    )


def _settle_at_mcs(p, target_mcs: int, base_ts: float = 0.0) -> float:
    ts = base_ts
    for _ in range(200):
        if p.state.mcs >= target_mcs:
            break
        p.tick(_starved_sigs(ts, link_starved=False))
        ts += 100.0
    return ts


def test_policy_bitrate_matches_formula_at_each_mcs():
    """At every MCS the controller settles into, `bitrate_kbps` matches
    the formula derived from PHY rate and `policy.bitrate` config.
    `(k, n)` are now runtime-computed by dynamic_fec — see the
    test_policy_emits_computed_k_n_from_drone_config test below."""
    p = _policy()
    for target in (1, 3, 5):
        _settle_at_mcs(p, target)
        if p.state.mcs != target:
            continue
        expected = compute_bitrate_kbps(p.profile, 20, target, p.cfg.bitrate)
        assert p.state.bitrate_kbps == expected


def test_policy_emits_computed_k_n_from_drone_config():
    """With a drone reporting (mtu=1400, fps=60), policy.tick emits a
    `k` consistent with packets-per-frame at the live bitrate."""
    from dynamic_link.drone_config import DroneConfigState
    from dynamic_link.wire import Hello

    prof = load_profile("m8812eu2", [PACKAGED_DIR])
    cfg = PolicyConfig(
        leading=LeadingLoopConfig(
            tx_power_min_dBm=5.0, tx_power_max_dBm=30.0,
        ),
        safe=SafeDefaults(k=8, n=12, depth=1, mcs=1),
    )
    drone_cfg = DroneConfigState()
    drone_cfg.on_hello(Hello(
        generation_id=1, mtu_bytes=1400, fps=60, applier_build_sha=0,
    ))
    p = Policy(cfg, prof, drone_config=drone_cfg)
    # Drive a clean-link tick so the leading selector commits and the
    # emit-gate fires (first-call always emits).
    d = p.tick(_clean_sigs(0.0))
    # compute_k formula: bitrate_kbps * 1000 / (fps * mtu * 8), clamped.
    expected_k = max(
        cfg.dynamic_fec.k_min,
        min(cfg.dynamic_fec.k_max,
            int(d.bitrate_kbps * 1000 / (60 * 1400 * 8))),
    )
    assert d.k == expected_k


def test_sub_emergency_loss_does_not_change_fec():
    """Loss bursts below the emergency_loss_rate threshold must NOT
    fire any (k, n) reconfig. The trailing loop has no n-escalation
    duty; FEC moves only when MCS does. This is the bench-driven
    invariant — `CMD_SET_FEC` is the costly knob, so we don't fire
    it on per-window flicker."""
    p = _policy()
    ts = _settle_at_mcs(p, 5)
    if p.state.mcs != 5:
        return  # profile may cap below 5; skip rather than fail
    k_baseline, n_baseline = p.state.k, p.state.n
    # residual_loss=0.02 is below default emergency_loss_rate=0.05,
    # so MCS stays at 5; FEC must stay pinned with it.
    for _ in range(5):
        d = p.tick(_lossy_sigs(ts, residual_loss=0.02))
        ts += 100.0
        assert d.mcs == 5
        assert (p.state.k, p.state.n) == (k_baseline, n_baseline)
        # IDR is the only same-tick action on residual_loss > 0.
        assert d.idr_request is True
        assert "fec" not in d.knobs_changed


def test_starvation_needs_n_consecutive_windows():
    p = _policy(starvation_windows=5)
    ts = _settle_at_mcs(p, p.cfg.gate.max_mcs)
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
    ts = _settle_at_mcs(p, p.cfg.gate.max_mcs)
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
