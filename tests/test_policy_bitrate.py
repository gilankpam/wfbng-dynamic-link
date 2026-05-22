"""Bitrate per MCS row — sanity check of the new pipeline."""
from __future__ import annotations

from pathlib import Path

import pytest

from dynamic_link.bitrate import BitrateConfig
from dynamic_link.profile import load_profile

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


@pytest.fixture
def profile():
    return load_profile("m8812eu2", [PACKAGED_DIR])


def _cfg() -> BitrateConfig:
    return BitrateConfig(
        utilization_factor=0.8,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )


def test_wire_target_higher_mcs_has_higher_wire_target(profile):
    from dynamic_link.bitrate import compute_wire_target_kbps
    a = compute_wire_target_kbps(profile, 20, 1, 1400, 0.8)
    b = compute_wire_target_kbps(profile, 20, 5, 1400, 0.8)
    assert b > a


def test_wire_target_grows_with_mtu(profile):
    from dynamic_link.bitrate import compute_wire_target_kbps
    a = compute_wire_target_kbps(profile, 20, 4, 1500, 0.8)
    b = compute_wire_target_kbps(profile, 20, 4, 3994, 0.8)
    assert b > a


def test_policy_wire_rate_under_target_across_escalation(profile):
    """Parametric: for every (mcs, escalation) combo at MCS 0–5,
    bitrate × n / k ≤ wire_target.

    This is THE death-spiral regression — it currently fails on
    master and locks in the fix in this commit.
    """
    from dynamic_link.bitrate import (
        BitrateConfig, compute_bitrate_kbps, compute_wire_target_kbps,
    )
    from dynamic_link.dynamic_fec import (
        DynamicFecConfig, clamp_n_for_bitrate_floor, compute_k, compute_n,
    )

    fec_cfg = DynamicFecConfig(
        k_min=2, k_max=20,
        base_redundancy_ratio=0.4, max_redundancy_ratio=1.0,
        n_loss_threshold=0.015, n_loss_windows=3, n_loss_step=1,
        n_recover_windows=10, n_recover_step=1, max_n_escalation=6,
    )
    util = 0.6
    mtu = 1500
    fps = 60
    min_br = 1000
    max_br = 24000

    for mcs in range(profile.mcs_min, profile.mcs_max + 1):
        wire_target = compute_wire_target_kbps(profile, 20, mcs, mtu, util)
        k = compute_k(wire_target_kbps=wire_target, mtu_bytes=mtu, fps=fps, cfg=fec_cfg)
        for escalation in range(0, fec_cfg.max_n_escalation + 1):
            n_unclamped = compute_n(k=k, n_escalation=escalation, cfg=fec_cfg)
            n = clamp_n_for_bitrate_floor(n_unclamped, k, wire_target, min_br)
            bitrate = compute_bitrate_kbps(
                wire_target_kbps=wire_target, k=k, n=n,
                min_bitrate_kbps=min_br, max_bitrate_kbps=max_br,
            )
            wire_actual = bitrate * n / k
            assert wire_actual <= wire_target + 1, (
                f"mcs={mcs} esc={escalation}: "
                f"wire_actual={wire_actual:.0f} > target={wire_target:.0f}"
            )
            assert bitrate >= min_br, (
                f"mcs={mcs} esc={escalation}: "
                f"bitrate={bitrate} < floor={min_br}"
            )


def test_policy_bitrate_shrinks_with_n_escalation(profile):
    """Under sustained loss at MCS 0, policy.tick emits a bitrate that
    decreases as NEscalator ramps escalation up."""
    from dynamic_link.bitrate import compute_wire_target_kbps
    from dynamic_link.drone_config import DroneConfigState
    from dynamic_link.policy import (
        LeadingLoopConfig, Policy, PolicyConfig, SafeDefaults,
    )
    from dynamic_link.signals import Signals
    from dynamic_link.wire import Hello

    from dynamic_link.policy import GateConfig
    cfg = PolicyConfig(
        leading=LeadingLoopConfig(
            tx_power_min_dBm=5.0, tx_power_max_dBm=30.0,
        ),
        gate=GateConfig(max_mcs=5),
        safe=SafeDefaults(k=2, n=3, depth=1, mcs=0),
    )
    drone_cfg = DroneConfigState()
    drone_cfg.on_hello(Hello(
        generation_id=1, mtu_bytes=1500, fps=60, applier_build_sha=0,
    ))
    p = Policy(cfg, profile, drone_config=drone_cfg)

    # Pin MCS at 0 by feeding a stress trace that emergency-forces MCS down.
    bitrates_at_mcs0 = []
    for tick_i in range(40):
        sigs = Signals(
            timestamp=tick_i * 0.1,
            rssi=-80.0, rssi_min_w=-82.0, rssi_max_w=-78.0,
            snr=4.0, snr_slope=0.0,
            residual_loss_w=0.05, fec_work=0.20,
            link_starved_w=False,
        )
        d = p.tick(sigs)
        # Wire-safety: every emitted Decision keeps wire ≤ target.
        wire_target = compute_wire_target_kbps(
            profile, cfg.leading.bandwidth, d.mcs, 1500,
            cfg.bitrate.utilization_factor,
        )
        wire_actual = d.bitrate_kbps * d.n / d.k
        assert wire_actual <= wire_target + 1, (
            f"tick={tick_i} mcs={d.mcs}: "
            f"wire_actual={wire_actual:.0f} > target={wire_target:.0f} "
            f"(bitrate={d.bitrate_kbps} k={d.k} n={d.n})"
        )
        if d.mcs == 0:
            bitrates_at_mcs0.append(d.bitrate_kbps)

    # Need at least a handful of MCS-0 ticks where escalation could ramp.
    assert len(bitrates_at_mcs0) >= 15, (
        f"too few MCS-0 ticks: {len(bitrates_at_mcs0)}"
    )
    # Bitrate must not be strictly constant under sustained loss —
    # it should shrink at least once during the trace.
    assert min(bitrates_at_mcs0) < bitrates_at_mcs0[0], (
        f"bitrate did not shrink: first={bitrates_at_mcs0[0]} "
        f"min={min(bitrates_at_mcs0)}"
    )
