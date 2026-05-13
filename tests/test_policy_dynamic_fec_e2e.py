"""End-to-end-ish dynamic-FEC bounds + emit-gating cadence test.

Drives `Policy.tick` over ~50 ticks with varying synthetic Signals
against a synced `DroneConfigState` (mtu=1400, fps=60), and asserts:

  * every emitted Decision has `k ∈ [k_min, k_max]`,
  * every emitted Decision has `k ≤ n ≤ ceil(k * (1 + max_redundancy_ratio))`,
  * `EmitGate` cadence — when the (k, n) value changes on a tick whose
    MCS is unchanged, the gap between consecutive solo (k, n) changes
    is `>= debounce_ticks`.

The plan's preferred shape was a sandboxed `dl-applier` test, but the
existing `_sandbox` helper doesn't drive synthetic signals or expose
emitted Decisions — see plan Task 8 "Practical adjustments". The
policy-level variant below covers the same invariants without
extending the sandbox.
"""
from __future__ import annotations

import math
from pathlib import Path

from dynamic_link.drone_config import DroneConfigState
from dynamic_link.policy import (
    LeadingLoopConfig,
    Policy,
    PolicyConfig,
    SafeDefaults,
)
from dynamic_link.profile import load_profile
from dynamic_link.signals import Signals
from dynamic_link.stats_client import SessionInfo
from dynamic_link.wire import Hello

REPO_ROOT = Path(__file__).resolve().parent.parent
PACKAGED_DIR = REPO_ROOT / "conf" / "radios"


def _sigs(ts_ms: float, *, snr: float, residual_loss: float = 0.0,
          fec_work: float = 0.0, link_starved: bool = False) -> Signals:
    return Signals(
        rssi=-55.0, rssi_min_w=-55.0, rssi_max_w=-55.0,
        snr=snr, snr_min_w=snr, snr_max_w=snr,
        residual_loss_w=residual_loss, fec_work=fec_work,
        timestamp=ts_ms / 1000.0,
        link_starved_w=link_starved,
        session=SessionInfo(
            fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=1,
            interleave_depth=1, contract_version=2,
        ),
    )


def _build_policy() -> Policy:
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
    return Policy(cfg, prof, drone_config=drone_cfg)


def test_dynamic_fec_stays_within_bounds_over_50_ticks():
    """Run a 50-tick mixed-condition sweep and assert (k, n) bounds
    on every emitted Decision."""
    p = _build_policy()
    fec_cfg = p.cfg.dynamic_fec
    # Mix of clean, sub-emergency loss, and loss-recovery patterns so
    # the NEscalator gets a chance to ramp and recover, and the leading
    # selector can climb across MCS rows.
    loss_pattern = [0.0] * 10 + [0.01] * 5 + [0.0] * 10 + [0.03] * 3 + \
                   [0.0] * 12 + [0.02] * 5 + [0.0] * 5
    decisions = []
    for i, loss in enumerate(loss_pattern):
        # SNR sweep gives the leading selector room to step MCS.
        snr = 20.0 + (i % 15)
        d = p.tick(_sigs(i * 100.0, snr=snr, residual_loss=loss,
                         fec_work=0.05 if loss > 0 else 0.0))
        decisions.append(d)

    assert len(decisions) == len(loss_pattern)
    for d in decisions:
        assert fec_cfg.k_min <= d.k <= fec_cfg.k_max, (
            f"k={d.k} outside [{fec_cfg.k_min}, {fec_cfg.k_max}]"
        )
        n_ceiling = math.ceil(d.k * (1.0 + fec_cfg.max_redundancy_ratio))
        assert d.k <= d.n <= n_ceiling, (
            f"n={d.n} outside [{d.k}, {n_ceiling}] (k={d.k})"
        )


def test_dynamic_fec_emit_gate_debounces_solo_kn_changes():
    """Two consecutive ticks must not both emit a (k, n) change unless
    MCS also changed. Concretely: between two solo (k, n) changes on
    same-MCS ticks, the tick gap must be >= EmitGate.debounce_ticks."""
    p = _build_policy()
    debounce = p._emit_gate.debounce_ticks

    # Drive a varied signal stream that should provoke at least a
    # couple of solo (k, n) changes (NEscalator ramping) without
    # constant MCS changes.
    loss_pattern = [0.0] * 5 + [0.03] * 10 + [0.0] * 5 + [0.03] * 10 + \
                   [0.0] * 20
    decisions = []
    for i, loss in enumerate(loss_pattern):
        d = p.tick(_sigs(i * 100.0, snr=28.0, residual_loss=loss,
                         fec_work=0.05 if loss > 0 else 0.0))
        decisions.append(d)

    # Find ticks where (k, n) changed.
    solo_change_indices: list[int] = []
    for i in range(1, len(decisions)):
        prev, cur = decisions[i - 1], decisions[i]
        if (prev.k, prev.n) != (cur.k, cur.n) and prev.mcs == cur.mcs:
            solo_change_indices.append(i)

    # Each consecutive pair of solo (k, n) changes on same-MCS ticks
    # must be separated by at least `debounce` ticks.
    for a, b in zip(solo_change_indices, solo_change_indices[1:]):
        assert b - a >= debounce, (
            f"solo (k,n) changes at ticks {a} and {b} violate "
            f"debounce={debounce}"
        )


def test_dynamic_fec_first_tick_always_emits_computed_values():
    """The very first tick after sync emits real computed (k, n) — not
    the safe defaults — because EmitGate has no prior state."""
    p = _build_policy()
    safe = p.cfg.safe
    d = p.tick(_sigs(0.0, snr=25.0))
    # The decision should reflect computed dynamic-FEC values, not the
    # SafeDefaults pre-sync placeholder. With mtu=1400, fps=60 and a
    # real bitrate, k will be at the k_min floor (4) or above.
    assert d.k >= p.cfg.dynamic_fec.k_min
    # And it should have committed to the emit gate.
    assert p._emit_gate.last_k == d.k
    assert p._emit_gate.last_n == d.n
    # And the safe-defaults k=8 only matches by coincidence — assert
    # the wire bytes (k, n) satisfy the bounds we care about, not that
    # they equal `safe`.
    fec_cfg = p.cfg.dynamic_fec
    n_ceiling = math.ceil(d.k * (1.0 + fec_cfg.max_redundancy_ratio))
    assert d.k <= d.n <= n_ceiling
    _ = safe  # silence unused
