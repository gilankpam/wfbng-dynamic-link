"""Unit tests for the dynamic FEC selection module."""
from __future__ import annotations

from dynamic_link.dynamic_fec import (
    DynamicFecConfig,
    EmitGate,
    NEscalator,
    compute_k,
    compute_n,
)


def _cfg(**over):
    base = dict(
        k_min=4, k_max=16,
        base_redundancy_ratio=0.5,
        max_redundancy_ratio=1.0,
        n_loss_threshold=0.02,
        n_loss_windows=3,
        n_loss_step=1,
        n_recover_windows=10,
        n_recover_step=1,
        max_n_escalation=4,
    )
    base.update(over)
    return DynamicFecConfig(**base)


# --- compute_k --------------------------------------------------------

def test_compute_k_packets_per_frame_for_8mbps_60fps_mtu1400():
    # 8000 kbps / (60 * 1400 * 8 / 1000) = 8000 / 672 = 11.9 → 11
    cfg = _cfg(k_min=4, k_max=16)
    assert compute_k(bitrate_kbps=8000, mtu_bytes=1400, fps=60, cfg=cfg) == 11


def test_compute_k_clamps_to_k_max():
    cfg = _cfg(k_min=4, k_max=8)
    # High bitrate, low fps → many packets/frame, clamp to k_max.
    assert compute_k(bitrate_kbps=24000, mtu_bytes=1400, fps=30, cfg=cfg) == 8


def test_compute_k_clamps_to_k_min():
    cfg = _cfg(k_min=4, k_max=16)
    # Very low bitrate at high fps → fewer than 4 packets/frame.
    assert compute_k(bitrate_kbps=1000, mtu_bytes=1400, fps=120, cfg=cfg) == 4


def test_compute_k_handles_mtu_3994_60fps_8mbps():
    # MTU 3994 → 8000 / (60 * 3994 * 8 / 1000) = 8000 / 1917 = 4.17 → 4
    cfg = _cfg(k_min=4, k_max=16)
    assert compute_k(bitrate_kbps=8000, mtu_bytes=3994, fps=60, cfg=cfg) == 4


# --- compute_n + NEscalator ------------------------------------------

def test_compute_n_base():
    # k=8, base_ratio=0.5 → n_base = ceil(8 * 1.5) = 12
    cfg = _cfg(base_redundancy_ratio=0.5)
    assert compute_n(k=8, n_escalation=0, cfg=cfg) == 12


def test_compute_n_with_escalation():
    cfg = _cfg(base_redundancy_ratio=0.5, max_redundancy_ratio=1.0)
    assert compute_n(k=8, n_escalation=2, cfg=cfg) == 14


def test_compute_n_capped_by_max_redundancy():
    # k=8, max_ratio=1.0 → n_max = 16. n_escalation=10 → 22, but cap is 16.
    cfg = _cfg(base_redundancy_ratio=0.5, max_redundancy_ratio=1.0)
    assert compute_n(k=8, n_escalation=10, cfg=cfg) == 16


def test_n_escalator_ramps_on_sustained_loss():
    cfg = _cfg(n_loss_threshold=0.02, n_loss_windows=3, n_loss_step=1,
               max_n_escalation=4)
    e = NEscalator(cfg)
    assert e.update(loss=0.05) == 0   # 1st loss window
    assert e.update(loss=0.05) == 0   # 2nd
    assert e.update(loss=0.05) == 1   # 3rd → step up
    assert e.update(loss=0.05) == 1   # streak continues, counter reset
    assert e.update(loss=0.05) == 1
    assert e.update(loss=0.05) == 2   # another 3 windows → step up


def test_n_escalator_resets_loss_counter_on_clean_window():
    cfg = _cfg(n_loss_threshold=0.02, n_loss_windows=3, n_loss_step=1)
    e = NEscalator(cfg)
    e.update(loss=0.05); e.update(loss=0.05)
    assert e.update(loss=0.0) == 0    # clean → reset
    assert e.update(loss=0.05) == 0   # new run starts
    assert e.update(loss=0.05) == 0
    assert e.update(loss=0.05) == 1


def test_n_escalator_recovers_on_sustained_clean():
    cfg = _cfg(n_loss_threshold=0.02, n_loss_windows=3, n_loss_step=1,
               n_recover_windows=10, n_recover_step=1, max_n_escalation=4)
    e = NEscalator(cfg)
    for _ in range(9): e.update(loss=0.05)  # 9 loss → 3 escalation
    assert e.escalation == 3
    for _ in range(9): e.update(loss=0.0)
    assert e.escalation == 3   # 9 clean → not yet
    e.update(loss=0.0)
    assert e.escalation == 2   # 10 clean → step down


def test_n_escalator_clamps_at_max_n_escalation():
    cfg = _cfg(n_loss_threshold=0.02, n_loss_windows=1, n_loss_step=10,
               max_n_escalation=4)
    e = NEscalator(cfg)
    e.update(loss=0.05)   # tries to add 10
    assert e.escalation == 4


# --- EmitGate ---------------------------------------------------------

def test_emit_gate_emits_on_mcs_change():
    gate = EmitGate()
    # Initial: no last-emitted. First call always emits.
    assert gate.should_emit(new_k=8, new_n=12, mcs_changed=False) is True
    gate.commit(k=8, n=12, tick=1)
    # Same values, MCS unchanged, only 1 tick — held.
    assert gate.should_emit(new_k=8, new_n=12, mcs_changed=False) is False
    # MCS-change forces emit even with same (k, n).
    assert gate.should_emit(new_k=8, new_n=12, mcs_changed=True) is True


def test_emit_gate_holds_solo_kn_change_for_two_ticks():
    gate = EmitGate()
    gate.should_emit(8, 12, False); gate.commit(8, 12, tick=0)
    # 1 tick later — different k, same MCS. Below debounce window.
    assert gate.should_emit(9, 12, False, current_tick=1) is False
    # 2 ticks later — passes debounce.
    assert gate.should_emit(9, 12, False, current_tick=2) is True
