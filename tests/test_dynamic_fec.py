"""Unit tests for the dynamic FEC selection module."""
from __future__ import annotations

import pytest

from dynamic_link.dynamic_fec import (
    DynamicFecConfig,
    EmitGate,
    NEscalator,
    clamp_n_for_bitrate_floor,
    compute_k,
    compute_n,
)


def _cfg(**over):
    base = dict(
        k_min=4, k_max=16,
        base_redundancy_ratio=0.5,
        max_redundancy_ratio=1.0,
        blocks_per_frame=2.0,
        n_loss_threshold=0.02,
        n_loss_windows=3,
        n_loss_step=1,
        n_recover_windows=10,
        n_recover_step=1,
        max_n_escalation=4,
    )
    base.update(over)
    return DynamicFecConfig(**base)


# --- DynamicFecConfig validation ------------------------------------------

def test_dynamic_fec_config_rejects_nonpositive_blocks_per_frame():
    with pytest.raises(ValueError, match="blocks_per_frame must be > 0"):
        _cfg(blocks_per_frame=0)
    with pytest.raises(ValueError, match="blocks_per_frame must be > 0"):
        _cfg(blocks_per_frame=-1.5)


# --- compute_k -------------------------------------------------------
# Anchored on encoder bitrate at n_base
# (anchor_encoder = wire_target / (1 + base_red)), then divided by
# cfg.blocks_per_frame. See dynamic_fec module docstring.

def test_compute_k_anchor_encoder_division():
    # wire=20000, base_red=0.4 -> anchor_encoder = 20000/1.4 ≈ 14285.7
    # pps = 14285.7 * 1000 / (60 * 1500 * 8) ≈ 19.84
    # k_raw = 19.84 / 2.0 = 9.92 -> int = 9
    cfg = _cfg(
        k_min=2, k_max=50,
        base_redundancy_ratio=0.4,
        max_redundancy_ratio=1.0,
        blocks_per_frame=2.0,
    )
    assert compute_k(
        wire_target_kbps=20000, mtu_bytes=1500, fps=60, cfg=cfg,
    ) == 9


def test_compute_k_clamps_to_k_max():
    # High wire, low fps, low base_red, low bpf -> many packets/frame,
    # clamp to k_max.
    cfg = _cfg(
        k_min=4, k_max=8,
        base_redundancy_ratio=0.0,   # no anchor compression
        max_redundancy_ratio=1.0,
        blocks_per_frame=1.0,        # no divisor effect
    )
    assert compute_k(
        wire_target_kbps=24000, mtu_bytes=1400, fps=30, cfg=cfg,
    ) == 8


def test_compute_k_clamps_to_k_min():
    # Very low wire at high fps -> fewer than k_min packets/frame.
    cfg = _cfg(k_min=4, k_max=16, base_redundancy_ratio=0.5, blocks_per_frame=2.0)
    assert compute_k(
        wire_target_kbps=1000, mtu_bytes=1400, fps=120, cfg=cfg,
    ) == 4


def test_compute_k_falls_back_to_kmin_on_degenerate_inputs():
    cfg = _cfg(k_min=4, k_max=16)
    assert compute_k(wire_target_kbps=0, mtu_bytes=1400, fps=60, cfg=cfg) == 4
    assert compute_k(wire_target_kbps=8000, mtu_bytes=0, fps=60, cfg=cfg) == 4
    assert compute_k(wire_target_kbps=8000, mtu_bytes=1400, fps=0, cfg=cfg) == 4


def test_compute_k_higher_bpf_shrinks_k():
    # Same inputs, bigger bpf -> smaller k.
    cfg_lo = _cfg(k_min=2, k_max=50, base_redundancy_ratio=0.4, blocks_per_frame=1.0)
    cfg_hi = _cfg(k_min=2, k_max=50, base_redundancy_ratio=0.4, blocks_per_frame=4.0)
    k_lo = compute_k(wire_target_kbps=20000, mtu_bytes=1500, fps=60, cfg=cfg_lo)
    k_hi = compute_k(wire_target_kbps=20000, mtu_bytes=1500, fps=60, cfg=cfg_hi)
    assert k_hi < k_lo, f"expected k_hi < k_lo; got {k_hi} >= {k_lo}"


def test_block_fill_bound_holds_with_default_bpf():
    """For every (MCS 0..7, n_escalation 0..max) at the deploy radio
    profile and (mtu, fps), the post-clamp n satisfies
    block_fill_ms <= frame_period_ms when k > k_min.
    When k == k_min the k_min floor can produce block_fill slightly
    above one frame period (by at most one packet-time), which is
    documented as acceptable for the edge-of-range / degenerate case.
    This is the load-bearing invariant the spec restores.
    """
    from pathlib import Path
    from dynamic_link.bitrate import compute_wire_target_kbps
    from dynamic_link.profile import load_profile_file

    prof = load_profile_file(Path("conf/radios/m8812eu2.yaml"))
    cfg = _cfg(
        k_min=2, k_max=50,
        base_redundancy_ratio=0.4, max_redundancy_ratio=1.0,
        blocks_per_frame=2.0,   # = 1 + max_red, hard-bound default
        max_n_escalation=6,
    )
    util = 0.7
    for mtu, fps in [(1500, 60), (1400, 60), (1500, 90)]:
        mtu_bits = mtu * 8
        frame_period_ms = 1000.0 / fps
        for mcs in range(prof.mcs_min, prof.mcs_max + 1):
            wire = compute_wire_target_kbps(prof, 20, mcs, mtu, util)
            k = compute_k(
                wire_target_kbps=wire, mtu_bytes=mtu, fps=fps, cfg=cfg,
            )
            for esc in range(cfg.max_n_escalation + 1):
                n_unclamped = compute_n(k=k, n_escalation=esc, cfg=cfg)
                n = clamp_n_for_bitrate_floor(
                    n_unclamped, k, wire, min_bitrate_kbps=1000,
                )
                block_fill_ms = n * mtu_bits / wire
                # When k > k_min the formula guarantees block_fill < frame_period.
                # When k == k_min the hard floor breaks the tight bound; allow
                # up to one extra packet-time of slack (≈ mtu_bits / wire ms).
                one_pkt_ms = mtu_bits / wire
                applicable = frame_period_ms if k > cfg.k_min else frame_period_ms + one_pkt_ms
                assert block_fill_ms <= applicable + 1e-6, (
                    f"mtu={mtu} fps={fps} mcs={mcs} esc={esc}: "
                    f"block_fill={block_fill_ms:.2f}ms > "
                    f"applicable={applicable:.2f}ms (k={k} n={n})"
                )


def test_block_fill_bound_holds_when_bpf_exceeds_floor():
    """Same parameterization at bpf=3.0 — formula is correct above
    the hard-bound floor too.
    When k > k_min, block_fill stays well below one frame period.
    When k == k_min, fall back to the looser one-frame + one-packet bound.
    """
    from pathlib import Path
    from dynamic_link.bitrate import compute_wire_target_kbps
    from dynamic_link.profile import load_profile_file

    prof = load_profile_file(Path("conf/radios/m8812eu2.yaml"))
    cfg = _cfg(
        k_min=2, k_max=50,
        base_redundancy_ratio=0.4, max_redundancy_ratio=1.0,
        blocks_per_frame=3.0,
        max_n_escalation=6,
    )
    util = 0.7
    mtu, fps = 1500, 60
    mtu_bits = mtu * 8
    frame_period_ms = 1000.0 / fps
    for mcs in range(prof.mcs_min, prof.mcs_max + 1):
        wire = compute_wire_target_kbps(prof, 20, mcs, mtu, util)
        k = compute_k(wire_target_kbps=wire, mtu_bytes=mtu, fps=fps, cfg=cfg)
        for esc in range(cfg.max_n_escalation + 1):
            n = clamp_n_for_bitrate_floor(
                compute_n(k=k, n_escalation=esc, cfg=cfg),
                k, wire, min_bitrate_kbps=1000,
            )
            block_fill_ms = n * mtu_bits / wire
            # At low MCS the k_min floor may bite, relaxing the bound.
            # When k > k_min, block_fill stays below one frame period.
            # For k == k_min, allow one extra packet-time of slack.
            one_pkt_ms = mtu_bits / wire
            applicable = frame_period_ms if k > cfg.k_min else frame_period_ms + one_pkt_ms
            assert block_fill_ms <= applicable + 1e-6, (
                f"mcs={mcs} esc={esc} k={k} n={n}: "
                f"block_fill={block_fill_ms:.2f}ms > {applicable:.2f}ms"
            )


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


# --- clamp_n_for_bitrate_floor ---------------------------------------

def test_clamp_n_headroom_case_returns_unchanged():
    """n_candidate well below n_max_phy → returned unchanged."""
    # wire=3568, k=4 → n_max_phy = floor(3568*4/1000) = 14
    # n_candidate=6 < 14 → returned unchanged
    assert clamp_n_for_bitrate_floor(
        n_candidate=6, k=4,
        wire_target_kbps=3568.0,
        min_bitrate_kbps=1000,
    ) == 6


def test_clamp_n_capped_to_n_max_phy():
    """n_candidate > n_max_phy → returns n_max_phy; bitrate stays ≥ floor."""
    # wire=3568, k=4, floor=1000 → n_max_phy = floor(14272/1000) = 14
    # n_candidate=20 → cap at 14
    n = clamp_n_for_bitrate_floor(
        n_candidate=20, k=4,
        wire_target_kbps=3568.0,
        min_bitrate_kbps=1000,
    )
    assert n == 14
    # Sanity: bitrate at the capped n is ≥ floor
    bitrate = int(3568.0 * 4 / n)
    assert bitrate >= 1000


def test_clamp_n_degenerate_link_falls_back_to_k():
    """When wire_target < min_bitrate (link can't carry minimum video),
    n_max_phy < k. Helper returns k (no parity, wire-safety lightly
    bent — see spec)."""
    # wire=500, k=4, floor=1000 → n_max_phy = floor(2000/1000) = 2 < k
    assert clamp_n_for_bitrate_floor(
        n_candidate=6, k=4,
        wire_target_kbps=500.0,
        min_bitrate_kbps=1000,
    ) == 4


def test_clamp_n_at_exact_n_max_phy_boundary_returns_n_max_phy():
    """Inclusive boundary: when n_candidate == n_max_phy, clamp returns it."""
    # wire=3568, k=4, floor=1000 → n_max_phy = floor(14272/1000) = 14
    # n_candidate=14 exactly → returns 14 (not 13)
    assert clamp_n_for_bitrate_floor(
        n_candidate=14, k=4,
        wire_target_kbps=3568.0,
        min_bitrate_kbps=1000,
    ) == 14


def test_clamp_n_rejects_invalid_args():
    """Defensive guards: k, min_bitrate_kbps, wire_target_kbps must be > 0."""
    import pytest
    with pytest.raises(ValueError, match="k must be > 0"):
        clamp_n_for_bitrate_floor(n_candidate=6, k=0, wire_target_kbps=3568.0, min_bitrate_kbps=1000)
    with pytest.raises(ValueError, match="min_bitrate_kbps must be > 0"):
        clamp_n_for_bitrate_floor(n_candidate=6, k=4, wire_target_kbps=3568.0, min_bitrate_kbps=0)
    with pytest.raises(ValueError, match="wire_target_kbps must be > 0"):
        clamp_n_for_bitrate_floor(n_candidate=6, k=4, wire_target_kbps=0.0, min_bitrate_kbps=1000)
