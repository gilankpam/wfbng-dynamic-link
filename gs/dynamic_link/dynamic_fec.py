"""Dynamic FEC selection — see spec §"Dynamic FEC algorithm".

Three pieces:
  - `compute_k`: from (wire_target_kbps, mtu_bytes, fps) → integer k,
    anchored on the encoder rate (wire/(1+base_red)) and divided by
    cfg.blocks_per_frame. Default bpf = 1 + max_red, giving
    block_fill_ms ≤ frame_period_ms at all n ≤ n_max.
  - `NEscalator`: tracks `n_escalation` across ticks; ramps up on
    sustained `residual_loss`, decrements on sustained clean
    windows. Hysteresis is the load-bearing piece — bare-reactive
    n changes are the cadence cost the bench warned against.
  - `EmitGate`: gates when the freshly computed `(k, n)` is allowed
    to ride the wire. Always emits on MCS change; otherwise debounces
    solo `(k, n)` changes for two ticks.

All three are pure / stateless except for `NEscalator` and
`EmitGate`, which carry small per-tick state and have explicit
`update()` / `commit()` methods.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DynamicFecConfig:
    k_min: int = 4
    k_max: int = 16
    base_redundancy_ratio: float = 0.5
    max_redundancy_ratio: float = 1.0
    blocks_per_frame: float = 2.0
    n_loss_threshold: float = 0.02
    n_loss_windows: int = 3
    n_loss_step: int = 1
    n_recover_windows: int = 10
    n_recover_step: int = 1
    max_n_escalation: int = 4

    def __post_init__(self) -> None:
        if self.k_min < 1:
            raise ValueError(f"k_min must be >= 1; got {self.k_min}")
        if self.k_max < self.k_min:
            raise ValueError(
                f"k_max ({self.k_max}) < k_min ({self.k_min})"
            )
        if self.base_redundancy_ratio < 0:
            raise ValueError(
                f"base_redundancy_ratio must be >= 0; "
                f"got {self.base_redundancy_ratio}"
            )
        if self.max_redundancy_ratio < self.base_redundancy_ratio:
            raise ValueError(
                f"max_redundancy_ratio ({self.max_redundancy_ratio}) "
                f"< base ({self.base_redundancy_ratio})"
            )
        if self.blocks_per_frame <= 0:
            raise ValueError(
                f"blocks_per_frame must be > 0; "
                f"got {self.blocks_per_frame}"
            )
        if self.max_n_escalation < 0:
            raise ValueError(
                f"max_n_escalation must be >= 0; "
                f"got {self.max_n_escalation}"
            )
        if self.n_loss_windows < 1:
            raise ValueError(
                f"n_loss_windows must be >= 1; "
                f"got {self.n_loss_windows}"
            )
        if self.n_recover_windows < 1:
            raise ValueError(
                f"n_recover_windows must be >= 1; "
                f"got {self.n_recover_windows}"
            )
        if self.n_loss_step < 0:
            raise ValueError(
                f"n_loss_step must be >= 0; got {self.n_loss_step}"
            )
        if self.n_recover_step < 0:
            raise ValueError(
                f"n_recover_step must be >= 0; "
                f"got {self.n_recover_step}"
            )


def compute_k(
    *,
    wire_target_kbps: float,
    mtu_bytes: int,
    fps: int,
    cfg: DynamicFecConfig,
) -> int:
    """Block size, sized so block_fill stays inside one frame period
    even at maximum n escalation.

    Anchored on the encoder bitrate at n_base (= wire_target / (1 +
    base_redundancy_ratio)) rather than on the wire rate, so block_fill
    does not inflate as n grows above n_base. The result is divided by
    `cfg.blocks_per_frame`; with the default (1 + max_redundancy_ratio)
    the bound holds at every emitted n by construction. See
    docs/superpowers/specs/2026-05-23-block-fill-enforcement-design.md.
    """
    if wire_target_kbps <= 0 or mtu_bytes <= 0 or fps <= 0:
        return cfg.k_min
    anchor_encoder_kbps = wire_target_kbps / (1.0 + cfg.base_redundancy_ratio)
    packets_per_frame = (
        anchor_encoder_kbps * 1000.0 / (fps * mtu_bytes * 8.0)
    )
    k = packets_per_frame / cfg.blocks_per_frame
    return max(cfg.k_min, min(cfg.k_max, int(k)))


def compute_n(
    *,
    k: int,
    n_escalation: int,
    cfg: DynamicFecConfig,
) -> int:
    """n_base + escalation, capped at the max-redundancy ceiling."""
    n_base = math.ceil(k * (1.0 + cfg.base_redundancy_ratio))
    n_max = math.ceil(k * (1.0 + cfg.max_redundancy_ratio))
    n = n_base + max(0, n_escalation)
    return min(n, n_max)


def clamp_n_for_bitrate_floor(
    n_candidate: int,
    k: int,
    wire_target_kbps: float,
    min_bitrate_kbps: int,
) -> int:
    """Cap n so that bitrate = wire_target_kbps × k / n stays ≥ min_bitrate_kbps.

    Preserves the wire-safety invariant at the bitrate floor: when
    n_escalation would push encoder bitrate below the floor, we stop
    growing FEC instead of letting wire rate inflate past PHY.

    Pathological edge: when `min_bitrate_kbps > wire_target_kbps`
    (link can't carry minimum video at all), `n_max_phy < k` and we
    cap at k (degenerate: no parity). The wire-safety invariant
    lightly bends here, but only when the link is failing beyond
    what this layer can fix.
    """
    if k <= 0:
        raise ValueError(f"k must be > 0; got {k}")
    if min_bitrate_kbps <= 0:
        raise ValueError(f"min_bitrate_kbps must be > 0; got {min_bitrate_kbps}")
    if wire_target_kbps <= 0:
        raise ValueError(f"wire_target_kbps must be > 0; got {wire_target_kbps}")
    n_max_phy = int(wire_target_kbps * k / min_bitrate_kbps)
    return max(k, min(n_candidate, n_max_phy))


@dataclass
class NEscalator:
    cfg: DynamicFecConfig
    escalation: int = 0
    _loss_streak: int = 0
    _clean_streak: int = 0

    def update(self, *, loss: float) -> int:
        """Apply one tick's residual_loss. Returns the new escalation."""
        if loss > self.cfg.n_loss_threshold:
            self._clean_streak = 0
            self._loss_streak += 1
            if self._loss_streak >= self.cfg.n_loss_windows:
                self.escalation = min(
                    self.cfg.max_n_escalation,
                    self.escalation + self.cfg.n_loss_step,
                )
                self._loss_streak = 0
        elif loss == 0.0:
            self._loss_streak = 0
            self._clean_streak += 1
            if self._clean_streak >= self.cfg.n_recover_windows:
                self.escalation = max(
                    0,
                    self.escalation - self.cfg.n_recover_step,
                )
                self._clean_streak = 0
        else:
            # 0 < loss <= threshold: borderline — no escalation, but
            # don't count toward recovery either.
            self._loss_streak = 0
            self._clean_streak = 0
        return self.escalation


@dataclass
class EmitGate:
    """Decides whether a freshly computed (k, n) should ride the wire
    this tick or be held."""
    last_k: int | None = None
    last_n: int | None = None
    last_emit_tick: int = -1_000_000
    debounce_ticks: int = 2

    def should_emit(
        self,
        new_k: int,
        new_n: int,
        mcs_changed: bool,
        *,
        current_tick: int = 0,
    ) -> bool:
        if self.last_k is None:
            return True
        if mcs_changed:
            return True
        if new_k == self.last_k and new_n == self.last_n:
            return False
        if current_tick - self.last_emit_tick >= self.debounce_ticks:
            return True
        return False

    def commit(self, k: int, n: int, tick: int) -> None:
        self.last_k = k
        self.last_n = n
        self.last_emit_tick = tick
