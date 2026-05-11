"""Dynamic FEC selection — see spec §"Dynamic FEC algorithm".

Three pieces:
  - `compute_k`: from (bitrate_kbps, mtu_bytes, fps) → integer k,
    clamped to `[k_min, k_max]`. The intuition is "one FEC block
    per frame" (k = packets per frame), so block-fill stays within
    a frame period.
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
    bitrate_kbps: int,
    mtu_bytes: int,
    fps: int,
    cfg: DynamicFecConfig,
) -> int:
    """Packets-per-frame, clamped to [k_min, k_max]."""
    if bitrate_kbps <= 0 or mtu_bytes <= 0 or fps <= 0:
        return cfg.k_min
    packets_per_frame = (bitrate_kbps * 1000.0) / (fps * mtu_bytes * 8.0)
    return max(cfg.k_min, min(cfg.k_max, int(packets_per_frame)))


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
