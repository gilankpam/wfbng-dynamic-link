"""Policy engine — leading + trailing loops (§4).

Runs at 10 Hz, one tick per RxEvent. Pure function of
(Signals snapshot, internal hysteresis state). Emits a Decision on
every tick; `knobs_changed` records which knobs actually moved.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

from .decision import Decision
from .predictor import (
    LADDER_DROP,
    LADDER_STEPS,
    BudgetExhausted,
    PredictorConfig,
    Proposal,
    fit_or_degrade,
    predict,
)
from .profile import MCSRow, RadioProfile
from .signals import Signals

log = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Config dataclasses — mirror the §6 gs.yaml layout.
# ------------------------------------------------------------------

@dataclass
class LeadingLoopConfig:
    bandwidth: int = 20
    mcs_max: int = 7
    rssi_margin_db: float = 8.0
    rssi_up_guard_db: float = 3.0
    rssi_up_hold_ms: float = 2000.0
    rssi_down_hold_ms: float = 500.0
    rssi_target_dBm: float = -60.0
    rssi_deadband_db: float = 3.0
    tx_power_min_dBm: float = 5.0
    tx_power_max_dBm: float = 23.0
    tx_power_cooldown_ms: float = 1000.0
    tx_power_freeze_after_mcs_ms: float = 2000.0
    tx_power_step_max_db: float = 3.0
    tx_power_gain_up_db: float = 1.0
    tx_power_gain_down_db: float = 1.0


@dataclass
class CooldownConfig:
    min_change_interval_ms_fec: float = 200.0
    min_change_interval_ms_depth: float = 200.0
    min_change_interval_ms_radio: float = 500.0
    min_change_interval_ms_cross: float = 50.0


@dataclass
class FECBounds:
    n_min: int = 4
    n_max: int = 16
    k_min: int = 2
    k_max: int = 8
    depth_max: int = 3


@dataclass
class SafeDefaults:
    k: int = 8
    n: int = 12
    depth: int = 1
    mcs: int = 1
    bitrate_kbps: int = 2000


@dataclass
class PolicyConfig:
    leading: LeadingLoopConfig = field(default_factory=LeadingLoopConfig)
    cooldown: CooldownConfig = field(default_factory=CooldownConfig)
    fec: FECBounds = field(default_factory=FECBounds)
    safe: SafeDefaults = field(default_factory=SafeDefaults)
    predictor: PredictorConfig = field(default_factory=PredictorConfig)
    max_latency_ms: float = 50.0
    sustained_loss_windows: int = 3  # §4.2 "last two windows also had loss"


# ------------------------------------------------------------------
# Leading loop — MCS-row selector with hysteresis.
# ------------------------------------------------------------------

def _find_target_row(rssi: float, rows: list[MCSRow]) -> MCSRow:
    """Pick the highest-MCS row whose floor `rssi` clears.

    Rows are ordered high-MCS first. Returns the lowest row if nothing
    clears (operator sees `rssi` below MCS0 floor; §4.1 'budget exhausted'
    handled elsewhere — we still report a row so the encoder-degrade
    ladder has a reference).
    """
    for row in rows:
        if rssi >= row.rssi_floor_dBm:
            return row
    return rows[-1]


@dataclass
class LeadingState:
    current_row_idx: int  # into the rows list
    tx_power_dBm: float
    last_tx_power_change_ts: float = 0.0
    last_mcs_change_ts: float = 0.0
    # Row-hysteresis tracking
    candidate_row_idx: int | None = None
    candidate_seen_since_ts: float = 0.0
    candidate_is_upgrade: bool = False


class LeadingLoop:
    """RSSI-driven feed-forward (§4.1). Returns (row, tx_power, changed)."""

    def __init__(self, cfg: LeadingLoopConfig, profile: RadioProfile):
        self.cfg = cfg
        self.profile = profile
        self.rows = profile.rssi_mcs_map(cfg.bandwidth, cfg.rssi_margin_db)
        # Start at MCS 1 (safe_defaults) — index in the high-MCS-first list.
        start_mcs = 1
        start_idx = next(
            (i for i, r in enumerate(self.rows) if r.mcs == start_mcs),
            len(self.rows) - 1,
        )
        self.state = LeadingState(
            current_row_idx=start_idx,
            tx_power_dBm=cfg.tx_power_max_dBm,  # survival: start at max
        )
        self._reasons: list[str] = []

    @property
    def current_row(self) -> MCSRow:
        return self.rows[self.state.current_row_idx]

    def _mcs_down_target(self, rssi: float) -> int | None:
        """If smoothed RSSI has fallen below current row's floor, return index
        of the row the RSSI now lives in (lower-MCS). Else None."""
        cur = self.current_row
        if rssi >= cur.rssi_floor_dBm:
            return None
        # Walk down to find the highest-MCS row whose floor clears.
        for i in range(self.state.current_row_idx + 1, len(self.rows)):
            if rssi >= self.rows[i].rssi_floor_dBm:
                return i
        return len(self.rows) - 1  # below MCS0 floor — last row

    def _mcs_up_target(self, rssi: float) -> int | None:
        """If RSSI is well above next-higher row's floor (+ up_guard),
        return that row's index. Else None."""
        if self.state.current_row_idx == 0:
            return None  # already at top
        next_idx = self.state.current_row_idx - 1
        target = self.rows[next_idx]
        if rssi >= target.rssi_floor_dBm + self.cfg.rssi_up_guard_db:
            return next_idx
        return None

    def tick(
        self,
        rssi_smooth: float | None,
        ts_ms: float,
        forced_mcs_drop: bool = False,
    ) -> tuple[MCSRow, float, bool]:
        """One tick of the leading loop.

        Returns (chosen_row, tx_power_dBm, mcs_changed).
        `forced_mcs_drop=True` overrides hysteresis to drop one row
        immediately (§4.2 trailing-loop sustained-loss escalation).
        """
        self._reasons = []
        mcs_changed = False

        if forced_mcs_drop and self.state.current_row_idx < len(self.rows) - 1:
            self.state.current_row_idx += 1
            self.state.last_mcs_change_ts = ts_ms
            self.state.candidate_row_idx = None
            self._reasons.append("forced_mcs_drop")
            mcs_changed = True

        # Outer loop — row hysteresis. Skip if we have no rssi reading yet
        # or we just made a forced drop.
        elif rssi_smooth is not None:
            down = self._mcs_down_target(rssi_smooth)
            up = self._mcs_up_target(rssi_smooth)
            st = self.state
            if down is not None:
                if st.candidate_row_idx != down or st.candidate_is_upgrade:
                    st.candidate_row_idx = down
                    st.candidate_seen_since_ts = ts_ms
                    st.candidate_is_upgrade = False
                if (ts_ms - st.candidate_seen_since_ts
                        >= self.cfg.rssi_down_hold_ms):
                    st.current_row_idx = down
                    st.last_mcs_change_ts = ts_ms
                    st.candidate_row_idx = None
                    self._reasons.append(
                        f"mcs_down rssi={rssi_smooth:.1f}"
                    )
                    mcs_changed = True
            elif up is not None:
                if st.candidate_row_idx != up or not st.candidate_is_upgrade:
                    st.candidate_row_idx = up
                    st.candidate_seen_since_ts = ts_ms
                    st.candidate_is_upgrade = True
                if (ts_ms - st.candidate_seen_since_ts
                        >= self.cfg.rssi_up_hold_ms):
                    st.current_row_idx = up
                    st.last_mcs_change_ts = ts_ms
                    st.candidate_row_idx = None
                    self._reasons.append(
                        f"mcs_up rssi={rssi_smooth:.1f}"
                    )
                    mcs_changed = True
            else:
                # RSSI is inside the current row's band; clear any candidate.
                st.candidate_row_idx = None

        # Inner loop — TX power closed loop.
        self._tick_tx_power(rssi_smooth, ts_ms, mcs_changed)

        return self.current_row, self.state.tx_power_dBm, mcs_changed

    def _tick_tx_power(
        self,
        rssi: float | None,
        ts_ms: float,
        mcs_changed_this_tick: bool,
    ) -> None:
        st = self.state
        c = self.cfg

        # MCS-change freeze (§4.1). 2 s after any MCS move, hold power steady.
        if mcs_changed_this_tick:
            return
        if ts_ms - st.last_mcs_change_ts < c.tx_power_freeze_after_mcs_ms:
            return

        # Cooldown — at most one step per tx_power_cooldown_ms.
        if ts_ms - st.last_tx_power_change_ts < c.tx_power_cooldown_ms:
            return

        if rssi is None:
            return

        delta = c.rssi_target_dBm - rssi  # positive when we're below target
        if abs(delta) < c.rssi_deadband_db:
            return

        gain = c.tx_power_gain_up_db if delta > 0 else c.tx_power_gain_down_db
        step = delta * gain
        step = max(-c.tx_power_step_max_db, min(c.tx_power_step_max_db, step))
        new_power = st.tx_power_dBm + step
        new_power = max(c.tx_power_min_dBm, min(c.tx_power_max_dBm, new_power))
        if int(round(new_power)) == int(round(st.tx_power_dBm)):
            return  # rounded away
        st.tx_power_dBm = new_power
        st.last_tx_power_change_ts = ts_ms
        self._reasons.append(f"tx_power->{new_power:.1f}")

    @property
    def reasons(self) -> list[str]:
        return list(self._reasons)


# ------------------------------------------------------------------
# Trailing loop — loss-driven protective escalation.
# ------------------------------------------------------------------

def _ladder_step_up(k: int, n: int) -> tuple[int, int]:
    """One step up the §4.2 ladder within the current k-band.

    At n_max of the band, drop to the next lower band's floor.
    """
    band = LADDER_STEPS.get(k)
    if band is None:
        return k, n
    for idx, (bk, bn) in enumerate(band):
        if bk == k and bn == n:
            if idx + 1 < len(band):
                return band[idx + 1]
            return LADDER_DROP.get(k, (k, n))
    # Current (k, n) isn't on the ladder — rebase at this band's floor.
    return band[0]


@dataclass
class TrailingState:
    recent_loss_windows: list[bool] = field(default_factory=list)
    last_fec_change_ts: float = 0.0
    last_depth_change_ts: float = 0.0


class TrailingLoop:
    """Loss-driven protective escalation (§4.2)."""

    def __init__(self, cfg: PolicyConfig):
        self.cfg = cfg
        self.state = TrailingState()
        self._reasons: list[str] = []

    def tick(
        self,
        signals: Signals,
        current_k: int,
        current_n: int,
        current_depth: int,
        ts_ms: float,
    ) -> tuple[int, int, int, bool]:
        """Returns (k, n, depth, idr_request)."""
        self._reasons = []
        st = self.state
        had_loss = signals.residual_loss_w > 0.0

        # Trim window history to the configured look-back length.
        st.recent_loss_windows.append(had_loss)
        if len(st.recent_loss_windows) > self.cfg.sustained_loss_windows:
            st.recent_loss_windows = st.recent_loss_windows[
                -self.cfg.sustained_loss_windows:
            ]

        new_k, new_n, new_depth = current_k, current_n, current_depth
        idr = False

        if had_loss:
            # §4.2: immediate escalation — bypass min_change_interval_ms_fec.
            new_k, new_n = _ladder_step_up(current_k, current_n)
            st.last_fec_change_ts = ts_ms
            idr = True
            self._reasons.append(
                f"residual_loss={signals.residual_loss_w:.3f} "
                f"-> ({new_k},{new_n}) +IDR"
            )
            # Depth escalation only when burst is clumpy AND holdoff firing.
            if (signals.burst_rate > 1.0 and signals.holdoff_rate > 0.0
                    and new_depth < self.cfg.fec.depth_max):
                # Don't lower depth + n in same tick — we're raising both
                # but never lowering both, so no conflict here.
                if ts_ms - st.last_depth_change_ts >= (
                    self.cfg.cooldown.min_change_interval_ms_depth
                ):
                    new_depth = min(new_depth + 1, self.cfg.fec.depth_max)
                    st.last_depth_change_ts = ts_ms
                    self._reasons.append(
                        f"burst={signals.burst_rate:.1f} "
                        f"holdoff={signals.holdoff_rate:.1f} "
                        f"-> depth={new_depth}"
                    )
        elif signals.fec_work > 0.05:
            # Preemptive raise — respect cooldown.
            if (ts_ms - st.last_fec_change_ts
                    >= self.cfg.cooldown.min_change_interval_ms_fec):
                stepped_k, stepped_n = _ladder_step_up(current_k, current_n)
                if (stepped_k, stepped_n) != (current_k, current_n):
                    new_k, new_n = stepped_k, stepped_n
                    st.last_fec_change_ts = ts_ms
                    self._reasons.append(
                        f"fec_work={signals.fec_work:.3f} "
                        f"-> ({new_k},{new_n}) preemptive"
                    )

        return new_k, new_n, new_depth, idr

    @property
    def reasons(self) -> list[str]:
        return list(self._reasons)

    def sustained_loss(self) -> bool:
        """True when the last N windows all had loss (§4.2)."""
        if len(self.state.recent_loss_windows) < self.cfg.sustained_loss_windows:
            return False
        return all(self.state.recent_loss_windows)


# ------------------------------------------------------------------
# Top-level policy: composes leading + trailing + predictor.
# ------------------------------------------------------------------

@dataclass
class PolicyState:
    mcs: int
    bandwidth: int
    tx_power_dBm: int
    k: int
    n: int
    depth: int
    bitrate_kbps: int


class Policy:
    """Composes the two loops and runs the latency-budget predictor."""

    def __init__(self, cfg: PolicyConfig, profile: RadioProfile):
        self.cfg = cfg
        self.profile = profile
        self.leading = LeadingLoop(cfg.leading, profile)
        self.trailing = TrailingLoop(cfg)
        # Boot with safe_defaults — keeps the controller safe on a
        # link that hasn't produced an RSSI reading yet.
        row = self.leading.current_row
        self.state = PolicyState(
            mcs=row.mcs,
            bandwidth=cfg.leading.bandwidth,
            tx_power_dBm=int(self.leading.state.tx_power_dBm),
            k=cfg.safe.k,
            n=cfg.safe.n,
            depth=cfg.safe.depth,
            bitrate_kbps=int(row.bitrate_Mbps * 1000),
        )

    def tick(self, signals: Signals) -> Decision:
        ts_ms = signals.timestamp * 1000.0 if signals.timestamp else 0.0
        prev = PolicyState(**self.state.__dict__)

        # Trailing sustained-loss check (done before the leading tick
        # so the forced_mcs_drop flag is fresh).
        forced_drop = False
        if self.trailing.sustained_loss() and signals.residual_loss_w > 0.0:
            forced_drop = True

        row, tx_power, mcs_changed = self.leading.tick(
            signals.rssi, ts_ms, forced_mcs_drop=forced_drop
        )

        # On MCS row change, rebase (k, n) to the new band's floor
        # (§4.2 "band boundary crossings").
        if mcs_changed:
            band = LADDER_STEPS.get(row.preferred_k)
            if band is not None:
                self.state.k, self.state.n = band[0]

        # Trailing loop operates on the (maybe just-rebased) (k, n).
        new_k, new_n, new_depth, idr = self.trailing.tick(
            signals, self.state.k, self.state.n, self.state.depth, ts_ms,
        )

        # Latency-budget gate — drop depth / drop k until it fits.
        proposal = Proposal(k=new_k, n=new_n, depth=new_depth)
        reason_budget = ""
        try:
            adjusted = fit_or_degrade(
                proposal, self.cfg.max_latency_ms, self.cfg.predictor
            )
            if adjusted != proposal:
                reason_budget = (
                    f"budget_degrade {proposal}->{adjusted}"
                )
            new_k, new_n, new_depth = adjusted.k, adjusted.n, adjusted.depth
        except BudgetExhausted:
            # Refuse: hold current state and flag budget_exhausted.
            reason_budget = "budget_exhausted"
            new_k, new_n, new_depth = (
                self.state.k, self.state.n, self.state.depth
            )

        # Commit new state.
        self.state.mcs = row.mcs
        self.state.tx_power_dBm = int(round(tx_power))
        self.state.k = new_k
        self.state.n = new_n
        self.state.depth = new_depth
        self.state.bitrate_kbps = int(row.bitrate_Mbps * 1000)

        # Assemble Decision.
        knobs_changed: list[str] = []
        if self.state.mcs != prev.mcs:
            knobs_changed.append("mcs")
        if self.state.bitrate_kbps != prev.bitrate_kbps:
            knobs_changed.append("bitrate")
        if self.state.tx_power_dBm != prev.tx_power_dBm:
            knobs_changed.append("tx_power")
        if (self.state.k, self.state.n) != (prev.k, prev.n):
            knobs_changed.append("fec")
        if self.state.depth != prev.depth:
            knobs_changed.append("depth")
        if idr:
            knobs_changed.append("idr")

        reasons = self.leading.reasons + self.trailing.reasons
        if reason_budget:
            reasons.append(reason_budget)

        return Decision(
            timestamp=signals.timestamp,
            mcs=self.state.mcs,
            bandwidth=self.state.bandwidth,
            tx_power_dBm=self.state.tx_power_dBm,
            k=self.state.k,
            n=self.state.n,
            depth=self.state.depth,
            bitrate_kbps=self.state.bitrate_kbps,
            idr_request=idr,
            reason="; ".join(reasons),
            knobs_changed=knobs_changed,
            signals_snapshot={
                "rssi": signals.rssi,
                "rssi_min_w": signals.rssi_min_w,
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "burst_rate": signals.burst_rate,
                "holdoff_rate": signals.holdoff_rate,
            },
        )
