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
    LADDER_CLIMB,
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
    """Static / hardware-side knobs that aren't part of the gate.

    The dual-gate selector (Channel A SNR-margin / Channel B emergency)
    lives in `GateConfig` and `ProfileSelectionConfig`. This dataclass
    only carries the inputs the selector needs from the radio side
    plus deprecated keys kept for back-compat YAML parsing.
    """
    bandwidth: int = 20
    # MCS-coupled TX power: power = max - (mcs / max_mcs) * (max - min).
    # Atomic per-tick. Inputs to the selector's _compute_tx_power().
    tx_power_min_dBm: float = 5.0
    tx_power_max_dBm: float = 23.0

    # Deprecated — kept so old gs.yaml files still parse. The new
    # selector ignores these. Operator should migrate to `gate:` and
    # `profile_selection:` sections; service.py emits a WARN if any
    # of these are explicitly set.
    mcs_max: int = 7
    snr_margin_db: float = 3.0
    snr_up_guard_db: float = 2.0
    snr_up_hold_ms: float = 2000.0
    snr_down_hold_ms: float = 500.0
    loss_margin_weight: float = 20.0
    fec_margin_weight: float = 20.0
    forced_drop_inhibit_ms: float = 5000.0
    rssi_margin_db: float = 8.0
    rssi_up_guard_db: float = 3.0
    rssi_up_hold_ms: float = 2000.0
    rssi_down_hold_ms: float = 500.0
    rssi_target_dBm: float = -60.0
    rssi_deadband_db: float = 3.0
    tx_power_cooldown_ms: float = 1000.0
    tx_power_freeze_after_mcs_ms: float = 2000.0
    tx_power_step_max_db: float = 3.0
    tx_power_gain_up_db: float = 1.0
    tx_power_gain_down_db: float = 1.0


@dataclass
class GateConfig:
    """Two-channel gate (alink_gs port).

    Channel A (slow/symmetric): SNR-margin hysteresis. Drives normal
    upgrades and graceful downgrades, with stress-aware margin and
    slope-based prediction.

    Channel B (fast/asymmetric): emergency triggers. Loss or FEC
    pressure above threshold force an immediate one-step downgrade
    bypassing rate limit and hold timers.
    """
    # SNR smoothing
    snr_ema_alpha: float = 0.3
    snr_slope_alpha: float = 0.3
    snr_predict_horizon_ticks: float = 3.0
    # Stress-widened margin: base + loss*loss_w + fec*fec_w
    snr_safety_margin: float = 3.0
    loss_margin_weight: float = 20.0  # +1 dB per 0.05 loss
    fec_margin_weight: float = 5.0    # +0.5 dB per 0.10 fec_work
    # Channel A hysteresis (in dB of margin)
    hysteresis_up_db: float = 2.5
    hysteresis_down_db: float = 1.0
    # Channel B emergency thresholds
    emergency_loss_rate: float = 0.05
    emergency_fec_pressure: float = 0.80
    # MCS bounds
    max_mcs: int = 7
    max_mcs_step_up: int = 1


@dataclass
class ProfileSelectionConfig:
    """Timing/cadence knobs for the dual-gate selector."""
    hold_fallback_mode_ms: int = 1000
    hold_modes_down_ms: int = 2000
    min_between_changes_ms: int = 200
    fast_downgrade: bool = True
    upward_confidence_loops: int = 4


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
    gate: GateConfig = field(default_factory=GateConfig)
    selection: ProfileSelectionConfig = field(
        default_factory=ProfileSelectionConfig
    )
    cooldown: CooldownConfig = field(default_factory=CooldownConfig)
    fec: FECBounds = field(default_factory=FECBounds)
    safe: SafeDefaults = field(default_factory=SafeDefaults)
    predictor: PredictorConfig = field(default_factory=PredictorConfig)
    max_latency_ms: float = 50.0
    sustained_loss_windows: int = 3  # §4.2 "last two windows also had loss"
    # §4.2 step-down: how many consecutive zero-loss windows before
    # depth steps down one notch. At 10 Hz, 10 windows = 1 s of clean
    # link before reclaiming a depth step. Walks down one step per
    # threshold-met period (counter resets after each step), so a full
    # depth=3 → 1 recovery takes ~2 s of sustained clean link.
    clean_windows_for_depth_stepdown: int = 10
    # FEC ladder step-down: how many consecutive clean+idle windows
    # (no residual_loss AND fec_work below idle threshold) before
    # one rung of FEC redundancy is reclaimed. Mirrors the depth
    # step-down knob. At 10 Hz, 10 windows = 1 s clean link between
    # rungs. Counter resets after each step; full recovery from
    # (2, 8) to (8, 12) is up to 11 rungs ≈ 11 s.
    clean_windows_for_fec_stepdown: int = 10
    # FEC is "idle enough to recover redundancy" below this fraction
    # of FEC-recovered packets per window. Just below the existing
    # preemptive step-up threshold (0.05) so fec_work in the band
    # 0.02–0.05 holds (k, n) steady — neither tightens nor loosens.
    fec_work_idle_threshold: float = 0.02
    # Total-blackout failsafe: this many consecutive starved windows
    # (packet_rate_w < starvation_threshold while session active) trips
    # forced_mcs_drop and pins TX power to max. Intentionally short —
    # at 10 Hz, 5 windows = 0.5 s — because starvation is unambiguous
    # and the alternative is the trailing loop's ~38 s sustained_loss
    # path, which we observed in the live antenna-cover test.
    starvation_windows: int = 5


# ------------------------------------------------------------------
# Leading selector — dual-gate ProfileSelector (alink_gs port).
# ------------------------------------------------------------------

@dataclass
class LeadingState:
    current_mcs: int                  # currently selected MCS
    tx_power_dBm: float               # last-applied power (inverse-coupled)
    # Initialise the timing anchors well in the past so the first
    # decision after boot doesn't get gated by min_between_changes_ms
    # / hold_modes_down_ms. In production ts_ms is a wall-clock value
    # in the trillions; this just guarantees the same "long-elapsed"
    # condition holds in tests where ts_ms starts near zero.
    last_change_time_ms: float = -1.0e9
    last_mcs_change_time_ms: float = -1.0e9
    up_confidence_count: int = 0           # alink_gs confidence loop
    up_target_mcs: int = -1                # MCS the confidence counter aims at


class LeadingSelector:
    """MCS / TX power selector — dual-gate (Channel A SNR-margin / B
    emergency).

    Port of `alink_gs.ProfileSelector.select()` adapted to dynamic-link's
    split-loop architecture. The selector decides MCS only; FEC ladder,
    bitrate, and depth are computed downstream by the trailing loop and
    bitrate helper.

    Channel A (slow/symmetric): SNR-margin hysteresis with predictive
    horizon (snr_slope) and confidence-loop gating on upgrades.

    Channel B (fast/asymmetric): emergency triggers — loss_rate,
    fec_pressure, or link_starved force an immediate one-step
    downgrade, bypassing rate limit and hold timers.

    TX power follows MCS via inverse coupling: low MCS → high power,
    high MCS → low power. Atomic per tick.
    """

    def __init__(
        self,
        leading: LeadingLoopConfig,
        gate: GateConfig,
        sel: ProfileSelectionConfig,
        profile: RadioProfile,
    ):
        self.leading = leading
        self.gate = gate
        self.sel = sel
        self.profile = profile
        # Build the row table. Profile's mcs_max is the hardware ceiling;
        # gate.max_mcs is the operator's runtime cap. Clamp to the lower.
        cap = min(int(gate.max_mcs), profile.mcs_max)
        if cap < profile.mcs_min:
            raise ValueError(
                f"gate.max_mcs={gate.max_mcs} excludes every MCS in "
                f"profile {profile.name!r} (mcs_min={profile.mcs_min}, "
                f"mcs_max={profile.mcs_max})"
            )
        # rows: descending by MCS (highest first), to match how
        # _pick_mcs walks the table.
        rows = profile.snr_mcs_map(
            leading.bandwidth,
            snr_margin_db=0.0,           # static margin lives in gate
            rssi_margin_db=leading.rssi_margin_db,
        )
        self.rows: list[MCSRow] = [
            r for r in rows if profile.mcs_min <= r.mcs <= cap
        ]
        if not self.rows:
            raise ValueError("LeadingSelector: empty MCS row table")
        self._cap_mcs = cap
        # Boot at the safe-default MCS (1) just like the prior loop.
        start_mcs = 1
        if start_mcs > cap:
            start_mcs = cap
        self.state = LeadingState(
            current_mcs=start_mcs,
            tx_power_dBm=leading.tx_power_max_dBm,  # survival: start at max
        )
        self._reasons: list[str] = []

    # ---- helpers ----

    def _row(self, mcs: int) -> MCSRow:
        for r in self.rows:
            if r.mcs == mcs:
                return r
        return self.rows[-1]   # mcs below mcs_min → lowest row

    @property
    def current_row(self) -> MCSRow:
        return self._row(self.state.current_mcs)

    def _stress_margin_dB(self, loss_rate: float, fec_pressure: float) -> float:
        return (
            self.gate.snr_safety_margin
            + max(0.0, loss_rate) * self.gate.loss_margin_weight
            + max(0.0, fec_pressure) * self.gate.fec_margin_weight
        )

    def _margin(self, mcs: int, snr_ema: float,
                loss_rate: float, fec_pressure: float) -> float:
        """Margin (dB) of `snr_ema` above the stress-widened MCS floor."""
        floor = self._row(mcs).snr_floor_dB
        return snr_ema - floor - self._stress_margin_dB(loss_rate, fec_pressure)

    def _pick_mcs(self, snr_ema: float, loss_rate: float,
                  fec_pressure: float) -> int:
        """Highest MCS whose stress-widened threshold is cleared."""
        for r in self.rows:                # rows are high-MCS first
            if r.mcs > self._cap_mcs:
                continue
            if self._margin(r.mcs, snr_ema, loss_rate, fec_pressure) >= 0:
                return r.mcs
        return self.profile.mcs_min        # below MCS0 floor — bottom row

    def _emergency_active(
        self, loss_rate: float, fec_pressure: float, link_starved: bool
    ) -> bool:
        return (
            loss_rate >= self.gate.emergency_loss_rate
            or fec_pressure >= self.gate.emergency_fec_pressure
            or link_starved
        )

    def _compute_tx_power(self, mcs: int) -> float:
        """Inverse MCS↔power coupling. Atomic per tick."""
        cap = max(1, int(self._cap_mcs))
        t = max(0.0, min(1.0, mcs / cap))
        return (
            self.leading.tx_power_max_dBm
            - t * (self.leading.tx_power_max_dBm
                   - self.leading.tx_power_min_dBm)
        )

    def _try_confidence_feed(self, target: int) -> None:
        """Bump the upward-confidence counter even when hysteresis blocks,
        so when conditions improve the upgrade fires quickly."""
        cur = self.state.current_mcs
        if cur < 0 or target <= cur:
            return
        # Cap by max_mcs_step_up.
        if self.gate.max_mcs_step_up > 0:
            target = min(target, cur + self.gate.max_mcs_step_up)
        if target == cur:
            return
        if target != self.state.up_target_mcs:
            self.state.up_target_mcs = target
            self.state.up_confidence_count = 1
        else:
            self.state.up_confidence_count += 1

    # ---- main entry ----

    def select(
        self,
        snr_ema: float | None,
        snr_slope: float,
        loss_rate: float,
        fec_pressure: float,
        link_starved: bool,
        ts_ms: float,
        snr_raw: float | None = None,
    ) -> tuple[int, float, bool]:
        """Decide MCS for this tick. Returns (mcs, tx_power_dBm, changed).

        Asymmetric SNR: `snr_ema` (smoothed) gates upgrades — stable,
        slow, ignores single-tick spikes. `snr_raw` (latest per-window
        max(snr_avg) across antennas) gates downgrades — fresh, fast,
        catches fast fades within one tick instead of waiting ~500 ms
        for the EWMA to lag-track. If `snr_raw` is None we fall back
        to symmetric behaviour (both decisions use `snr_ema`).
        """
        self._reasons = []
        st = self.state
        cur = st.current_mcs

        # Without an SNR reading, hold current state but still let
        # emergency / starvation force a step-down.
        if snr_ema is None and not link_starved:
            tx = self._compute_tx_power(cur)
            st.tx_power_dBm = tx
            return cur, tx, False

        emergency = self._emergency_active(loss_rate, fec_pressure, link_starved)

        # Channel B: rate limit doesn't apply during emergency.
        if not emergency and (
            ts_ms - st.last_change_time_ms < self.sel.min_between_changes_ms
        ):
            tx = self._compute_tx_power(cur)
            st.tx_power_dBm = tx
            return cur, tx, False

        # Compute candidate from current link state. Asymmetric pick:
        #   - candidate_up via smoothed snr (stable; upgrade-side)
        #   - candidate_down via raw snr_max_w (fresh; downgrade-side)
        # Take the more pessimistic of the two so a fast fade visible
        # in raw drives MCS down before EWMA catches up. When raw
        # spikes high above smoothed, candidate_up bounds us — we
        # don't climb on noisy raw alone.
        if snr_ema is None:
            candidate = cur
        else:
            candidate_up = self._pick_mcs(snr_ema, loss_rate, fec_pressure)
            candidate_down = (
                self._pick_mcs(snr_raw, loss_rate, fec_pressure)
                if snr_raw is not None else candidate_up
            )
            candidate = min(candidate_up, candidate_down)

        # Channel B emergency: at least one MCS step down. If already
        # at the floor, refuse any climb — emergency means link state
        # is bad, climbing on a survivor-biased SNR would walk us
        # straight into a worse oscillation.
        if emergency:
            if cur > self.profile.mcs_min:
                forced = cur - 1
                if candidate > forced:
                    candidate = forced
            else:
                candidate = cur

        # Cap upward step size.
        if (cur >= 0 and candidate > cur and self.gate.max_mcs_step_up > 0):
            candidate = min(candidate, cur + self.gate.max_mcs_step_up)

        # Channel A: SNR-margin hysteresis (skipped during emergency).
        if not emergency and cur >= 0:
            if candidate > cur:
                tgt_margin = self._margin(
                    candidate, snr_ema, loss_rate, fec_pressure
                )
                predicted = (
                    tgt_margin
                    + snr_slope * self.gate.snr_predict_horizon_ticks
                )
                if (tgt_margin < self.gate.hysteresis_up_db
                        or predicted < 0):
                    # Block the upgrade — keep confidence bumping toward it.
                    self._try_confidence_feed(candidate)
                    tx = self._compute_tx_power(cur)
                    st.tx_power_dBm = tx
                    return cur, tx, False
            elif candidate < cur:
                # Use raw snr for the down-margin so a fast fade
                # triggers the drop within one tick instead of waiting
                # ~500 ms for the EWMA to catch up. Falls back to
                # smoothed if raw is unavailable.
                snr_for_down = snr_raw if snr_raw is not None else snr_ema
                cur_margin = self._margin(
                    cur, snr_for_down, loss_rate, fec_pressure
                )
                predicted = (
                    cur_margin
                    + snr_slope * self.gate.snr_predict_horizon_ticks
                )
                if (cur_margin > -self.gate.hysteresis_down_db
                        and predicted > -self.gate.hysteresis_down_db):
                    tx = self._compute_tx_power(cur)
                    st.tx_power_dBm = tx
                    return cur, tx, False

        # Same-MCS path: power follows current MCS, no log-worthy change.
        if candidate == cur:
            st.up_confidence_count = 0
            st.up_target_mcs = -1
            tx = self._compute_tx_power(cur)
            st.tx_power_dBm = tx
            return cur, tx, False

        # Direction + timing + confidence gating.
        is_downgrade = candidate < cur
        elapsed_since_mcs_ms = ts_ms - st.last_mcs_change_time_ms

        if is_downgrade and (emergency or self.sel.fast_downgrade):
            st.up_confidence_count = 0
            st.up_target_mcs = -1
        elif is_downgrade:
            # Slow downgrade — wait out the hold timer.
            if elapsed_since_mcs_ms < self.sel.hold_modes_down_ms:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            st.up_confidence_count = 0
            st.up_target_mcs = -1
        elif cur == self.profile.mcs_min:
            # Climbing out of fallback — extra-conservative hold.
            if elapsed_since_mcs_ms < self.sel.hold_fallback_mode_ms:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
        else:
            # Upgrade: confidence-loop gating + hold timer.
            if candidate != st.up_target_mcs:
                st.up_target_mcs = candidate
                st.up_confidence_count = 1
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            st.up_confidence_count += 1
            if st.up_confidence_count < self.sel.upward_confidence_loops:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False
            if elapsed_since_mcs_ms < self.sel.hold_modes_down_ms:
                tx = self._compute_tx_power(cur)
                st.tx_power_dBm = tx
                return cur, tx, False

        # Apply the change.
        old = cur
        st.current_mcs = candidate
        st.last_change_time_ms = ts_ms
        st.last_mcs_change_time_ms = ts_ms
        st.up_confidence_count = 0
        st.up_target_mcs = -1
        if emergency and is_downgrade:
            cause = "emergency"
            if loss_rate >= self.gate.emergency_loss_rate:
                cause += f" loss={loss_rate:.3f}"
            elif fec_pressure >= self.gate.emergency_fec_pressure:
                cause += f" fec={fec_pressure:.3f}"
            elif link_starved:
                cause += " starved"
            self._reasons.append(f"{cause} mcs {old}->{candidate}")
        elif is_downgrade:
            self._reasons.append(
                f"mcs_down snr={snr_ema:.1f} {old}->{candidate}"
            )
        else:
            self._reasons.append(
                f"mcs_up snr={snr_ema:.1f} {old}->{candidate}"
            )

        tx = self._compute_tx_power(candidate)
        st.tx_power_dBm = tx
        return candidate, tx, True

    @property
    def reasons(self) -> list[str]:
        return list(self._reasons)


# ------------------------------------------------------------------
# Trailing loop — loss-driven protective escalation.
# ------------------------------------------------------------------

def _ladder_step_up(k: int, n: int) -> tuple[int, int]:
    """One step up the §4.2 ladder within the current k-band.

    At n_max of the band, drop to the next lower band's floor — but
    refuse to land on k=1, which mirrors `predictor.fit_or_degrade`'s
    "k=1 isn't a real operating band" rule and matches the per-airframe
    `video_k_min` ceiling. At the floor of the lowest band the loop
    holds (k, n) and lets depth/encoder degradation carry the load.
    """
    band = LADDER_STEPS.get(k)
    if band is None:
        return k, n
    for idx, (bk, bn) in enumerate(band):
        if bk == k and bn == n:
            if idx + 1 < len(band):
                return band[idx + 1]
            dropped = LADDER_DROP.get(k, (k, n))
            if dropped[0] < 2:
                return k, n
            return dropped
    # Current (k, n) isn't on the ladder — rebase at this band's floor.
    return band[0]


def _ladder_step_down(k: int, n: int) -> tuple[int, int]:
    """One step down the §4.2 ladder. Inverse direction of step_up.

    Within a band: decrement to the previous rung in LADDER_STEPS.
    At a band floor: climb to the previous (higher-k) band's TOP rung
    via LADDER_CLIMB.
    Off-ladder or already at band-8 floor: hold.

    Step-down walks every rung in LADDER_STEPS in reverse — including
    band floors that step-up skips. This gives granular rung-by-rung
    recovery toward higher efficiency.
    """
    band = LADDER_STEPS.get(k)
    if band is None:
        return k, n
    for idx, (bk, bn) in enumerate(band):
        if bk == k and bn == n:
            if idx > 0:
                return band[idx - 1]
            return LADDER_CLIMB.get((k, n), (k, n))
    return k, n


@dataclass
class TrailingState:
    recent_loss_windows: list[bool] = field(default_factory=list)
    last_fec_change_ts: float = 0.0
    last_depth_change_ts: float = 0.0
    # Consecutive zero-loss windows since last loss; drives depth
    # step-down. Resets to 0 on any loss tick or after a step-down.
    consecutive_clean_windows: int = 0
    # Stricter than consecutive_clean_windows: increments only when
    # NO loss AND fec_work below the idle threshold. Drives FEC
    # ladder step-down. Resets on any loss tick, on any tick with
    # fec_work above the idle threshold, or after a step-down.
    consecutive_clean_for_fec_windows: int = 0


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
        stepdown_floor: tuple[int, int] | None = None,
    ) -> tuple[int, int, int, bool]:
        """Returns (k, n, depth, idr_request).

        `stepdown_floor` (kept None for back-compat callers) is the
        per-MCS target rung for FEC step-down. The trailing loop will
        not step down past this rung. Callers from `Policy.tick` pass
        `LADDER_STEPS[row.preferred_k][0]` so step-down lands at the
        same operating point that the MCS-change rebase would.
        """
        self._reasons = []
        st = self.state
        had_loss = signals.residual_loss_w > 0.0

        # Trim window history to the configured look-back length.
        st.recent_loss_windows.append(had_loss)
        if len(st.recent_loss_windows) > self.cfg.sustained_loss_windows:
            st.recent_loss_windows = st.recent_loss_windows[
                -self.cfg.sustained_loss_windows:
            ]

        # Track consecutive clean windows for depth step-down.
        if had_loss:
            st.consecutive_clean_windows = 0
        else:
            st.consecutive_clean_windows += 1

        # Track stricter clean+idle windows for FEC step-down. Resets
        # on any loss tick OR on any tick where FEC was actively
        # recovering (fec_work above the idle threshold).
        if (not had_loss
                and signals.fec_work < self.cfg.fec_work_idle_threshold):
            st.consecutive_clean_for_fec_windows += 1
        else:
            st.consecutive_clean_for_fec_windows = 0

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

        # FEC ladder step-down: after sustained clean+idle link,
        # reclaim one rung toward the per-MCS target (stepdown_floor).
        # Same cooldown anchor as step-up — either direction blocks
        # the other for min_change_interval_ms_fec. Don't fire if
        # we're already at the floor or no floor was provided.
        if (stepdown_floor is not None
                and (new_k, new_n) != stepdown_floor
                and (ts_ms - st.last_fec_change_ts
                     >= self.cfg.cooldown.min_change_interval_ms_fec)
                and st.consecutive_clean_for_fec_windows
                    >= self.cfg.clean_windows_for_fec_stepdown):
            stepped = _ladder_step_down(new_k, new_n)
            if stepped != (new_k, new_n):
                new_k, new_n = stepped
                st.last_fec_change_ts = ts_ms
                st.consecutive_clean_for_fec_windows = 0
                self._reasons.append(
                    f"clean*{self.cfg.clean_windows_for_fec_stepdown} "
                    f"fec_work={signals.fec_work:.3f} "
                    f"-> ({new_k},{new_n}) fec_stepdown"
                )

        # Depth path (independent of FEC k/n path above). Two triggers:
        #
        #   bootstrap  (depth=1 → 2): wfb-ng's burst/holdoff counters are
        #     interleaver-internal and structurally zero while depth==1
        #     (rx.cpp:357 short-circuits the interleaved code path), so
        #     they can never trigger the first depth raise. Use a
        #     non-interleaver proxy: sustained loss across the window plus
        #     busy FEC. Once depth>1 the interleaver is engaged and the
        #     real signals become live.
        #
        #   refine     (depth ≥ 2 → higher): the design doc §4.2 trigger
        #     using burst_rate + holdoff_rate. Valid because the interleaver
        #     is on and these counters now reflect reality.
        cooled_depth = (ts_ms - st.last_depth_change_ts
                        >= self.cfg.cooldown.min_change_interval_ms_depth)
        depth_raised = False
        if cooled_depth and new_depth < self.cfg.fec.depth_max:
            bootstrap = (
                current_depth == 1
                and self.sustained_loss()
                and signals.fec_work > 0.10
            )
            refine = (
                signals.burst_rate > 1.0 and signals.holdoff_rate > 0.0
            )
            if bootstrap or refine:
                new_depth = min(new_depth + 1, self.cfg.fec.depth_max)
                st.last_depth_change_ts = ts_ms
                depth_raised = True
                if bootstrap:
                    self._reasons.append(
                        f"sustained_loss fec_work={signals.fec_work:.3f} "
                        f"-> depth={new_depth} (bootstrap)"
                    )
                else:
                    self._reasons.append(
                        f"burst={signals.burst_rate:.1f} "
                        f"holdoff={signals.holdoff_rate:.1f} "
                        f"-> depth={new_depth}"
                    )
        # Step-down (§4.2): after sustained clean link, reclaim a depth
        # step. Walks down one notch per threshold-met period; counter
        # resets so the next step-down requires another clean window.
        # Don't step down on the same tick we raised.
        if (not depth_raised
                and current_depth > 1
                and cooled_depth
                and st.consecutive_clean_windows
                    >= self.cfg.clean_windows_for_depth_stepdown):
            new_depth = max(current_depth - 1, 1)
            st.last_depth_change_ts = ts_ms
            st.consecutive_clean_windows = 0
            self._reasons.append(
                f"clean*{self.cfg.clean_windows_for_depth_stepdown} "
                f"-> depth={new_depth} (stepdown)"
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


def _fec_aware_bitrate_kbps(
    bitrate_Mbps: float, k: int, n: int, safe_k: int, safe_n: int,
) -> int:
    """Scale a profile bitrate so total on-air rate stays bounded as the
    trailing loop tightens FEC. The profile's `bitrate_Mbps` is treated
    as the operator-validated video bitrate at the *default* FEC
    `(safe_k, safe_n)`. As FEC efficiency drops below default, we scale
    video bitrate down by the ratio of efficiencies so on-air bitrate
    stays roughly constant. Capped at the profile value — never auto-
    boost above what the operator tuned.
    """
    default_eff = safe_k / safe_n
    current_eff = k / n
    scale = min(1.0, current_eff / default_eff)
    return int(bitrate_Mbps * 1000 * scale)


class Policy:
    """Composes the dual-gate selector + trailing loop + latency-budget
    predictor."""

    def __init__(self, cfg: PolicyConfig, profile: RadioProfile):
        self.cfg = cfg
        self.profile = profile
        self.leading = LeadingSelector(
            cfg.leading, cfg.gate, cfg.selection, profile
        )
        self.trailing = TrailingLoop(cfg)
        # Per-window link_starved_w can flicker on brief packet-rate
        # dips inside an otherwise-healthy bursty stream. Require N
        # consecutive starved windows before treating the link as
        # actually starved for the selector's emergency channel —
        # loss/FEC pressure remain direct triggers (those are real
        # glitches). At 10 Hz, starvation_windows=5 = 0.5 s of below-
        # threshold packet rate before declaring blackout.
        self._starvation_count: int = 0
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
            bitrate_kbps=_fec_aware_bitrate_kbps(
                row.bitrate_Mbps, cfg.safe.k, cfg.safe.n,
                cfg.safe.k, cfg.safe.n,
            ),
        )

    def tick(self, signals: Signals) -> Decision:
        ts_ms = signals.timestamp * 1000.0 if signals.timestamp else 0.0
        prev = PolicyState(**self.state.__dict__)

        # Starvation hysteresis: per-tick link_starved_w flickers on
        # brief packet-rate dips in bursty video. Require N consecutive
        # starved windows before the selector treats it as emergency.
        if signals.link_starved_w:
            self._starvation_count += 1
        else:
            self._starvation_count = 0
        sustained_starved = (
            self._starvation_count >= self.cfg.starvation_windows
        )

        # Dual-gate selector picks MCS + computes inverse-coupled TX
        # power. Channel B (emergency) is owned by the selector; we
        # don't need to compute forced_drop here anymore.
        # snr_raw (latest per-window max-of-avg across antennas) is
        # used asymmetrically — only for downgrade decisions. Climbs
        # still gate on smoothed snr_ema to avoid bouncing on noise.
        new_mcs, tx_power, mcs_changed = self.leading.select(
            snr_ema=signals.snr,
            snr_raw=signals.snr_max_w,
            snr_slope=signals.snr_slope,
            loss_rate=signals.residual_loss_w,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            ts_ms=ts_ms,
        )
        row = self.leading.current_row

        # On MCS row change, rebase (k, n) to the new band's floor
        # (§4.2 "band boundary crossings").
        if mcs_changed:
            band = LADDER_STEPS.get(row.preferred_k)
            if band is not None:
                self.state.k, self.state.n = band[0]

        # FEC step-down target: the rung the MCS-change rebase would
        # land on for the current MCS row. Step-down stops here so we
        # never recover *past* the operator's per-MCS operating point.
        band = LADDER_STEPS.get(row.preferred_k)
        stepdown_floor = band[0] if band else None

        # Trailing loop operates on the (maybe just-rebased) (k, n).
        new_k, new_n, new_depth, idr = self.trailing.tick(
            signals, self.state.k, self.state.n, self.state.depth, ts_ms,
            stepdown_floor=stepdown_floor,
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
        self.state.bitrate_kbps = _fec_aware_bitrate_kbps(
            row.bitrate_Mbps, new_k, new_n,
            self.cfg.safe.k, self.cfg.safe.n,
        )

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
                "rssi_max_w": signals.rssi_max_w,
                "snr": signals.snr,
                "snr_min_w": signals.snr_min_w,
                "snr_max_w": signals.snr_max_w,
                "snr_slope": signals.snr_slope,
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "burst_rate": signals.burst_rate,
                "holdoff_rate": signals.holdoff_rate,
                "packet_rate_w": signals.packet_rate_w,
                "link_starved_w": signals.link_starved_w,
            },
        )
