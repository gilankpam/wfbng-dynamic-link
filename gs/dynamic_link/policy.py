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
    # SNR-driven MCS hysteresis (§4.1). MCS row floors come from the
    # radio profile's snr_floor_dB table; snr_margin_db is operator-
    # tunable safety budget added on top.
    snr_margin_db: float = 3.0
    snr_up_guard_db: float = 2.0
    snr_up_hold_ms: float = 2000.0
    snr_down_hold_ms: float = 500.0
    # After a forced MCS drop (sustained loss or starvation), inhibit
    # any up-climb for this long. Prevents instant snap-back on
    # survivor-biased SNR/RSSI samples.
    forced_drop_inhibit_ms: float = 5000.0
    # RSSI knobs are now consumed only by the TX power inner loop;
    # the legacy rssi_*_hold and rssi_*_guard fields are no longer
    # read but kept here for back-compat YAML parsing.
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
    # §4.2 step-down: how many consecutive zero-loss windows before
    # depth steps down one notch. At 10 Hz, 10 windows = 1 s of clean
    # link before reclaiming a depth step. Walks down one step per
    # threshold-met period (counter resets after each step), so a full
    # depth=3 → 1 recovery takes ~2 s of sustained clean link.
    clean_windows_for_depth_stepdown: int = 10
    # Total-blackout failsafe: this many consecutive starved windows
    # (packet_rate_w < starvation_threshold while session active) trips
    # forced_mcs_drop and pins TX power to max. Intentionally short —
    # at 10 Hz, 5 windows = 0.5 s — because starvation is unambiguous
    # and the alternative is the trailing loop's ~38 s sustained_loss
    # path, which we observed in the live antenna-cover test.
    starvation_windows: int = 5


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
    last_forced_drop_ts: float = 0.0  # post-drop inhibit anchor
    # Row-hysteresis tracking
    candidate_row_idx: int | None = None
    candidate_seen_since_ts: float = 0.0
    candidate_is_upgrade: bool = False


class LeadingLoop:
    """SNR-driven MCS hysteresis + RSSI-driven TX power loop (§4.1).

    The two sub-loops use different signals on purpose: MCS selection
    is bounded by SNR (the actual decode constraint, robust to
    noise-floor changes), while TX power drives RSSI toward target
    (a direct dBm-vs-dBm relationship). They couple through the
    physics: TX power up → RSSI up → SNR up since noise is unchanged.
    """

    def __init__(self, cfg: LeadingLoopConfig, profile: RadioProfile):
        self.cfg = cfg
        self.profile = profile
        # Profile's mcs_max is the static hardware ceiling; cfg.mcs_max
        # is the operator's runtime cap (gs.yaml leading_loop.mcs_max).
        # Clamp to the lower of the two.
        rows = profile.snr_mcs_map(
            cfg.bandwidth, cfg.snr_margin_db, cfg.rssi_margin_db
        )
        rows = [r for r in rows if r.mcs <= cfg.mcs_max]
        if not rows:
            raise ValueError(
                f"leading_loop.mcs_max={cfg.mcs_max} excludes every MCS "
                f"in profile {profile.name!r} "
                f"(mcs_min={profile.mcs_min}, mcs_max={profile.mcs_max})"
            )
        self.rows = rows
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

    def _mcs_down_target(self, snr: float) -> int | None:
        """If smoothed SNR has fallen below current row's floor, return
        index of the row the SNR now lives in (lower-MCS). Else None."""
        cur = self.current_row
        if snr >= cur.snr_floor_dB:
            return None
        # Walk down to find the highest-MCS row whose floor clears.
        for i in range(self.state.current_row_idx + 1, len(self.rows)):
            if snr >= self.rows[i].snr_floor_dB:
                return i
        return len(self.rows) - 1  # below MCS0 floor — last row

    def _mcs_up_target(self, snr: float, ts_ms: float) -> int | None:
        """If SNR is above the next-higher row's floor (+ up_guard) AND
        we're past the post-drop inhibit window, return that row's
        index. Else None."""
        if self.state.current_row_idx == 0:
            return None  # already at top
        # Post-drop inhibit: after a forced drop, hold the dropped MCS
        # for forced_drop_inhibit_ms regardless of how good SNR looks.
        # Defends against survivor-biased SNR samples spuriously
        # triggering an immediate snap-back.
        if (ts_ms - self.state.last_forced_drop_ts
                < self.cfg.forced_drop_inhibit_ms):
            return None
        next_idx = self.state.current_row_idx - 1
        target = self.rows[next_idx]
        if snr >= target.snr_floor_dB + self.cfg.snr_up_guard_db:
            return next_idx
        return None

    def tick(
        self,
        rssi_smooth: float | None,
        snr_smooth: float | None,
        ts_ms: float,
        forced_mcs_drop: bool = False,
        force_tx_power_to: float | None = None,
    ) -> tuple[MCSRow, float, bool]:
        """One tick of the leading loop.

        Returns (chosen_row, tx_power_dBm, mcs_changed).
        `forced_mcs_drop=True` overrides hysteresis to drop one row
        immediately (§4.2 sustained-loss / starvation escalation).
        `force_tx_power_to` pins TX power to a specific dBm,
        bypassing the cooldown and freeze (used by the starvation
        path to push power to max).
        """
        self._reasons = []
        mcs_changed = False

        if forced_mcs_drop and self.state.current_row_idx < len(self.rows) - 1:
            self.state.current_row_idx += 1
            self.state.last_mcs_change_ts = ts_ms
            self.state.last_forced_drop_ts = ts_ms
            self.state.candidate_row_idx = None
            self._reasons.append("forced_mcs_drop")
            mcs_changed = True

        # Outer loop — row hysteresis on SNR. Skip if we have no SNR
        # reading yet or we just made a forced drop.
        elif snr_smooth is not None:
            down = self._mcs_down_target(snr_smooth)
            up = self._mcs_up_target(snr_smooth, ts_ms)
            st = self.state
            if down is not None:
                if st.candidate_row_idx != down or st.candidate_is_upgrade:
                    st.candidate_row_idx = down
                    st.candidate_seen_since_ts = ts_ms
                    st.candidate_is_upgrade = False
                if (ts_ms - st.candidate_seen_since_ts
                        >= self.cfg.snr_down_hold_ms):
                    st.current_row_idx = down
                    st.last_mcs_change_ts = ts_ms
                    st.candidate_row_idx = None
                    self._reasons.append(
                        f"mcs_down snr={snr_smooth:.1f}"
                    )
                    mcs_changed = True
            elif up is not None:
                if st.candidate_row_idx != up or not st.candidate_is_upgrade:
                    st.candidate_row_idx = up
                    st.candidate_seen_since_ts = ts_ms
                    st.candidate_is_upgrade = True
                if (ts_ms - st.candidate_seen_since_ts
                        >= self.cfg.snr_up_hold_ms):
                    st.current_row_idx = up
                    st.last_mcs_change_ts = ts_ms
                    st.candidate_row_idx = None
                    self._reasons.append(
                        f"mcs_up snr={snr_smooth:.1f}"
                    )
                    mcs_changed = True
            else:
                # SNR is inside the current row's band; clear any candidate.
                st.candidate_row_idx = None

        # Inner loop — TX power closed loop on RSSI, OR force-pin override.
        if force_tx_power_to is not None:
            pinned = max(
                self.cfg.tx_power_min_dBm,
                min(self.cfg.tx_power_max_dBm, force_tx_power_to),
            )
            if int(round(pinned)) != int(round(self.state.tx_power_dBm)):
                self.state.tx_power_dBm = pinned
                self.state.last_tx_power_change_ts = ts_ms
                self._reasons.append(f"tx_power_pinned->{pinned:.1f}")
        else:
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


@dataclass
class TrailingState:
    recent_loss_windows: list[bool] = field(default_factory=list)
    last_fec_change_ts: float = 0.0
    last_depth_change_ts: float = 0.0
    # Consecutive zero-loss windows since last loss; drives depth
    # step-down. Resets to 0 on any loss tick or after a step-down.
    consecutive_clean_windows: int = 0


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

        # Track consecutive clean windows for depth step-down.
        if had_loss:
            st.consecutive_clean_windows = 0
        else:
            st.consecutive_clean_windows += 1

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
    """Composes the two loops and runs the latency-budget predictor."""

    def __init__(self, cfg: PolicyConfig, profile: RadioProfile):
        self.cfg = cfg
        self.profile = profile
        self.leading = LeadingLoop(cfg.leading, profile)
        self.trailing = TrailingLoop(cfg)
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

        # Starvation failsafe: after N consecutive starved windows,
        # bypass the trailing loop's sustained_loss accumulator (which
        # is poisoned by dead-window resets — see live antenna-cover
        # test) and force MCS down + TX power to max.
        if signals.link_starved_w:
            self._starvation_count += 1
        else:
            self._starvation_count = 0
        starvation_drop = self._starvation_count >= self.cfg.starvation_windows

        # Trailing sustained-loss check (done before the leading tick
        # so the forced_mcs_drop flag is fresh).
        forced_drop = starvation_drop or (
            self.trailing.sustained_loss() and signals.residual_loss_w > 0.0
        )
        force_tx = self.cfg.leading.tx_power_max_dBm if starvation_drop else None

        row, tx_power, mcs_changed = self.leading.tick(
            signals.rssi,
            signals.snr,
            ts_ms,
            forced_mcs_drop=forced_drop,
            force_tx_power_to=force_tx,
        )

        if starvation_drop:
            # Reset so the next 5 starved windows have to accumulate
            # again before we drop another notch (and so a recovering
            # link clears the counter immediately).
            self._starvation_count = 0

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
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "burst_rate": signals.burst_rate,
                "holdoff_rate": signals.holdoff_rate,
                "packet_rate_w": signals.packet_rate_w,
                "link_starved_w": signals.link_starved_w,
            },
        )
