"""Per-window signal aggregator and EWMA smoother (§3 "Derived metrics").

Each RxEvent carries one 100 ms window of counters. We compute the six
derived signals per window, then EWMA-smooth most of them. residual_loss
is intentionally *not* smoothed — one lost block in a window is already
a visible FPV glitch (§3).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .stats_client import RxEvent, SessionInfo

WINDOW_S = 0.1  # design cadence: log_interval = 100 ms (§3)


@dataclass
class Signals:
    """One tick's view of the controller inputs.

    Raw `_w` fields are per-100 ms-window; EWMA-smoothed inputs have no
    suffix. All optional fields are None until the first window arrives.
    """
    # Raw per-window
    rssi_min_w: float | None = None       # min across antennas of rssi_min
    rssi_avg_w: float | None = None       # diversity-combined estimate
    rssi_max_w: float | None = None       # max(rssi_avg) — best-antenna operating point
    snr_min_w: float | None = None
    snr_avg_w: float | None = None
    snr_max_w: float | None = None        # max(snr_avg) — best-antenna SNR
    residual_loss_w: float = 0.0          # used raw — no smoothing (§3)
    fec_work_rate_w: float = 0.0
    packet_rate_w: float = 0.0            # fragments / sec
    burst_rate_w: float = 0.0             # events / sec
    holdoff_rate_w: float = 0.0
    late_rate_w: float = 0.0
    # Starvation flag: link is up (session known) but data fragments
    # have dropped to near zero — distinct from "healthy idle" because
    # a session implies the drone is still TXing.
    link_starved_w: bool = False

    # EWMA-smoothed controller inputs
    rssi: float | None = None
    snr: float | None = None
    snr_slope: float = 0.0    # EMA of per-tick Δs.snr (dB/tick); + = rising
    fec_work: float = 0.0
    burst_rate: float = 0.0
    holdoff_rate: float = 0.0
    late_rate: float = 0.0

    # Last session seen (for knowing current k, n, depth).
    session: SessionInfo | None = None

    timestamp: float = 0.0
    windows_seen: int = 0
    ant_count: int = 0


def _ewma(prev: float | None, new: float, alpha: float) -> float:
    if prev is None:
        return new
    return alpha * new + (1.0 - alpha) * prev


@dataclass
class SignalAggregator:
    """Folds each RxEvent into the running Signals snapshot.

    Ownership: one aggregator per running service. Call `consume(ev)`
    on every RxEvent; read `.signals` at controller tick time.
    """
    ewma_alpha_rssi: float = 0.2
    ewma_alpha_fec: float = 0.2
    ewma_alpha_burst: float = 0.1
    # SNR-slope EMA: smooths the per-tick Δsnr so the dual-gate
    # selector can extrapolate trend. Higher α = faster trend
    # tracking, more chatter under noise.
    ewma_alpha_snr_slope: float = 0.3
    # link_starved_w threshold: data fragments/sec below this counts as
    # starved. Compared per-window against packet_rate_w. Default 50 pps
    # is well below an FPV video stream's nominal ~700-1500 pps but well
    # above background noise from a stalled stream.
    starvation_threshold_pps: float = 50.0

    signals: Signals = field(default_factory=Signals)
    # Last s.snr observed; used to compute Δsnr for the slope EMA.
    _prev_snr: float | None = field(default=None, repr=False)

    def update_session(self, session: SessionInfo) -> None:
        self.signals.session = session

    def consume(self, ev: RxEvent) -> Signals:
        s = self.signals
        s.timestamp = ev.timestamp
        s.windows_seen += 1
        if ev.session is not None:
            s.session = ev.session

        # --- Packet-derived signals (§3) --------------------------------
        p = ev.packets_window
        out = p.get("out", 0)
        lost = p.get("lost", 0)
        fec_rec = p.get("fec_rec", 0)
        data = p.get("data", 0)
        bursts_rec = p.get("bursts_rec", 0)
        holdoff = p.get("holdoff", 0)
        late = p.get("late_deadline", 0)

        tx_primaries = out + lost  # TX-emitted primaries this window
        if tx_primaries > 0:
            s.residual_loss_w = lost / tx_primaries
            s.fec_work_rate_w = fec_rec / tx_primaries
        else:
            s.residual_loss_w = 0.0
            s.fec_work_rate_w = 0.0

        s.packet_rate_w = data / WINDOW_S
        s.burst_rate_w = bursts_rec / WINDOW_S
        s.holdoff_rate_w = holdoff / WINDOW_S
        s.late_rate_w = late / WINDOW_S

        # --- Antenna-derived signals (§3) ------------------------------
        if ev.rx_ant_stats:
            rssi_mins = [a.rssi_min for a in ev.rx_ant_stats]
            rssi_avgs = [a.rssi_avg for a in ev.rx_ant_stats]
            snr_mins = [a.snr_min for a in ev.rx_ant_stats]
            snr_avgs = [a.snr_avg for a in ev.rx_ant_stats]
            s.rssi_min_w = float(min(rssi_mins))
            s.rssi_avg_w = float(sum(rssi_avgs) / len(rssi_avgs))
            s.rssi_max_w = float(max(rssi_avgs))
            s.snr_min_w = float(min(snr_mins))
            s.snr_avg_w = float(sum(snr_avgs) / len(snr_avgs))
            s.snr_max_w = float(max(snr_avgs))
            s.ant_count = len(ev.rx_ant_stats)
        # If no antenna lines this window, keep prior values — don't
        # reset; the RSSI operating point doesn't vanish just because
        # no fragments arrived.

        # --- Starvation flag (post-blackout detection) -----------------
        # Only meaningful once we've seen a session — otherwise we'd
        # flag every pre-link tick. Bypasses the survivor-bias trap of
        # rssi/snr because it watches packet_rate, not signal quality.
        s.link_starved_w = (
            s.session is not None
            and s.packet_rate_w < self.starvation_threshold_pps
        )

        # --- EWMA smoothing (§3) ---------------------------------------
        # Smoothed inputs feed the leading loop. Use best-antenna
        # aggregations: max(rssi_avg) / max(snr_avg) match what the
        # diversity receiver actually decodes against (and what the OSD
        # shows). min(rssi_min) tracks the weakest antenna and misses
        # best-antenna degradation entirely.
        if s.rssi_max_w is not None:
            s.rssi = _ewma(s.rssi, s.rssi_max_w, self.ewma_alpha_rssi)
        if s.snr_max_w is not None:
            s.snr = _ewma(s.snr, s.snr_max_w, self.ewma_alpha_rssi)

        # SNR slope: EMA of Δs.snr per tick. First sample yields 0
        # (no prior to diff against). Used by the dual-gate selector
        # to predict whether margin will hold across the next horizon.
        if s.snr is not None:
            if self._prev_snr is None:
                s.snr_slope = 0.0
            else:
                delta = s.snr - self._prev_snr
                s.snr_slope = _ewma(
                    s.snr_slope, delta, self.ewma_alpha_snr_slope
                )
            self._prev_snr = s.snr

        s.fec_work = _ewma(s.fec_work, s.fec_work_rate_w, self.ewma_alpha_fec)
        s.burst_rate = _ewma(s.burst_rate, s.burst_rate_w, self.ewma_alpha_burst)
        s.holdoff_rate = _ewma(
            s.holdoff_rate, s.holdoff_rate_w, self.ewma_alpha_burst
        )
        s.late_rate = _ewma(s.late_rate, s.late_rate_w, self.ewma_alpha_burst)

        return s
