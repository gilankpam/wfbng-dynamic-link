"""Tests for the per-window signal aggregator + EWMA smoother (§3)."""
from __future__ import annotations

from dynamic_link.signals import SignalAggregator
from dynamic_link.stats_client import RxAnt, RxEvent, SessionInfo


def _rx(
    ts: float,
    *,
    out: int = 0,
    lost: int = 0,
    fec_rec: int = 0,
    data: int = 0,
    bursts_rec: int = 0,
    holdoff: int = 0,
    late_deadline: int = 0,
    ants: list[tuple[int, int, int, int]] | None = None,
) -> RxEvent:
    """Build a minimal RxEvent for tests. ants = [(rssi_min, rssi_avg, snr_min, snr_avg), ...]"""
    if ants is None:
        ants = [(-60, -58, 20, 22)]
    ant_stats = [
        RxAnt(
            ant=i, freq=5765, mcs=7, bw=20, pkt_recv=100,
            rssi_min=a[0], rssi_avg=a[1], rssi_max=a[1] + 2,
            snr_min=a[2], snr_avg=a[3], snr_max=a[3] + 2,
        )
        for i, a in enumerate(ants)
    ]
    session = SessionInfo(
        fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=1,
        interleave_depth=1, contract_version=2,
    )
    return RxEvent(
        timestamp=ts,
        id="rx1",
        packets_window={
            "out": out, "lost": lost, "fec_rec": fec_rec, "data": data,
            "bursts_rec": bursts_rec, "holdoff": holdoff,
            "late_deadline": late_deadline,
        },
        rx_ant_stats=ant_stats,
        session=session,
    )


def test_residual_loss_from_lost_over_tx_primaries():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, out=99, lost=1, data=150))
    # 1 lost out of 100 tx primaries = 0.01
    assert s.residual_loss_w == 0.01


def test_residual_loss_zero_on_empty_window():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, out=0, lost=0))
    assert s.residual_loss_w == 0.0
    assert s.fec_work_rate_w == 0.0


def test_fec_work_rate():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, out=90, lost=0, fec_rec=5))
    assert s.fec_work_rate_w == 5 / 90


def test_rssi_min_is_min_across_antennas():
    agg = SignalAggregator()
    # three antennas — weakest should be picked
    s = agg.consume(_rx(0.1, ants=[(-55, -55, 20, 20),
                                   (-72, -70, 15, 17),
                                   (-60, -58, 18, 20)]))
    assert s.rssi_min_w == -72.0
    # rssi_avg_w is diversity-combined (simple average of rssi_avg)
    expected_avg = (-55 + -70 + -58) / 3
    assert abs(s.rssi_avg_w - expected_avg) < 1e-9


def test_ewma_alpha_rssi_matches_config():
    agg = SignalAggregator(ewma_alpha_rssi=0.2)
    s = agg.consume(_rx(0.1, ants=[(-60, -60, 20, 20)]))
    # First window bootstraps prev=None → EWMA returns raw value
    assert s.rssi == -60.0
    # Next window at -80: 0.2*-80 + 0.8*-60 = -64
    s = agg.consume(_rx(0.2, ants=[(-80, -80, 10, 10)]))
    assert abs(s.rssi - (-64.0)) < 1e-9


def test_residual_loss_is_not_smoothed():
    """§3: residual_loss must fire on raw per-window value."""
    agg = SignalAggregator()
    # Zero loss → spike → zero loss. residual_loss_w must track raw.
    agg.consume(_rx(0.1, out=100, lost=0))
    s = agg.consume(_rx(0.2, out=90, lost=10))
    assert s.residual_loss_w == 10 / 100
    s = agg.consume(_rx(0.3, out=100, lost=0))
    assert s.residual_loss_w == 0.0


def test_packet_rate_from_data_over_window():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, data=140))
    # data fragments per second at 100 ms window
    assert abs(s.packet_rate_w - 1400.0) < 1e-9


def test_burst_holdoff_late_smoothed_with_alpha_burst():
    agg = SignalAggregator(ewma_alpha_burst=0.1)
    s = agg.consume(_rx(0.1, bursts_rec=0))
    assert s.burst_rate == 0.0
    s = agg.consume(_rx(0.2, bursts_rec=10))
    # 0.1 * 100 + 0.9 * 0 = 10.0 (100 events/s)
    assert abs(s.burst_rate - 10.0) < 1e-9


def test_rssi_max_is_max_of_avgs_across_antennas():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, ants=[(-55, -55, 20, 20),
                                   (-72, -70, 15, 17),
                                   (-60, -58, 18, 20)]))
    assert s.rssi_max_w == -55.0  # best antenna's avg


def test_snr_max_is_max_of_avgs_across_antennas():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, ants=[(-55, -55, 20, 22),
                                   (-72, -70, 15, 17),
                                   (-60, -58, 18, 30)]))
    assert s.snr_max_w == 30.0  # best antenna's snr_avg


def test_ewma_smoothes_rssi_max_not_min():
    """Smoothed s.rssi must track best-antenna avg (max-of-avgs), not
    the worst-antenna min — that was the survivor-bias bug."""
    agg = SignalAggregator(ewma_alpha_rssi=1.0)  # no smoothing
    s = agg.consume(_rx(0.1, ants=[(-55, -50, 25, 30),
                                   (-72, -70, 15, 17)]))
    assert s.rssi == -50.0   # max(rssi_avg) — the best antenna
    assert s.snr == 30.0     # max(snr_avg)


def test_link_starved_w_when_packet_rate_below_threshold():
    agg = SignalAggregator(starvation_threshold_pps=50.0)
    # Bootstrap a session.
    agg.consume(_rx(0.1, data=2000))
    # Now drop traffic. data=4 in 100 ms → packet_rate_w = 40 < 50 → starved.
    s = agg.consume(_rx(0.2, data=4))
    assert s.link_starved_w is True


def test_link_starved_false_when_no_session():
    """Pre-link windows must not flag starvation — only meaningful once
    we know the drone is supposed to be TXing."""
    agg = SignalAggregator(starvation_threshold_pps=50.0)
    # Build an RxEvent with no session.
    ev = _rx(0.1, data=0)
    ev.session = None
    s = agg.consume(ev)
    assert s.link_starved_w is False


def test_link_starved_false_when_packet_rate_high():
    agg = SignalAggregator(starvation_threshold_pps=50.0)
    agg.consume(_rx(0.1, data=2000))   # session bootstrapped
    s = agg.consume(_rx(0.2, data=200))  # 2000 pps - well above
    assert s.link_starved_w is False


def test_snr_slope_initialises_zero_on_first_window():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, ants=[(-55, -55, 25, 25)]))
    # First sample — no prior to diff against.
    assert s.snr_slope == 0.0


def test_snr_slope_tracks_per_tick_delta_with_alpha():
    agg = SignalAggregator(ewma_alpha_rssi=1.0,         # no smoothing on s.snr
                           ewma_alpha_snr_slope=1.0)    # no smoothing on slope
    # Two windows with snr 20 → 25. Δ = +5.
    s = agg.consume(_rx(0.1, ants=[(-50, -50, 20, 20)]))
    assert s.snr == 20.0
    s = agg.consume(_rx(0.2, ants=[(-50, -50, 25, 25)]))
    # alpha=1.0 → slope = delta = 5.0
    assert abs(s.snr_slope - 5.0) < 1e-9


def test_snr_slope_stable_under_constant_snr():
    agg = SignalAggregator(ewma_alpha_rssi=1.0,
                           ewma_alpha_snr_slope=0.5)
    # Repeated identical windows — slope should converge to 0.
    for _ in range(10):
        s = agg.consume(_rx(0.1, ants=[(-50, -50, 25, 25)]))
    assert abs(s.snr_slope) < 1e-9


def test_snr_slope_tracks_negative_trend():
    agg = SignalAggregator(ewma_alpha_rssi=1.0,
                           ewma_alpha_snr_slope=1.0)
    # Falling SNR: 30 → 25 → 20 → 15.
    agg.consume(_rx(0.1, ants=[(-50, -50, 30, 30)]))
    s = agg.consume(_rx(0.2, ants=[(-50, -50, 25, 25)]))
    assert s.snr_slope == -5.0
    s = agg.consume(_rx(0.3, ants=[(-50, -50, 20, 20)]))
    assert s.snr_slope == -5.0
