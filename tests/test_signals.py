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
