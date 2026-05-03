"""Tests for derive_observed."""
from __future__ import annotations

from dynamic_link.observed import derive_observed
from dynamic_link.stats_client import RxAnt, RxEvent, SessionInfo


def _ant(*, mcs: int = 7, ant: int = 0) -> RxAnt:
    return RxAnt(
        ant=ant, freq=5765, mcs=mcs, bw=20, pkt_recv=100,
        rssi_min=-60, rssi_avg=-58, rssi_max=-56,
        snr_min=20, snr_avg=22, snr_max=24,
    )


def _session(k: int = 8, n: int = 12, depth: int = 1) -> SessionInfo:
    return SessionInfo(
        fec_type="VDM_RS", fec_k=k, fec_n=n, epoch=1,
        interleave_depth=depth, contract_version=2,
    )


def _ev(*, ants=None, session=None, packets=None) -> RxEvent:
    return RxEvent(
        timestamp=1.0,
        id="rx1",
        packets_window=packets or {},
        rx_ant_stats=ants or [],
        session=session,
    )


def test_derive_observed_none_returns_empty():
    assert derive_observed(None) == {}


def test_derive_observed_empty_event_returns_empty():
    assert derive_observed(_ev()) == {}


def test_derive_observed_single_antenna_mcs():
    out = derive_observed(_ev(ants=[_ant(mcs=4)]))
    assert out["mcs"] == 4


def test_derive_observed_dual_antenna_takes_max():
    out = derive_observed(_ev(ants=[_ant(mcs=3, ant=0), _ant(mcs=5, ant=1)]))
    assert out["mcs"] == 5


def test_derive_observed_session_fields():
    out = derive_observed(_ev(session=_session(k=6, n=9, depth=2)))
    assert out["fec_k"] == 6
    assert out["fec_n"] == 9
    assert out["interleave_depth"] == 2


def test_derive_observed_missing_session_omits_fec_keys():
    out = derive_observed(_ev(ants=[_ant()]))
    assert "fec_k" not in out
    assert "fec_n" not in out
    assert "interleave_depth" not in out


def test_derive_observed_bitrate_kbps_from_out_bytes():
    # 100 ms window, 100_000 bytes -> 800 kB/s = 8000 kbps
    out = derive_observed(_ev(packets={"out_bytes": 100_000, "out": 50}))
    assert out["bitrate_kbps"] == 8000
    assert out["packet_rate_pps"] == 500


def test_derive_observed_zero_byte_window():
    out = derive_observed(_ev(packets={"out_bytes": 0, "out": 0}))
    assert out["bitrate_kbps"] == 0
    assert out["packet_rate_pps"] == 0


def test_derive_observed_typical_full_event():
    ev = _ev(
        ants=[_ant(mcs=5, ant=0), _ant(mcs=5, ant=1)],
        session=_session(k=8, n=10, depth=1),
        packets={"out_bytes": 200_000, "out": 1000},
    )
    out = derive_observed(ev)
    assert out == {
        "mcs": 5,
        "fec_k": 8,
        "fec_n": 10,
        "interleave_depth": 1,
        "bitrate_kbps": 16000,
        "packet_rate_pps": 10000,
    }


def test_derive_observed_partial_packets_window():
    # Only out_bytes present, no `out` -> packet_rate_pps key omitted.
    out = derive_observed(_ev(packets={"out_bytes": 50_000}))
    assert out["bitrate_kbps"] == 4000
    assert "packet_rate_pps" not in out
