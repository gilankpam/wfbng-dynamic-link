"""Tests for the per-flight directory rotator."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from dynamic_link.flight_log import (
    EVENTS_FILE,
    LATENCY_FILE,
    MANIFEST_FILE,
    VERBOSE_FILE,
    VIDEO_RTP_FILE,
    FlightDirRotator,
)
from dynamic_link.stats_client import RxAnt, RxEvent, SessionInfo


def _rx_with_ants(ts: float, *, epoch: int = 1) -> RxEvent:
    ants = [
        RxAnt(
            ant=0, freq=5765, mcs=7, bw=20, pkt_recv=100,
            rssi_min=-60, rssi_avg=-58, rssi_max=-56,
            snr_min=20, snr_avg=22, snr_max=24,
        ),
    ]
    return RxEvent(
        timestamp=ts, id="rx1",
        packets_window={"data": 100},
        rx_ant_stats=ants,
        session=SessionInfo(
            fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=epoch,
            interleave_depth=1, contract_version=2,
        ),
    )


def _rx_empty(ts: float) -> RxEvent:
    return RxEvent(
        timestamp=ts, id="rx1",
        packets_window={"data": 0},
        rx_ant_stats=[],
        session=None,
    )


# ----------------------------------------------------------------------
# Open / files / getters
# ----------------------------------------------------------------------

def test_first_non_empty_rx_opens_flight_dir(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=10.0)
    assert r.events_stream() is None
    assert r.verbose_stream() is None

    r.on_rx_event(_rx_with_ants(0.1))

    flight = tmp_path / "flight-0001"
    assert flight.is_dir()
    for name in (EVENTS_FILE, VERBOSE_FILE, LATENCY_FILE, VIDEO_RTP_FILE):
        assert (flight / name).exists()

    assert r.events_stream() is not None
    assert r.verbose_stream() is not None
    assert r.latency_stream() is not None
    assert r.video_rtp_stream() is not None
    assert r.current_dir == flight

    r.close()


def test_idle_state_returns_none_streams(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=1.0)
    # No events at all → still idle.
    assert r.events_stream() is None
    assert r.latency_stream() is None
    assert r.video_rtp_stream() is None
    r.close()


# ----------------------------------------------------------------------
# Gap / close behavior
# ----------------------------------------------------------------------

def test_short_empty_streak_does_not_rotate(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=10.0)
    r.on_rx_event(_rx_with_ants(0.1))
    # 5 s of empty rx — under threshold.
    r.on_rx_event(_rx_empty(1.0))
    r.on_rx_event(_rx_empty(3.0))
    r.on_rx_event(_rx_empty(6.0))
    assert r.current_dir == tmp_path / "flight-0001"
    assert r.events_stream() is not None
    r.close()


def test_long_empty_streak_closes_and_writes_manifest(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=2.0)
    r.on_rx_event(_rx_with_ants(0.1, epoch=42))
    flight = tmp_path / "flight-0001"

    # First empty marks the start of the streak (t=1.0). Threshold is
    # 2.0 s, so an event at t=3.5 is the first one past the deadline.
    r.on_rx_event(_rx_empty(1.0))
    assert r.current_dir == flight  # still open
    r.on_rx_event(_rx_empty(3.5))

    assert r.current_dir is None
    assert r.events_stream() is None
    assert r.latency_stream() is None

    manifest = flight / MANIFEST_FILE
    assert manifest.exists()
    m = json.loads(manifest.read_text())
    assert m["dir"] == "flight-0001"
    assert m["reason"] == "gap"
    assert m["session_epoch"] == 42
    assert m["start_event_ts"] == pytest.approx(0.1)
    assert m["start_wall_us"] is not None
    assert m["stop_wall_us"] >= m["start_wall_us"]


def test_reconnect_opens_next_numbered_dir(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=2.0)
    r.on_rx_event(_rx_with_ants(0.1))
    r.on_rx_event(_rx_empty(1.0))
    r.on_rx_event(_rx_empty(3.5))  # close
    assert r.current_dir is None

    # Reconnect.
    r.on_rx_event(_rx_with_ants(10.0))
    assert r.current_dir == tmp_path / "flight-0002"
    assert (tmp_path / "flight-0002" / EVENTS_FILE).exists()
    r.close()


def test_intermittent_empty_resets_streak(tmp_path: Path):
    """A stray non-empty rx in the middle of a quiet period must reset
    the streak — otherwise frequent micro-fades would still tick the
    accumulated quiet time and force a close."""
    r = FlightDirRotator(tmp_path, gap_seconds=2.0)
    r.on_rx_event(_rx_with_ants(0.1))
    r.on_rx_event(_rx_empty(1.0))
    r.on_rx_event(_rx_with_ants(2.0))   # streak resets here
    r.on_rx_event(_rx_empty(3.0))
    r.on_rx_event(_rx_empty(4.5))       # 1.5 s into THIS streak — under
    assert r.current_dir == tmp_path / "flight-0001"
    r.close()


# ----------------------------------------------------------------------
# Counter resume
# ----------------------------------------------------------------------

def test_counter_resumes_from_existing_dirs(tmp_path: Path):
    (tmp_path / "flight-0007").mkdir()
    (tmp_path / "flight-0003").mkdir()
    r = FlightDirRotator(tmp_path, gap_seconds=10.0)
    r.on_rx_event(_rx_with_ants(0.1))
    assert r.current_dir == tmp_path / "flight-0008"
    r.close()


def test_counter_resume_ignores_malformed_entries(tmp_path: Path):
    (tmp_path / "flight-0002").mkdir()
    (tmp_path / "flight-foo").mkdir()
    (tmp_path / "flight-0009-rejected").mkdir()
    (tmp_path / "flight-0099").touch()  # file, not dir → ignored
    (tmp_path / "unrelated").mkdir()
    r = FlightDirRotator(tmp_path, gap_seconds=10.0)
    r.on_rx_event(_rx_with_ants(0.1))
    assert r.current_dir == tmp_path / "flight-0003"
    r.close()


def test_counter_starts_at_one_when_dir_empty(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=10.0)
    r.on_rx_event(_rx_with_ants(0.1))
    assert r.current_dir == tmp_path / "flight-0001"
    r.close()


# ----------------------------------------------------------------------
# Shutdown manifest
# ----------------------------------------------------------------------

def test_close_during_flight_writes_shutdown_manifest(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=10.0)
    r.on_rx_event(_rx_with_ants(0.1))
    flight = tmp_path / "flight-0001"
    r.close()
    m = json.loads((flight / MANIFEST_FILE).read_text())
    assert m["reason"] == "shutdown"


def test_close_when_idle_is_noop(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=10.0)
    r.close()  # must not raise
    assert list(tmp_path.iterdir()) == []


# ----------------------------------------------------------------------
# Stream behavior across rotation
# ----------------------------------------------------------------------

def test_writes_during_flight_land_in_flight_dir(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=10.0)
    r.on_rx_event(_rx_with_ants(0.1))
    s = r.events_stream()
    assert s is not None
    s.write('{"hello": 1}\n')
    s.flush()
    contents = (tmp_path / "flight-0001" / EVENTS_FILE).read_text()
    assert '"hello"' in contents
    r.close()


def test_separate_flights_get_separate_files(tmp_path: Path):
    r = FlightDirRotator(tmp_path, gap_seconds=2.0)
    r.on_rx_event(_rx_with_ants(0.1))
    r.events_stream().write('{"f": 1}\n')
    r.on_rx_event(_rx_empty(1.0))
    r.on_rx_event(_rx_empty(3.5))  # close

    r.on_rx_event(_rx_with_ants(10.0))
    r.events_stream().write('{"f": 2}\n')
    r.close()

    f1 = (tmp_path / "flight-0001" / EVENTS_FILE).read_text()
    f2 = (tmp_path / "flight-0002" / EVENTS_FILE).read_text()
    assert '"f": 1' in f1 and '"f": 2' not in f1
    assert '"f": 2' in f2 and '"f": 1' not in f2


# ----------------------------------------------------------------------
# Integration with LogSink (the coupling actually used by service.py)
# ----------------------------------------------------------------------

def test_logsink_rotates_with_rotator(tmp_path: Path):
    """LogSink + rotator together: the same sink instance writes into
    flight-0001 before the gap and flight-0002 after, with no writes
    between flights even though Decisions are still being produced."""
    from dynamic_link.decision import Decision
    from dynamic_link.sinks import LogSink

    r = FlightDirRotator(tmp_path, gap_seconds=2.0)
    sink = LogSink(
        events_stream=r.events_stream,
        verbose_stream=r.verbose_stream,
    )

    def _decision(tag: int, *, changed: bool) -> Decision:
        return Decision(
            timestamp=0.0,
            mcs=5, bandwidth=20, tx_power_dBm=18,
            k=8, n=14, depth=2, bitrate_kbps=12000,
            idr_request=False,
            reason=f"test-{tag}",
            knobs_changed=["mcs"] if changed else [],
            signals_snapshot={"tag": tag},
        )

    # Flight 1.
    r.on_rx_event(_rx_with_ants(0.1))
    sink.write(_decision(1, changed=True))   # → events + verbose
    sink.write(_decision(2, changed=False))  # → verbose only

    # Quiet streak triggers close.
    r.on_rx_event(_rx_empty(1.0))
    r.on_rx_event(_rx_empty(3.5))
    # Between flights — these writes must vanish.
    sink.write(_decision(99, changed=True))

    # Flight 2.
    r.on_rx_event(_rx_with_ants(10.0))
    sink.write(_decision(3, changed=True))
    r.close()

    f1_events = (tmp_path / "flight-0001" / EVENTS_FILE).read_text()
    f1_verbose = (tmp_path / "flight-0001" / VERBOSE_FILE).read_text()
    f2_events = (tmp_path / "flight-0002" / EVENTS_FILE).read_text()
    f2_verbose = (tmp_path / "flight-0002" / VERBOSE_FILE).read_text()

    # Flight 1: events has only the changed decision; verbose has both.
    assert "test-1" in f1_events and "test-2" not in f1_events
    assert "test-1" in f1_verbose and "test-2" in f1_verbose

    # Between-flights write was dropped — never appears anywhere.
    assert "test-99" not in f1_events and "test-99" not in f1_verbose
    assert "test-99" not in f2_events and "test-99" not in f2_verbose

    # Flight 2 carries only its own decision.
    assert "test-3" in f2_events and "test-3" in f2_verbose
    assert "test-1" not in f2_events
