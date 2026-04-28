"""Tests for the RTP/H.265 video tap.

Most cases bypass the asyncio plumbing and exercise the Protocol
directly — each test feeds raw RTP datagrams into
``datagram_received`` and checks the FrameRecord that comes out the
other end. The bottom of the file has one end-to-end test driving
the public ``VideoTap`` against a real UDP socket.
"""
from __future__ import annotations

import asyncio
import io
import json
import socket
import struct
from contextlib import contextmanager

import pytest

from dynamic_link.video_tap import (
    FrameRecord,
    RTP_HEADER_SIZE,
    RTP_VIDEO_CLOCK_HZ,
    VideoRtpSink,
    VideoTap,
    _Protocol,
)


@contextmanager
def _ephemeral_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    yield port


def _rtp(seq: int, ts: int, ssrc: int, *, marker: bool = False,
         payload: bytes = b"\x00") -> bytes:
    """Build a minimal RFC-3550 RTP packet (V=2, no padding/extension/csrc,
    PT=96 (dynamic / first H.265 binding)."""
    b0 = 0x80                # V=2, P=0, X=0, CC=0
    b1 = 96 | (0x80 if marker else 0x00)
    return struct.pack(">BBHII", b0, b1, seq, ts, ssrc) + payload


def _capturing_protocol() -> tuple[_Protocol, list[FrameRecord]]:
    out: list[FrameRecord] = []
    return _Protocol(on_frame=out.append), out


def test_marker_bit_emits_one_record():
    proto, frames = _capturing_protocol()
    ssrc = 0x12345678
    proto.datagram_received(_rtp(100, 90_000, ssrc), None)
    proto.datagram_received(_rtp(101, 90_000, ssrc), None)
    proto.datagram_received(_rtp(102, 90_000, ssrc, marker=True), None)
    assert len(frames) == 1
    f = frames[0]
    assert f.rtp_seq_first == 100
    assert f.rtp_seq_last == 102
    assert f.packets == 3
    assert f.expected == 3
    assert f.lost_in_frame == 0
    assert f.ssrc == ssrc


def test_lost_packet_within_frame_counted():
    proto, frames = _capturing_protocol()
    ssrc = 1
    proto.datagram_received(_rtp(10, 90_000, ssrc), None)
    # seq 11 missing
    proto.datagram_received(_rtp(12, 90_000, ssrc), None)
    proto.datagram_received(_rtp(13, 90_000, ssrc, marker=True), None)
    assert len(frames) == 1
    f = frames[0]
    assert f.expected == 4   # 13 - 10 + 1
    assert f.packets == 3
    assert f.lost_in_frame == 1


def test_first_frame_drift_is_zero():
    """First frame seeds the reference; gs_elapsed and rtp_elapsed
    are both zero, so drift = 0 by construction."""
    proto, frames = _capturing_protocol()
    proto.datagram_received(_rtp(0, 1234, 1, marker=True), None)
    assert len(frames) == 1
    assert frames[0].latency_drift_us == 0


def test_frame_interarrival_zero_for_first_frame_then_real():
    proto, frames = _capturing_protocol()
    ssrc = 1
    proto.datagram_received(_rtp(0, 0, ssrc, marker=True), None)
    proto.datagram_received(_rtp(1, 90_000, ssrc, marker=True), None)
    assert frames[0].frame_interarrival_us == 0
    assert frames[1].frame_interarrival_us > 0


def test_ssrc_change_resets_drift_reference():
    proto, frames = _capturing_protocol()
    proto.datagram_received(_rtp(0, 0, 0xAAAA, marker=True), None)
    # Fast-forward RTP timestamp, but encoder restarts (new SSRC).
    proto.datagram_received(_rtp(0, 99_999_999, 0xBBBB, marker=True), None)
    # Both frames should report drift ≈ 0 because the second one
    # triggered a fresh reference. If we *hadn't* reset, the second
    # frame would report a wildly negative drift.
    assert abs(frames[1].latency_drift_us) < 1_000_000


def test_short_datagram_ignored():
    proto, frames = _capturing_protocol()
    proto.datagram_received(b"\x80\xe0\x00\x01", None)  # 4 bytes < header
    assert frames == []
    assert proto.stats["malformed"] == 1


def test_non_v2_datagram_ignored():
    proto, frames = _capturing_protocol()
    bad = bytearray(_rtp(0, 0, 1, marker=True))
    bad[0] = 0x40   # version 1
    proto.datagram_received(bytes(bad), None)
    assert frames == []
    assert proto.stats["malformed"] == 1


def test_reordered_packet_dropped():
    proto, frames = _capturing_protocol()
    ssrc = 1
    proto.datagram_received(_rtp(100, 0, ssrc), None)
    proto.datagram_received(_rtp(99, 0, ssrc), None)   # reorder
    proto.datagram_received(_rtp(101, 0, ssrc, marker=True), None)
    assert len(frames) == 1
    # Reordered packet shouldn't have been counted toward this frame.
    assert frames[0].packets == 2
    assert proto.stats["malformed"] == 1


def test_drift_grows_when_arrival_lags_encoder():
    """Simulate: encoder advances 1 s of RTP timestamp, but GS arrival
    only advances 0.5 s of monotonic time → drift goes negative
    (recv elapsed < encoder elapsed). Then if arrival catches up while
    encoder pauses, drift swings positive."""
    proto, frames = _capturing_protocol()
    ssrc = 1
    # Frame 0 — seeds reference at gs_mono_us = T0, rtp_ts = 0.
    proto.datagram_received(_rtp(0, 0, ssrc, marker=True), None)
    # Frame 1 — RTP advances 1.0 s (90000 ticks), but mono advances
    # only ~tens of microseconds (back-to-back call). Drift should be
    # negative ≈ −1 s in microseconds.
    proto.datagram_received(_rtp(1, 90_000, ssrc, marker=True), None)
    drift = frames[1].latency_drift_us
    # We can't predict the gs_mono delta exactly, but it'll be much
    # less than 1 s in this test loop.
    assert drift < -500_000


def test_rtp_timestamp_wrap_handled():
    """RTP timestamps wrap at 2^32. The (delta & 0xFFFFFFFF) mask
    handles this, but only if the wrap is forward — the relative
    drift over a single wrap should stay in a sane range."""
    proto, frames = _capturing_protocol()
    ssrc = 1
    proto.datagram_received(_rtp(0, 0xFFFFFFF0, ssrc, marker=True), None)
    proto.datagram_received(_rtp(1, 0x10, ssrc, marker=True), None)
    # Delta should be 0x20 (32 ticks) ≈ 355 microseconds.
    expected_rtp_us = 0x20 * 1_000_000 // RTP_VIDEO_CLOCK_HZ
    # Actual drift = gs_elapsed - rtp_elapsed. gs_elapsed ≈ tens of µs.
    assert frames[1].latency_drift_us > -expected_rtp_us - 1000
    assert frames[1].latency_drift_us < 0


def test_video_rtp_sink_writes_jsonl():
    buf = io.StringIO()
    sink = VideoRtpSink(buf)
    sink.write(FrameRecord(
        ts_gs_mono_us=12345, ts_gs_wall_us=99999,
        rtp_seq_first=1, rtp_seq_last=3, rtp_ts=42,
        ssrc=0xDEADBEEF, packets=3, expected=3,
        lost_in_frame=0, latency_drift_us=-200, frame_interarrival_us=16667,
    ))
    line = buf.getvalue().rstrip("\n")
    rec = json.loads(line)
    assert rec["ssrc"] == "0xdeadbeef"
    assert rec["packets"] == 3
    assert rec["latency_drift_us"] == -200
    assert sink.stats["written"] == 1


@pytest.mark.asyncio
async def test_video_tap_end_to_end_real_socket():
    """Drive the full async plumbing — bind the tap, send three real
    UDP packets, assert one frame record was received."""
    received: list[FrameRecord] = []

    with _ephemeral_port() as port:
        tap = VideoTap("127.0.0.1", port, on_frame=received.append)
        await tap.start()
        try:
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ssrc = 0x12345678
            sender.sendto(_rtp(0, 0, ssrc), ("127.0.0.1", port))
            sender.sendto(_rtp(1, 0, ssrc), ("127.0.0.1", port))
            sender.sendto(_rtp(2, 0, ssrc, marker=True), ("127.0.0.1", port))
            sender.close()
            # Give the loop a moment to drain the socket.
            for _ in range(20):
                if received:
                    break
                await asyncio.sleep(0.02)
            assert len(received) == 1
            f = received[0]
            assert f.packets == 3
            assert f.ssrc == ssrc
        finally:
            tap.stop()
