"""Phase 3 — RTP/H.265 video tap.

Passive UDP listener on the wfb-ng video output port (default 5600).
Parses each datagram's 12-byte RTP fixed header (RFC 3550), tracks
per-packet sequence numbers and per-frame marker bits, and emits one
JSONL record per frame to `video_rtp.jsonl`.

The metric of interest is **latency drift**, not absolute latency.
RTP timestamps are encoder-relative and tick at the 90 kHz video
clock; with no drone-side clock reference we can't measure absolute
glass-to-glass latency from RTP alone, but we *can* measure how the
gap between (encoder-elapsed) and (GS-arrival-elapsed) evolves over
time. Under a steady link this gap is ~constant; a 200 ms goggle
freeze shows up as a 200 ms jump in `latency_drift_us` over a few
frames. That's the signal that's missing from the current log.

Reference state ``(rtp_ts_0, gs_mono_0)`` resets on SSRC change
(encoder restart) so a mid-flight encoder bounce doesn't poison the
drift estimate.

Coexistence with the goggle/recorder
------------------------------------
We bind with ``SO_REUSEPORT`` to share the video port with whatever
already consumes the stream. On Linux ≥ 3.9 this fans packets across
all bound sockets round-robin — which is exactly *not* what we want;
each socket sees ~half the traffic. In practice the tap still gets
useful samples (loss math is per-frame so missing every other packet
is just halved sampling density), but if the operator needs every
frame they should set ``debug.video_tap_port`` to a dedicated port
fed by an explicit upstream split. A future v2 may add a proxy mode.
"""
from __future__ import annotations

import asyncio
import logging
import socket
import struct
import time
from dataclasses import dataclass

log = logging.getLogger(__name__)

RTP_HEADER_SIZE = 12
RTP_VIDEO_CLOCK_HZ = 90_000


@dataclass(frozen=True)
class FrameRecord:
    """One end-of-frame observation, ready for the JSONL sink."""
    ts_gs_mono_us: int
    ts_gs_wall_us: int
    rtp_seq_first: int
    rtp_seq_last: int
    rtp_ts: int
    ssrc: int
    packets: int
    expected: int
    lost_in_frame: int
    latency_drift_us: int
    frame_interarrival_us: int


class VideoTap:
    """Bind one UDP socket on ``host:port`` and feed parsed frame
    records to ``on_frame``. Caller owns the sink (typically a JSONL
    writer) — the tap itself is sink-agnostic."""

    def __init__(self, host: str, port: int, on_frame) -> None:
        self.host = host
        self.port = port
        self._on_frame = on_frame
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        # Build the socket ourselves so we can set SO_REUSEPORT before
        # bind. asyncio.create_datagram_endpoint(reuse_port=...) only
        # exists on some Python versions; doing it by hand is cheaper.
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError as e:
                log.warning("video_tap: SO_REUSEPORT unsupported: %s", e)
        sock.setblocking(False)
        sock.bind((self.host, self.port))
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _Protocol(self._on_frame),
            sock=sock,
        )
        log.info("video_tap: bound %s:%d (SO_REUSEPORT)", self.host, self.port)

    def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
            self._transport = None


class _Protocol(asyncio.DatagramProtocol):
    """RTP-aware UDP protocol. Stateful: tracks reference timestamps,
    SSRC, sequence-number gaps, and the in-progress frame's packet
    accumulator."""

    def __init__(self, on_frame) -> None:
        self._on_frame = on_frame
        # Reference for drift math; (rtp_ts_0, gs_mono_us_0). None
        # before the first packet, and on SSRC change.
        self._ref_rtp_ts: int | None = None
        self._ref_gs_mono_us: int | None = None
        self._ssrc: int | None = None
        # Per-packet sequence tracking. Wraps at 65536.
        self._last_seq: int | None = None
        # In-progress frame accumulator. `lost_in_frame` is computed
        # at marker time as `expected - packets`; we don't keep a
        # running gap counter here (it would double-count losses that
        # span frame boundaries).
        self._frame_seq_first: int | None = None
        self._frame_seq_last: int | None = None
        self._frame_packets: int = 0
        # Last frame timestamp for inter-arrival.
        self._last_frame_gs_mono_us: int | None = None
        self._malformed_count: int = 0

    @property
    def stats(self) -> dict:
        return {
            "ssrc": self._ssrc,
            "last_seq": self._last_seq,
            "malformed": self._malformed_count,
        }

    def datagram_received(self, data: bytes, addr) -> None:
        # Stamp arrival time first — before the parser ever runs.
        gs_mono_us = time.monotonic_ns() // 1000

        if len(data) < RTP_HEADER_SIZE:
            self._malformed_count += 1
            return
        b0 = data[0]
        version = (b0 >> 6) & 0x3
        if version != 2:
            self._malformed_count += 1
            return
        b1 = data[1]
        marker = bool(b1 & 0x80)
        seq = struct.unpack_from(">H", data, 2)[0]
        rtp_ts = struct.unpack_from(">I", data, 4)[0]
        ssrc = struct.unpack_from(">I", data, 8)[0]

        # SSRC change → encoder restart. Drop the in-progress frame
        # and re-anchor the drift reference.
        if self._ssrc is not None and ssrc != self._ssrc:
            log.info(
                "video_tap: SSRC changed 0x%08x → 0x%08x; resetting "
                "drift reference",
                self._ssrc, ssrc,
            )
            self._reset_for_new_ssrc()

        if self._ssrc is None:
            self._ssrc = ssrc
        if self._ref_rtp_ts is None:
            self._ref_rtp_ts = rtp_ts
            self._ref_gs_mono_us = gs_mono_us

        # Reorder check (rare on Wi-Fi monitor mode, but possible).
        if self._last_seq is not None:
            delta = (seq - self._last_seq) & 0xFFFF
            if delta > 0x8000 and delta != 0:
                # Older-than-last_seq → drop. Frame loss is accounted
                # for at marker time via expected-vs-packets.
                self._malformed_count += 1
                return
        self._last_seq = seq

        # Frame accumulator.
        if self._frame_seq_first is None:
            self._frame_seq_first = seq
        self._frame_seq_last = seq
        self._frame_packets += 1

        if not marker:
            return

        # End-of-frame. Compute per-frame drift and emit.
        rtp_elapsed_us = (
            (rtp_ts - self._ref_rtp_ts) & 0xFFFFFFFF
        ) * 1_000_000 // RTP_VIDEO_CLOCK_HZ
        gs_elapsed_us = gs_mono_us - (self._ref_gs_mono_us or gs_mono_us)
        drift_us = gs_elapsed_us - rtp_elapsed_us

        interarrival_us = 0
        if self._last_frame_gs_mono_us is not None:
            interarrival_us = gs_mono_us - self._last_frame_gs_mono_us
        self._last_frame_gs_mono_us = gs_mono_us

        # Expected packet count: (last_seq - first_seq + 1) modulo 16-bit.
        first = self._frame_seq_first
        last = self._frame_seq_last
        expected = ((last - first) & 0xFFFF) + 1
        lost_in_frame = max(expected - self._frame_packets, 0)

        record = FrameRecord(
            ts_gs_mono_us=gs_mono_us,
            ts_gs_wall_us=int(time.time() * 1_000_000),
            rtp_seq_first=first,
            rtp_seq_last=last,
            rtp_ts=rtp_ts,
            ssrc=ssrc,
            packets=self._frame_packets,
            expected=expected,
            lost_in_frame=lost_in_frame,
            latency_drift_us=drift_us,
            frame_interarrival_us=interarrival_us,
        )
        try:
            self._on_frame(record)
        except Exception:  # pylint: disable=broad-except
            log.exception("video_tap: on_frame handler raised")

        # Reset frame accumulator.
        self._frame_seq_first = None
        self._frame_seq_last = None
        self._frame_packets = 0

    def _reset_for_new_ssrc(self) -> None:
        self._ref_rtp_ts = None
        self._ref_gs_mono_us = None
        self._frame_seq_first = None
        self._frame_seq_last = None
        self._frame_packets = 0
        self._last_seq = None
        self._last_frame_gs_mono_us = None
        self._ssrc = None

    def error_received(self, exc: Exception) -> None:
        log.warning("video_tap: %s", exc)


# ---- Sink ---------------------------------------------------------------

class VideoRtpSink:
    """JSONL writer for FrameRecord. Mirrors LatencySink's shape."""

    def __init__(self, stream) -> None:
        self._stream = stream
        self._written = 0

    def write(self, rec: FrameRecord) -> None:
        import json
        record = {
            "ts_gs_mono_us": rec.ts_gs_mono_us,
            "ts_gs_wall_us": rec.ts_gs_wall_us,
            "rtp_seq_first": rec.rtp_seq_first,
            "rtp_seq_last": rec.rtp_seq_last,
            "rtp_ts": rec.rtp_ts,
            "ssrc": f"0x{rec.ssrc:08x}",
            "packets": rec.packets,
            "expected": rec.expected,
            "lost_in_frame": rec.lost_in_frame,
            "latency_drift_us": rec.latency_drift_us,
            "frame_interarrival_us": rec.frame_interarrival_us,
        }
        self._stream.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._stream.flush()
        self._written += 1

    @property
    def stats(self) -> dict:
        return {"written": self._written}

    def close(self) -> None:
        import sys as _sys
        s = self._stream
        if s is None or s in (_sys.stdout, _sys.stderr):
            return
        try:
            s.close()
        except Exception:
            pass
