"""Tests for the GS MAVLink status reader.

Hand-crafts a MAVLink v1 STATUSTEXT frame (matching what the drone
applier would send), feeds it into the reader, and asserts the
parsed event.
"""
from __future__ import annotations

import asyncio
import socket

import pytest

from dynamic_link.mavlink_status import MAVLinkStatusReader


def _x25_step(crc: int, b: int) -> int:
    tmp = b ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def _build_statustext(
    seq: int, sysid: int, compid: int, severity: int, text: str
) -> bytes:
    """Build a MAVLink v1 STATUSTEXT frame (msgid 253, CRC extra 83)."""
    text_bytes = text.encode("utf-8")[:50]
    payload = bytes([severity]) + text_bytes + bytes(50 - len(text_bytes))
    header = bytes([0xFE, len(payload), seq, sysid, compid, 253])
    frame_body = header[1:] + payload  # CRC covers LEN onward
    crc = 0xFFFF
    for b in frame_body:
        crc = _x25_step(crc, b)
    crc = _x25_step(crc, 83)  # STATUSTEXT extra byte
    return header + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


async def _run_reader_scenario(*statustexts):
    """Start a reader on an ephemeral port, send packets, collect events."""
    got: list = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()

    reader = MAVLinkStatusReader("127.0.0.1", port, on_event=got.append)
    await reader.start()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for frame in statustexts:
            s.sendto(frame, ("127.0.0.1", port))
        s.close()
        # Give the asyncio loop time to dispatch.
        for _ in range(50):
            await asyncio.sleep(0.01)
            if got:
                break
    finally:
        reader.stop()
    return got, reader.stats


async def test_statustext_with_dl_prefix_surfaces_event():
    frame = _build_statustext(
        seq=0, sysid=250, compid=191,
        severity=4, text="DL REJECT mcs_too_high",
    )
    got, stats = await _run_reader_scenario(frame)
    assert len(got) == 1
    ev = got[0]
    assert ev.severity == 4
    assert ev.text == "DL REJECT mcs_too_high"
    assert ev.raw_text == "REJECT mcs_too_high"
    assert ev.sysid == 250
    assert ev.compid == 191
    assert stats["statustexts"] == 1
    assert stats["dl_events"] == 1


async def test_statustext_without_dl_prefix_is_ignored():
    """Flight-controller STATUSTEXTs shouldn't trigger our callback."""
    frame = _build_statustext(
        seq=1, sysid=1, compid=1,
        severity=6, text="ArduPilot: EKF2 primary changed",
    )
    got, stats = await _run_reader_scenario(frame)
    assert got == []
    assert stats["statustexts"] == 1
    assert stats["dl_events"] == 0


async def test_multiple_statustexts_in_one_datagram():
    f1 = _build_statustext(0, 250, 191, 3, "DL WATCHDOG safe_defaults")
    f2 = _build_statustext(1, 1, 1, 6, "ArduPilot: normal message")
    f3 = _build_statustext(2, 250, 191, 4, "DL REJECT n_too_large")
    # Concat all three into one UDP datagram — wfb-ng aggregates.
    got, stats = await _run_reader_scenario(f1 + f2 + f3)
    assert len(got) == 2
    assert got[0].raw_text == "WATCHDOG safe_defaults"
    assert got[1].raw_text == "REJECT n_too_large"
    assert stats["statustexts"] == 3


async def test_bad_frame_does_not_crash():
    # Garbage bytes before a valid frame — parser must recover.
    valid = _build_statustext(0, 250, 191, 4, "DL REJECT foo")
    got, _ = await _run_reader_scenario(b"\x00\x01\x02\x03" + valid)
    assert len(got) == 1
    assert got[0].raw_text == "REJECT foo"
