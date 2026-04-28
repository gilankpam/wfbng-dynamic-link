"""Tests for the GS-side tunnel UDP listener."""
from __future__ import annotations

import asyncio
import socket
from contextlib import contextmanager

import pytest

from dynamic_link import wire
from dynamic_link.tunnel_listener import TunnelListener


@contextmanager
def _ephemeral_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    yield port


@pytest.mark.asyncio
async def test_listener_dispatches_pong():
    received: list = []

    def on_pong(p: wire.Pong, t4_us: int) -> None:
        received.append((p, t4_us))

    with _ephemeral_port() as port:
        listener = TunnelListener("127.0.0.1", port, on_pong=on_pong)
        await listener.start()
        try:
            pkt = wire.encode_pong(wire.Pong(
                gs_seq=99,
                gs_mono_us_echo=1_000_000,
                drone_mono_recv_us=2_000_000,
                drone_mono_send_us=2_000_050,
            ))
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sender.sendto(pkt, ("127.0.0.1", port))
            sender.close()
            await asyncio.sleep(0.05)
            assert len(received) == 1
            pong, t4 = received[0]
            assert pong.gs_seq == 99
            assert pong.gs_mono_us_echo == 1_000_000
            assert t4 > 0
        finally:
            listener.stop()


@pytest.mark.asyncio
async def test_listener_ignores_unknown_magic():
    received: list = []

    def on_pong(p: wire.Pong, t4_us: int) -> None:
        received.append(p)

    with _ephemeral_port() as port:
        listener = TunnelListener("127.0.0.1", port, on_pong=on_pong)
        await listener.start()
        try:
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sender.sendto(b"\xDE\xAD\xBE\xEF garbage", ("127.0.0.1", port))
            sender.close()
            await asyncio.sleep(0.05)
            assert received == []
        finally:
            listener.stop()


@pytest.mark.asyncio
async def test_listener_drops_corrupt_pong():
    received: list = []

    def on_pong(p: wire.Pong, t4_us: int) -> None:
        received.append(p)

    with _ephemeral_port() as port:
        listener = TunnelListener("127.0.0.1", port, on_pong=on_pong)
        await listener.start()
        try:
            pkt = bytearray(wire.encode_pong(wire.Pong(
                gs_seq=1, gs_mono_us_echo=1,
                drone_mono_recv_us=2, drone_mono_send_us=3,
            )))
            pkt[10] ^= 0xFF  # corrupt a payload byte → CRC fails
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sender.sendto(bytes(pkt), ("127.0.0.1", port))
            sender.close()
            await asyncio.sleep(0.05)
            assert received == []
        finally:
            listener.stop()
