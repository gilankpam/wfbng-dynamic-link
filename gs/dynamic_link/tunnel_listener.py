"""Phase 3 — GS-side UDP listener for drone→GS tunnel traffic.

Mirror of `dl_applier.c`'s decision-listen socket, but in the reverse
direction: the drone sends PONG (and any future debug back-channel
packet) to ``10.5.0.1:<gs_tunnel_port>``; this listener receives it,
peeks the magic, and dispatches.

For now only PONG is dispatched, but the design accepts new magics
without restructuring (peek_kind drives a small registry).
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Callable

from . import wire

log = logging.getLogger(__name__)

PongHandler = Callable[[wire.Pong, int], None]
"""(pong, gs_mono_us_t4_recv) -> None"""


class _Protocol(asyncio.DatagramProtocol):
    def __init__(self, on_pong: PongHandler) -> None:
        self._on_pong = on_pong
        self._unknown_count = 0

    def datagram_received(self, data: bytes, addr) -> None:
        # Stamp arrival time as early as we possibly can — before any
        # parsing — so the offset estimate doesn't soak parser jitter.
        t4 = time.monotonic_ns() // 1000
        kind = wire.peek_kind(data)
        if kind == "pong":
            try:
                pong = wire.decode_pong(data)
            except ValueError as e:
                log.warning("tunnel_listener: bad pong from %s: %s", addr, e)
                return
            self._on_pong(pong, t4)
            return
        if kind == "unknown":
            self._unknown_count += 1
            if self._unknown_count <= 3 or self._unknown_count % 100 == 0:
                log.debug("tunnel_listener: unknown-magic packet from %s "
                          "(%d total)", addr, self._unknown_count)
            return
        # decision/ping packets aren't expected on this socket
        log.debug("tunnel_listener: unexpected %s packet from %s", kind, addr)

    def error_received(self, exc: Exception) -> None:
        log.warning("tunnel_listener: %s", exc)


class TunnelListener:
    def __init__(self, host: str, port: int, on_pong: PongHandler) -> None:
        self.host = host
        self.port = port
        self._on_pong = on_pong
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _Protocol(self._on_pong),
            local_addr=(self.host, self.port),
            allow_broadcast=False,
        )
        log.info("tunnel_listener: bound %s:%d", self.host, self.port)

    def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
            self._transport = None
