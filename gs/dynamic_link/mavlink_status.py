"""Phase 2 — MAVLink status-channel reader (drone → GS).

Reuses wfb-ng's bundled MAVLink codec (`wfb_ng.mavlink` /
`wfb_ng.mavlink_protocol`). Operators already have wfb-ng installed
to run the radio link; our GS service piggybacks on its codec rather
than adding a second MAVLink library.

Listens on a UDP socket (default `127.0.0.1:14550`, which is where
wfb-ng's `[gs_mavlink]` profile delivers decapsulated drone packets),
parses STATUSTEXT frames, and surfaces any whose text begins with
`DL ` as `DroneStatusEvent`s (the prefix convention the drone
applier uses — see `drone/src/dl_applier.c` reject/watchdog/apply_fail
call sites).

The reader doesn't block on wfb-ng being up: if nothing ever sends
to our port, we just sit quietly. All events land in a caller-provided
callback and in the service log.
"""
from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Any, Callable

log = logging.getLogger(__name__)


# Lazy import so `import mavlink_status` on a host without wfb-ng
# produces a clear error at construction time, not at import time
# (so tests can mock / fake without needing wfb-ng installed).
def _wfb_bits():
    from wfb_ng.mavlink import MAVLINK_MSG_ID_STATUSTEXT, mavlink_map  # noqa: F401
    from wfb_ng.mavlink_protocol import (
        mavlink_parser_gen,
        unpack_mavlink,
    )
    return MAVLINK_MSG_ID_STATUSTEXT, mavlink_parser_gen, unpack_mavlink


@dataclass
class DroneStatusEvent:
    """One STATUSTEXT from the drone applier (prefix `DL `)."""
    severity: int
    text: str
    raw_text: str          # bytes-decoded, un-prefixed
    sysid: int
    compid: int
    seq: int


# MAVLink severity levels
SEV_NAME = {
    0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR",
    4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG",
}


class MAVLinkStatusReader:
    """asyncio UDP listener that parses STATUSTEXT and fires a callback.

    Usage:
        reader = MAVLinkStatusReader("127.0.0.1", 14550,
                                     on_event=lambda ev: ...)
        await reader.start()
        # ... runs in background; later:
        reader.stop()
    """

    def __init__(
        self,
        addr: str,
        port: int,
        on_event: Callable[[DroneStatusEvent], Any],
    ) -> None:
        self.addr = addr
        self.port = port
        self._on_event = on_event
        self._transport: asyncio.DatagramTransport | None = None
        (
            self._statustext_msg_id,
            parser_gen_factory,
            self._unpack,
        ) = _wfb_bits()
        # The wfb-ng parser is a generator: prime with .send(None),
        # then feed bytes via .send(data) and get back list of
        # ((seq, sysid, compid, msg_id), payload) tuples.
        self._parser = parser_gen_factory(parse_l2=True)
        next(self._parser)  # prime
        self._stats = {"frames": 0, "statustexts": 0, "dl_events": 0}

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _Proto(self._on_datagram),
            local_addr=(self.addr, self.port),
        )
        log.info("mavlink_status: listening on %s:%d", self.addr, self.port)

    def stop(self) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:
                pass
            self._transport = None

    @property
    def stats(self) -> dict:
        return dict(self._stats)

    # ---- internal ----

    def _on_datagram(self, data: bytes) -> None:
        try:
            frames = self._parser.send(data)
        except Exception:
            log.exception("mavlink_status: parser error")
            return
        for frame in frames:
            (seq, sysid, compid, msg_id), payload = frame
            self._stats["frames"] += 1
            if msg_id != self._statustext_msg_id:
                continue
            self._stats["statustexts"] += 1
            try:
                _name, fmap = self._unpack(msg_id, payload)
            except Exception:
                log.exception("mavlink_status: unpack_mavlink failed")
                continue
            severity = int(fmap.get("severity", 0))
            raw = fmap.get("text", b"")
            if isinstance(raw, (bytes, bytearray)):
                text = raw.decode("utf-8", errors="replace")
            else:
                text = str(raw)
            if text.startswith("DL "):
                ev = DroneStatusEvent(
                    severity=severity,
                    text=text,
                    raw_text=text[3:],
                    sysid=int(sysid),
                    compid=int(compid),
                    seq=int(seq),
                )
                self._stats["dl_events"] += 1
                log.info(
                    "drone status [%s] sysid=%d compid=%d: %s",
                    SEV_NAME.get(severity, str(severity)),
                    ev.sysid, ev.compid, text,
                )
                try:
                    self._on_event(ev)
                except Exception:
                    log.exception("mavlink_status: on_event callback raised")


class _Proto(asyncio.DatagramProtocol):
    def __init__(self, on_datagram):
        self._on = on_datagram

    def datagram_received(self, data, addr):
        self._on(data)
