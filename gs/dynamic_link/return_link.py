"""Phase 2 — UDP writer that puts encoded decision packets onto the
wfb-ng tunnel stream.

Nothing tunnel-specific in the code: wfb-ng's `tunnel` profile
terminates on a TUN interface (`gs-wfb` at 10.5.0.1 on the GS,
`drone-wfb` at 10.5.0.2 on the drone). We just `sendto()` to the
drone's TUN IP; kernel routing via the `gs-wfb` interface hands the
datagram to wfb_tx, which sends it over wifi.
"""
from __future__ import annotations

import logging
import socket

log = logging.getLogger(__name__)


class ReturnLink:
    """One non-blocking UDP socket aimed at the drone's dl-applier."""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._sent = 0
        self._errors = 0

    def send(self, packet: bytes) -> bool:
        """Send one packet. Returns True on success, False on any
        send error (logged at WARN). Never raises — the service
        loop must keep running even if the drone is temporarily
        unreachable (e.g. tunnel not yet up)."""
        try:
            self._sock.sendto(packet, (self.host, self.port))
            self._sent += 1
            return True
        except OSError as e:
            self._errors += 1
            # Log at warn for transient errors; don't spam if it's
            # sustained (tunnel down) — first error at warn, rest at
            # debug so journalctl stays readable.
            if self._errors == 1 or self._errors % 100 == 0:
                log.warning(
                    "return_link: sendto %s:%d: %s (%d errors total)",
                    self.host, self.port, e, self._errors,
                )
            else:
                log.debug("return_link: sendto: %s", e)
            return False

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:
            pass

    @property
    def stats(self) -> dict:
        return {"sent": self._sent, "errors": self._errors}
