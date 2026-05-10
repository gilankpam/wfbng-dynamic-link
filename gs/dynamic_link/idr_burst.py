"""IDR burst sender — adopts alink_gs's keyframe-pump idea.

When the policy emits a Decision with idr_request=True, the regular
per-tick send path puts one packet on the wire. This module schedules
N-1 additional copies at interval_ms spacing, decoupled from the
10 Hz stats cadence, so a single packet loss on the lossy return
link doesn't swallow the IDR request.

Wire format unchanged: each burst packet is a normal DLK1 decision
frame with FLAG_IDR_REQUEST set and a fresh sequence number (so the
drone applier doesn't dedup it). Drone-side
dl_backend_enc_request_idr throttles HTTP calls by wall clock, so
the N attempted bursts collapse to one camera API call as long as
min_idr_interval_ms > interval_ms * (count - 1).

Defaults match alink_gs.conf [keyframe]:
  enabled     = True   (allow_idr)
  count       = 4      (idr_max_messages)
  interval_ms = 20     (idr_send_interval_ms)
"""
from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass

from .decision import Decision
from .return_link import ReturnLink
from .wire import Encoder as WireEncoder

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class IdrBurstConfig:
    enabled: bool = True
    count: int = 4
    interval_ms: int = 20


class IdrBurster:
    def __init__(
        self,
        cfg: IdrBurstConfig,
        return_link: ReturnLink,
        encoder: WireEncoder,
    ) -> None:
        self._cfg = cfg
        self._return_link = return_link
        self._encoder = encoder
        self._latest_decision: Decision | None = None
        self._remaining = 0
        self._task: asyncio.Task | None = None

    def trigger(self, decision: Decision) -> None:
        if not self._cfg.enabled:
            return
        self._latest_decision = decision
        self._remaining = max(0, self._cfg.count - 1)
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._pump())

    async def _pump(self) -> None:
        while self._remaining > 0:
            await asyncio.sleep(self._cfg.interval_ms / 1000.0)
            if self._latest_decision is None or self._remaining <= 0:
                break
            packet = self._encoder.encode(
                self._latest_decision, idr_request=True,
            )
            self._return_link.send(packet)
            self._remaining -= 1
