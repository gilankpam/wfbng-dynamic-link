"""Phase 3 — latency.jsonl writer.

One JSON line per PONG observation. The fields are the raw time-sync
state at that sample: GS-side recv timestamp, RTT, drone-side T2/T3,
the smoothed offset estimate, and an outlier flag (samples flagged as
outliers are still written so the operator can see them in context —
they just didn't move the offset estimate).
"""
from __future__ import annotations

import json
import logging
import sys
import time
from typing import TextIO

from .timesync import LatencySample

log = logging.getLogger(__name__)


class LatencySink:
    def __init__(self, stream: TextIO):
        self._stream = stream
        self._written = 0

    def write(self, sample: LatencySample) -> None:
        record = {
            "ts_gs_mono_us": sample.ts_gs_mono_us,
            "ts_gs_wall_us": int(time.time() * 1_000_000),
            "gs_seq": sample.gs_seq,
            "rtt_us": sample.rtt_us,
            "drone_mono_recv_us": sample.drone_mono_recv_us,
            "drone_mono_send_us": sample.drone_mono_send_us,
            "offset_us": sample.offset_us,
            "offset_stddev_us": sample.offset_stddev_us,
            "outlier": sample.outlier,
        }
        self._stream.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._stream.flush()
        self._written += 1

    @property
    def stats(self) -> dict:
        return {"written": self._written}

    def close(self) -> None:
        s = self._stream
        if s is None or s in (sys.stdout, sys.stderr):
            return
        try:
            s.close()
        except Exception:
            pass
