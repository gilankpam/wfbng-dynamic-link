"""Decision sinks (Phase 0 = log only)."""
from __future__ import annotations

import json
import logging
import sys
from typing import TextIO

from .decision import Decision

log = logging.getLogger(__name__)


class LogSink:
    """Write each Decision as one-line JSON to up to two streams.

    `events_stream` receives only ticks where `knobs_changed` is
    non-empty — the operator-facing event log. `verbose_stream`
    receives every tick — a debug/replay log. Either or both may
    be set; when neither is set the sink is a no-op.

    JSON is serialised once per write, so dual output is cheap.
    """

    def __init__(
        self,
        events_stream: TextIO | None = None,
        verbose_stream: TextIO | None = None,
    ):
        self._events_stream = events_stream
        self._verbose_stream = verbose_stream

    def write(self, decision: Decision) -> None:
        if self._events_stream is None and self._verbose_stream is None:
            return
        line = json.dumps(decision.to_dict(), separators=(",", ":")) + "\n"
        if self._events_stream is not None and decision.knobs_changed:
            self._events_stream.write(line)
            self._events_stream.flush()
        if self._verbose_stream is not None:
            self._verbose_stream.write(line)
            self._verbose_stream.flush()

    def close(self) -> None:
        for s in (self._events_stream, self._verbose_stream):
            if s is None or s in (sys.stdout, sys.stderr):
                continue
            try:
                s.close()
            except Exception:
                pass
