"""Decision sinks (Phase 0 = log only)."""
from __future__ import annotations

import json
import logging
import sys
from typing import Callable, TextIO

from .decision import Decision

log = logging.getLogger(__name__)


class LogSink:
    """Write each Decision as one-line JSON to up to two streams.

    `events_stream` receives only ticks where `knobs_changed` is
    non-empty — the operator-facing event log. `verbose_stream`
    receives every tick — a debug/replay log. Either or both may
    be set; when neither is set the sink is a no-op.

    `extras_provider`, when set, is called on each write and the
    returned dict is merged into the JSON output. Phase 3 uses this
    to stamp `offset_us` / `offset_stddev_us` from the timesync
    estimator onto every log line.

    JSON is serialised once per write, so dual output is cheap.
    """

    def __init__(
        self,
        events_stream: TextIO | None = None,
        verbose_stream: TextIO | None = None,
        extras_provider: Callable[[], dict] | None = None,
    ):
        self._events_stream = events_stream
        self._verbose_stream = verbose_stream
        self._extras_provider = extras_provider

    def write(self, decision: Decision) -> None:
        if self._events_stream is None and self._verbose_stream is None:
            return
        record = decision.to_dict()
        if self._extras_provider is not None:
            extras = self._extras_provider()
            if extras:
                record.update(extras)
        line = json.dumps(record, separators=(",", ":")) + "\n"
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
