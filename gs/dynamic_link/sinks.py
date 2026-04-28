"""Decision sinks (Phase 0 = log only)."""
from __future__ import annotations

import json
import logging
import sys
from typing import Callable, TextIO, Union

from .decision import Decision

log = logging.getLogger(__name__)

StreamSource = Union[TextIO, Callable[[], "TextIO | None"], None]


def _resolve(src: StreamSource) -> "TextIO | None":
    if src is None:
        return None
    if callable(src):
        return src()
    return src


class LogSink:
    """Write each Decision as one-line JSON to up to two streams.

    `events_stream` receives only ticks where `knobs_changed` is
    non-empty — the operator-facing event log. `verbose_stream`
    receives every tick — a debug/replay log. Either or both may be
    a fixed TextIO, a callable returning a TextIO (or None to drop),
    or None to disable that channel entirely. The callable form is
    used by the per-flight rotator to swap streams between flights.

    `extras_provider`, when set, is called on each write and the
    returned dict is merged into the JSON output. Phase 3 uses this
    to stamp `offset_us` / `offset_stddev_us` from the timesync
    estimator onto every log line.

    JSON is serialised once per write, so dual output is cheap.
    """

    def __init__(
        self,
        events_stream: StreamSource = None,
        verbose_stream: StreamSource = None,
        extras_provider: Callable[[], dict] | None = None,
    ):
        self._events_stream = events_stream
        self._verbose_stream = verbose_stream
        self._extras_provider = extras_provider

    def write(self, decision: Decision) -> None:
        events = _resolve(self._events_stream)
        verbose = _resolve(self._verbose_stream)
        if events is None and verbose is None:
            return
        record = decision.to_dict()
        if self._extras_provider is not None:
            extras = self._extras_provider()
            if extras:
                record.update(extras)
        line = json.dumps(record, separators=(",", ":")) + "\n"
        if events is not None and decision.knobs_changed:
            events.write(line)
            events.flush()
        if verbose is not None:
            verbose.write(line)
            verbose.flush()

    def close(self) -> None:
        # Only close fixed-stream sources we own; getter-backed streams
        # are owned by the rotator that produced them.
        for src in (self._events_stream, self._verbose_stream):
            if src is None or callable(src):
                continue
            if src in (sys.stdout, sys.stderr):
                continue
            try:
                src.close()
            except Exception:
                pass
