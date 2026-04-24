"""Decision sinks (Phase 0 = log only)."""
from __future__ import annotations

import json
import logging
import sys
from typing import TextIO

from .decision import Decision

log = logging.getLogger(__name__)


class LogSink:
    """Write each Decision to a file (or stdout) as one-line JSON.

    `verbose=False` (default) emits only ticks where `knobs_changed`
    is non-empty — steady-state ticks are suppressed. `verbose=True`
    emits every tick.
    """

    def __init__(self, stream: TextIO | None = None, verbose: bool = False):
        self._stream = stream if stream is not None else sys.stdout
        self._verbose = verbose

    def write(self, decision: Decision) -> None:
        if not self._verbose and not decision.knobs_changed:
            return
        line = json.dumps(decision.to_dict(), separators=(",", ":"))
        self._stream.write(line + "\n")
        self._stream.flush()

    def close(self) -> None:
        if self._stream not in (sys.stdout, sys.stderr):
            try:
                self._stream.close()
            except Exception:
                pass
