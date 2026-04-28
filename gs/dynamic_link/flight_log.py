"""Per-flight log directory rotator.

Detects drone disconnect via sustained empty `RxEvent.rx_ant_stats`
(see `docs/ideas/per-flight-jsonl-rotation.md`, Option C). On the
first non-empty rx after a quiet period, opens a fresh
``flight-NNNN/`` directory under the configured root. After
``gap_seconds`` of sustained empty rx_ant_stats, closes the
directory, writes a small ``flight.json`` manifest, and stops
emitting writes until the next reconnect.

Counter is monotonic and resumes from disk: scanning ``log_dir`` for
existing ``flight-NNNN`` dirs picks ``max(N) + 1`` as the seed.
Naming is incremental (not wall-clock) because the GS box has no
RTC battery and may boot with a wrong clock.

Sinks consume this rotator via the four ``*_stream()`` getters,
which return the live ``TextIO`` while in-flight or ``None`` between
flights — sinks should drop writes when the getter returns ``None``.
"""
from __future__ import annotations

import json
import logging
import re
import time
from pathlib import Path
from typing import TextIO

from .stats_client import RxEvent

log = logging.getLogger(__name__)

_DIR_RE = re.compile(r"^flight-(\d{4,})$")
_DIR_FMT = "flight-{:04d}"

EVENTS_FILE     = "gs.jsonl"
VERBOSE_FILE    = "gs.verbose.jsonl"
LATENCY_FILE    = "latency.jsonl"
VIDEO_RTP_FILE  = "video_rtp.jsonl"
MANIFEST_FILE   = "flight.json"


class FlightDirRotator:
    """Owns the current per-flight bundle directory and the four
    open JSONL handles. State machine: ``between_flights`` ⇄
    ``in_flight``."""

    def __init__(self, log_dir: Path, gap_seconds: float = 10.0):
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._gap_seconds = float(gap_seconds)

        self._counter = self._seed_counter(self._log_dir)
        self._in_flight = False
        self._current_dir: Path | None = None
        self._streams: dict[str, TextIO] = {}

        # Boundary tracking. ``_first_empty_ts`` is the timestamp of the
        # first empty-rx_ant_stats event in the current quiet streak; it
        # resets whenever a non-empty rx arrives.
        self._first_empty_ts: float | None = None

        # Per-flight metadata captured for the manifest.
        self._flight_start_wall_us: int | None = None
        self._flight_start_mono_us: int | None = None
        self._flight_start_event_ts: float | None = None
        self._flight_last_event_ts: float | None = None
        self._session_epoch: int | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def on_rx_event(self, ev: RxEvent) -> None:
        """Drive the state machine off one rx event. Call this BEFORE
        any sink writes for the same event so that the open/close
        decision is committed first."""
        if ev.rx_ant_stats:
            self._first_empty_ts = None
            if not self._in_flight:
                self._open_flight(ev)
            else:
                self._flight_last_event_ts = ev.timestamp
                if ev.session is not None and self._session_epoch is None:
                    self._session_epoch = ev.session.epoch
        else:
            if self._in_flight:
                if self._first_empty_ts is None:
                    self._first_empty_ts = ev.timestamp
                elif ev.timestamp - self._first_empty_ts >= self._gap_seconds:
                    self._close_flight(reason="gap")

    def events_stream(self) -> TextIO | None:
        return self._streams.get(EVENTS_FILE) if self._in_flight else None

    def verbose_stream(self) -> TextIO | None:
        return self._streams.get(VERBOSE_FILE) if self._in_flight else None

    def latency_stream(self) -> TextIO | None:
        return self._streams.get(LATENCY_FILE) if self._in_flight else None

    def video_rtp_stream(self) -> TextIO | None:
        return self._streams.get(VIDEO_RTP_FILE) if self._in_flight else None

    def close(self) -> None:
        if self._in_flight:
            self._close_flight(reason="shutdown")

    @property
    def current_dir(self) -> Path | None:
        return self._current_dir if self._in_flight else None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _seed_counter(log_dir: Path) -> int:
        """Scan log_dir for existing ``flight-NNNN`` dirs; return
        ``max(N) + 1``, or 1 if none found. Tolerates malformed
        sibling entries (files, mixed-name dirs) and gaps in numbering."""
        max_n = 0
        try:
            entries = list(log_dir.iterdir())
        except FileNotFoundError:
            return 1
        for p in entries:
            if not p.is_dir():
                continue
            m = _DIR_RE.match(p.name)
            if m is None:
                continue
            try:
                n = int(m.group(1))
            except ValueError:
                continue
            if n > max_n:
                max_n = n
        return max_n + 1

    def _open_flight(self, ev: RxEvent) -> None:
        n = self._counter
        self._counter += 1
        flight_dir = self._log_dir / _DIR_FMT.format(n)
        flight_dir.mkdir(parents=True, exist_ok=False)

        streams: dict[str, TextIO] = {}
        for name in (EVENTS_FILE, VERBOSE_FILE, LATENCY_FILE, VIDEO_RTP_FILE):
            streams[name] = open(flight_dir / name, "a", buffering=1)

        self._current_dir = flight_dir
        self._streams = streams
        self._in_flight = True
        self._first_empty_ts = None

        self._flight_start_wall_us = int(time.time() * 1_000_000)
        self._flight_start_mono_us = time.monotonic_ns() // 1000
        self._flight_start_event_ts = ev.timestamp
        self._flight_last_event_ts = ev.timestamp
        self._session_epoch = ev.session.epoch if ev.session is not None else None

        log.info("flight opened: %s", flight_dir)

    def _close_flight(self, *, reason: str) -> None:
        if not self._in_flight:
            return

        stop_wall_us = int(time.time() * 1_000_000)
        stop_mono_us = time.monotonic_ns() // 1000

        manifest = {
            "dir": self._current_dir.name if self._current_dir else None,
            "start_wall_us": self._flight_start_wall_us,
            "start_mono_us": self._flight_start_mono_us,
            "start_event_ts": self._flight_start_event_ts,
            "stop_wall_us": stop_wall_us,
            "stop_mono_us": stop_mono_us,
            "stop_event_ts": self._flight_last_event_ts,
            "session_epoch": self._session_epoch,
            "reason": reason,
        }

        for s in self._streams.values():
            try:
                s.flush()
                s.close()
            except Exception:  # pylint: disable=broad-except
                log.exception("flight close: stream close failed")
        self._streams = {}

        if self._current_dir is not None:
            mpath = self._current_dir / MANIFEST_FILE
            try:
                with open(mpath, "w") as fd:
                    json.dump(manifest, fd, indent=2)
                    fd.write("\n")
                    fd.flush()
                    import os
                    os.fsync(fd.fileno())
            except Exception:  # pylint: disable=broad-except
                log.exception("flight close: manifest write failed (%s)", mpath)

        log.info(
            "flight closed: %s (reason=%s, session_epoch=%s)",
            self._current_dir, reason, self._session_epoch,
        )

        self._in_flight = False
        self._current_dir = None
        self._first_empty_ts = None
        self._flight_start_wall_us = None
        self._flight_start_mono_us = None
        self._flight_start_event_ts = None
        self._flight_last_event_ts = None
        self._session_epoch = None
