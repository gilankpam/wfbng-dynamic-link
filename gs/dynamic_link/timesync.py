"""Phase 3 — drone↔GS clock-offset estimator (Cristian's algorithm).

Each PONG packet carries (T1=gs_mono_us_echo, T2=drone_mono_recv_us,
T3=drone_mono_send_us). The GS records T4=gs_mono_us on receive, then
computes:

    RTT      = (T4 − T1) − (T3 − T2)
    offset   = ((T2 − T1) + (T3 − T4)) / 2     # gs_mono = drone_mono + offset

We don't sync wall clocks; we estimate the offset between the two
free-running monotonic clocks and apply it post-hoc when joining
drone-side and GS-side events on a single timeline.

A simple EWMA smooths the raw offset samples; samples whose RTT exceeds
3× the median of the recent window are rejected as outliers (the
underlying assumption — symmetric one-way latency — fails badly when the
return path is congested). Outlier ratio is exposed as a debug signal:
under steady link conditions it stays near zero; under congestion it
climbs.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field


@dataclass(frozen=True)
class LatencySample:
    """One PONG observation, ready for logging or further processing."""
    gs_seq: int
    ts_gs_mono_us: int
    rtt_us: int
    drone_mono_recv_us: int
    drone_mono_send_us: int
    offset_us: int
    offset_stddev_us: int
    outlier: bool


@dataclass
class TimeSync:
    """Cristian's algorithm + EWMA + outlier rejection.

    Hold this on the GS service for the lifetime of one connection;
    drop it (or call ``reset()``) on a session change."""

    ewma_alpha: float = 0.2
    rtt_window: int = 16
    outlier_rtt_factor: float = 3.0

    _offset_us_ewma: float | None = field(default=None, init=False, repr=False)
    _offset_var_ewma: float = field(default=0.0, init=False, repr=False)
    _rtt_window: deque[int] = field(default_factory=deque, init=False, repr=False)
    _samples: int = field(default=0, init=False, repr=False)
    _outliers: int = field(default=0, init=False, repr=False)

    def reset(self) -> None:
        self._offset_us_ewma = None
        self._offset_var_ewma = 0.0
        self._rtt_window.clear()
        self._samples = 0
        self._outliers = 0

    @property
    def offset_us(self) -> int | None:
        """Smoothed estimate of (gs_mono − drone_mono); None until first
        sample. Apply as ``gs_mono = drone_mono + offset_us``."""
        if self._offset_us_ewma is None:
            return None
        return int(round(self._offset_us_ewma))

    @property
    def offset_stddev_us(self) -> int:
        return int(round(math.sqrt(self._offset_var_ewma)))

    @property
    def stats(self) -> dict:
        return {
            "samples": self._samples,
            "outliers": self._outliers,
            "outlier_ratio": (
                self._outliers / self._samples if self._samples else 0.0
            ),
        }

    def observe(
        self,
        *,
        gs_seq: int,
        gs_mono_us_t1: int,
        drone_mono_recv_us_t2: int,
        drone_mono_send_us_t3: int,
        gs_mono_us_t4: int,
    ) -> LatencySample:
        """Process one PONG. Returns the resulting LatencySample —
        including ``outlier=True`` for samples that were not folded into
        the smoothed offset (the sample itself is still reported so the
        caller can log it)."""
        self._samples += 1

        rtt = (gs_mono_us_t4 - gs_mono_us_t1) - (
            drone_mono_send_us_t3 - drone_mono_recv_us_t2
        )
        rtt = max(rtt, 0)
        raw_offset = (
            (drone_mono_recv_us_t2 - gs_mono_us_t1)
            + (drone_mono_send_us_t3 - gs_mono_us_t4)
        ) // 2

        outlier = self._is_outlier(rtt)
        if outlier:
            self._outliers += 1
        else:
            self._fold(raw_offset)
            self._rtt_window.append(rtt)
            if len(self._rtt_window) > self.rtt_window:
                self._rtt_window.popleft()

        return LatencySample(
            gs_seq=gs_seq,
            ts_gs_mono_us=gs_mono_us_t4,
            rtt_us=rtt,
            drone_mono_recv_us=drone_mono_recv_us_t2,
            drone_mono_send_us=drone_mono_send_us_t3,
            offset_us=self.offset_us if self.offset_us is not None else raw_offset,
            offset_stddev_us=self.offset_stddev_us,
            outlier=outlier,
        )

    def _is_outlier(self, rtt: int) -> bool:
        if len(self._rtt_window) < 4:
            return False  # not enough history to judge
        sorted_rtts = sorted(self._rtt_window)
        median = sorted_rtts[len(sorted_rtts) // 2]
        if median <= 0:
            return False
        return rtt > median * self.outlier_rtt_factor

    def _fold(self, raw_offset: int) -> None:
        if self._offset_us_ewma is None:
            self._offset_us_ewma = float(raw_offset)
            self._offset_var_ewma = 0.0
            return
        a = self.ewma_alpha
        prev = self._offset_us_ewma
        diff = raw_offset - prev
        self._offset_us_ewma = prev + a * diff
        # Recursive variance estimate (Welford-ish, EWMA-flavoured).
        self._offset_var_ewma = (1 - a) * (self._offset_var_ewma + a * diff * diff)
