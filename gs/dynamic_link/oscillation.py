"""Phase 2 — failsafe 2: GS-side oscillation detector.

Design doc §5 failsafe 2: if any knob emits > N distinct value
changes in a `window_ms` window, lock that knob at its *highest
recent value* for `lock_ms`. Applies independently per knob.

"Highest" semantics depend on the knob:
  - FEC `(k, n)`:   highest overhead wins → pick by (n - k) first,
                    then by n (more parity is safer).
  - depth:          max (more interleaving is safer against bursts).
  - MCS:            min (lower MCS is more robust).
  - TX power:       max (more link margin is safer).
  - bitrate_kbps:   min (lower is safer under marginal link).

The detector mutates a `Decision` in-place (via `maybe_override`),
appending a `reason` token noting the lock and which knobs were
overridden.
"""
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from .decision import Decision

log = logging.getLogger(__name__)


@dataclass
class OscillationConfig:
    window_ms: int = 30_000      # sliding-window length
    lock_ms: int = 60_000        # how long a locked knob stays locked
    max_changes: int = 4         # strictly-more-than-this triggers lock


@dataclass
class _Tracker:
    """Per-knob state: recent (ts_ms, value) pairs and lock state."""
    history: deque = field(default_factory=deque)
    locked_until_ms: int = 0
    locked_value: Any = None

    def add_change(self, ts_ms: int, value: Any) -> None:
        """Record a change ONLY if the value actually differs from
        the last-known. Otherwise pure no-ops don't count toward
        the oscillation budget."""
        if self.history and self.history[-1][1] == value:
            return
        self.history.append((ts_ms, value))

    def trim(self, cutoff_ms: int) -> None:
        while self.history and self.history[0][0] < cutoff_ms:
            self.history.popleft()

    def change_count(self) -> int:
        return len(self.history)


class OscillationDetector:
    """Tracks knobs `fec`, `depth`, `mcs`, `tx_power`, `bitrate`
    independently.

    Usage:
        det = OscillationDetector(cfg)
        for decision in stream:
            decision = det.maybe_override(decision, ts_ms=<int>)
            wire_send(decision)
    """

    def __init__(self, cfg: OscillationConfig):
        self.cfg = cfg
        self._trackers: dict[str, _Tracker] = {
            "fec":      _Tracker(),
            "depth":    _Tracker(),
            "mcs":      _Tracker(),
            "tx_power": _Tracker(),
            "bitrate":  _Tracker(),
        }

    # ------------------------------------------------------------
    # "safer value" selectors per knob.
    # Take a list of observed values; return the safer one.
    # ------------------------------------------------------------
    @staticmethod
    def _safer_fec(values: list[tuple[int, int]]) -> tuple[int, int]:
        # Highest overhead: maximise (n - k), tie-break by largest n.
        return max(values, key=lambda kn: (kn[1] - kn[0], kn[1]))

    @staticmethod
    def _safer_depth(values: list[int]) -> int:
        return max(values)

    @staticmethod
    def _safer_mcs(values: list[int]) -> int:
        return min(values)

    @staticmethod
    def _safer_tx_power(values: list[int]) -> int:
        return max(values)

    @staticmethod
    def _safer_bitrate(values: list[int]) -> int:
        return min(values)

    # ------------------------------------------------------------

    def maybe_override(self, decision: Decision, ts_ms: int) -> Decision:
        """Update trackers with the new values and, if any knob is
        currently locked, override the Decision's corresponding
        field. Returns the (possibly mutated) Decision with a
        `reason` suffix identifying any overrides."""
        cfg = self.cfg
        cutoff = ts_ms - cfg.window_ms
        overrides: list[str] = []

        # --- FEC (k, n) ----------------------------------------
        fec_val = (decision.k, decision.n)
        fec = self._trackers["fec"]
        fec.add_change(ts_ms, fec_val)
        fec.trim(cutoff)
        if ts_ms < fec.locked_until_ms and fec.locked_value is not None:
            lk, ln = fec.locked_value
            if (decision.k, decision.n) != (lk, ln):
                decision.k, decision.n = lk, ln
                overrides.append(f"lock_fec={lk},{ln}")
        elif fec.change_count() > cfg.max_changes:
            values = [v for _, v in fec.history]
            safe = self._safer_fec(values)
            fec.locked_until_ms = ts_ms + cfg.lock_ms
            fec.locked_value = safe
            log.warning(
                "oscillation: fec locked to (%d,%d) for %d ms (%d changes in %d ms)",
                safe[0], safe[1], cfg.lock_ms, fec.change_count(), cfg.window_ms,
            )
            decision.k, decision.n = safe
            overrides.append(f"lock_fec={safe[0]},{safe[1]}")

        # --- depth ---------------------------------------------
        self._apply_scalar(
            decision, "depth", ts_ms, cutoff, self._safer_depth,
            overrides, "depth",
        )
        # --- mcs -----------------------------------------------
        self._apply_scalar(
            decision, "mcs", ts_ms, cutoff, self._safer_mcs,
            overrides, "mcs",
        )
        # --- tx_power ------------------------------------------
        self._apply_scalar(
            decision, "tx_power_dBm", ts_ms, cutoff, self._safer_tx_power,
            overrides, "tx_power", tracker_key="tx_power",
        )
        # --- bitrate -------------------------------------------
        self._apply_scalar(
            decision, "bitrate_kbps", ts_ms, cutoff, self._safer_bitrate,
            overrides, "bitrate", tracker_key="bitrate",
        )

        if overrides:
            tag = "oscillation[" + ",".join(overrides) + "]"
            decision.reason = (
                f"{decision.reason}; {tag}" if decision.reason else tag
            )
            if "oscillation" not in decision.knobs_changed:
                decision.knobs_changed.append("oscillation")

        return decision

    def _apply_scalar(
        self,
        decision: Decision,
        attr: str,
        ts_ms: int,
        cutoff: int,
        safer,
        overrides: list[str],
        reason_prefix: str,
        tracker_key: str | None = None,
    ) -> None:
        key = tracker_key or attr
        t = self._trackers[key]
        val = getattr(decision, attr)
        t.add_change(ts_ms, val)
        t.trim(cutoff)
        if ts_ms < t.locked_until_ms and t.locked_value is not None:
            if val != t.locked_value:
                setattr(decision, attr, t.locked_value)
                overrides.append(f"lock_{reason_prefix}={t.locked_value}")
        elif t.change_count() > self.cfg.max_changes:
            values = [v for _, v in t.history]
            safe = safer(values)
            t.locked_until_ms = ts_ms + self.cfg.lock_ms
            t.locked_value = safe
            log.warning(
                "oscillation: %s locked to %s for %d ms (%d changes in %d ms)",
                reason_prefix, safe, self.cfg.lock_ms,
                t.change_count(), self.cfg.window_ms,
            )
            if val != safe:
                setattr(decision, attr, safe)
            overrides.append(f"lock_{reason_prefix}={safe}")
