"""Derive an `observed` block from a wfb-ng RxEvent.

Used by the verbose log so each per-tick row carries RX-side ground
truth (actual MCS, FEC k/n, post-FEC byte rate) alongside the
controller's intended knobs. Lag between intent and observation is
*not* computed here — that's an offline step in `dl_report`.
"""
from __future__ import annotations

from .signals import WINDOW_S
from .stats_client import RxEvent


def derive_observed(ev: RxEvent | None) -> dict:
    """Return the observed-block dict for one RxEvent.

    Empty dict if `ev` is None or carries nothing useful. Keys are
    omitted (not nulled) when the source data isn't present, so the
    JSON stays small.

    Fields:
      mcs              - max(RxAnt.mcs) across antennas
      fec_k, fec_n     - from RxEvent.session
      interleave_depth - from RxEvent.session
      bitrate_kbps     - out_bytes * 8 / WINDOW_S / 1000, rounded
      packet_rate_pps  - out / WINDOW_S, rounded
    """
    if ev is None:
        return {}
    out: dict[str, int] = {}

    if ev.rx_ant_stats:
        out["mcs"] = max(a.mcs for a in ev.rx_ant_stats)

    if ev.session is not None:
        out["fec_k"] = ev.session.fec_k
        out["fec_n"] = ev.session.fec_n
        out["interleave_depth"] = ev.session.interleave_depth

    pw = ev.packets_window
    if "out_bytes" in pw:
        out["bitrate_kbps"] = round(pw["out_bytes"] * 8 / WINDOW_S / 1000)
    if "out" in pw:
        out["packet_rate_pps"] = round(pw["out"] / WINDOW_S)

    return out
