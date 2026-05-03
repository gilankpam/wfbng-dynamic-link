"""Generate a self-contained HTML flight report from a debug bundle.

Reads the four Phase-3 JSONL streams (events, verbose, latency, video),
plus an optional drone-side dl-events.jsonl, computes summary stats and
an anomaly leaderboard, and writes a single HTML file with linked
Plotly subplots — the shared x-axis means a moment of distress in one
stream is immediately visible in all the others.

Usage:
    python -m gs.tools.dl_report --bundle ./debug
    python -m gs.tools.dl_report --bundle flight.tar.gz -o report.html
"""
from __future__ import annotations

import argparse
import bisect
import html
import json
import logging
import statistics as st
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from .dl_review import _open_bundle  # bundle/tarball unpacker

log = logging.getLogger("dl-report")


# ----------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------

def _load_jsonl_resilient(path: Path) -> list[dict]:
    """Tolerant of partial/interrupted writes — common when the GS
    service restarts mid-flight."""
    out: list[dict] = []
    bad = 0
    if not path.exists():
        return out
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except ValueError:
            bad += 1
    if bad:
        log.warning("%s: skipped %d corrupt lines", path.name, bad)
    return out


@dataclass
class Streams:
    """Parsed contents of a flight bundle."""
    events: list[dict]
    verbose: list[dict]
    latency: list[dict]
    video: list[dict]
    drone: list[dict]


def load_bundle(bundle_path: Path,
                drone_events_path: Path | None) -> tuple[Streams, object]:
    bundle_dir, tmp = _open_bundle(bundle_path)
    s = Streams(
        events=_load_jsonl_resilient(bundle_dir / "gs.jsonl"),
        verbose=_load_jsonl_resilient(bundle_dir / "gs.verbose.jsonl"),
        latency=_load_jsonl_resilient(bundle_dir / "latency.jsonl"),
        video=_load_jsonl_resilient(bundle_dir / "video_rtp.jsonl"),
        drone=_load_jsonl_resilient(drone_events_path)
              if drone_events_path else [],
    )
    return s, tmp


# ----------------------------------------------------------------------
# Time alignment
# ----------------------------------------------------------------------

@dataclass
class TimeAxis:
    """Maps between wall-clock and gs-mono time domains.

    Verbose/event records carry only float `timestamp` (wall seconds).
    Latency/video records carry only `ts_gs_mono_us`. To plot them on
    one axis we walk the latency stream which has both, build a
    piecewise-linear map, and use it to convert verbose/event records
    onto the gs-mono axis."""
    # Sorted arrays for bisect lookup
    wall_us: np.ndarray
    mono_us: np.ndarray
    flight_start_mono_us: int

    @classmethod
    def build(cls, streams: Streams) -> "TimeAxis":
        if not streams.latency:
            # Fallback: identity map; verbose `timestamp` becomes the
            # "x axis" and we lose alignment with video. Useful when
            # ping_pong was off.
            return cls(wall_us=np.array([0]), mono_us=np.array([0]),
                       flight_start_mono_us=0)
        # Latency log already has both stamps. Sort by mono just in
        # case (writes are in order but EWMA outliers don't reorder).
        L = sorted(streams.latency, key=lambda r: r["ts_gs_mono_us"])
        wall_us = np.array([r["ts_gs_wall_us"] for r in L])
        mono_us = np.array([r["ts_gs_mono_us"] for r in L])
        # Flight start: earliest of all observed mono timestamps
        starts = [int(mono_us[0])]
        if streams.video:
            starts.append(min(r["ts_gs_mono_us"] for r in streams.video))
        return cls(wall_us=wall_us, mono_us=mono_us,
                   flight_start_mono_us=min(starts))

    def wall_to_mono(self, wall_us: int) -> int:
        if len(self.wall_us) < 2:
            return wall_us  # fallback
        i = bisect.bisect_left(self.wall_us, wall_us)
        if i == 0:
            # Before earliest sample — extrapolate from first segment
            dw = self.wall_us[1] - self.wall_us[0]
            dm = self.mono_us[1] - self.mono_us[0]
            return int(self.mono_us[0] + (wall_us - self.wall_us[0]) * dm / dw) \
                   if dw else int(self.mono_us[0])
        if i >= len(self.wall_us):
            # After last sample — extrapolate from last segment
            dw = self.wall_us[-1] - self.wall_us[-2]
            dm = self.mono_us[-1] - self.mono_us[-2]
            return int(self.mono_us[-1] + (wall_us - self.wall_us[-1]) * dm / dw) \
                   if dw else int(self.mono_us[-1])
        # Interpolate within bracketing samples
        w0, w1 = self.wall_us[i-1], self.wall_us[i]
        m0, m1 = self.mono_us[i-1], self.mono_us[i]
        return int(m0 + (wall_us - w0) * (m1 - m0) / (w1 - w0)) \
               if w1 != w0 else int(m0)

    def relative_seconds(self, mono_us: int) -> float:
        """gs_mono_us → seconds since flight start."""
        return (mono_us - self.flight_start_mono_us) / 1e6


# ----------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------

def _quantiles(xs, n=20):
    if len(xs) < n:
        return [min(xs), st.median(xs), max(xs)] if xs else [0, 0, 0]
    qs = st.quantiles(xs, n=n)
    return [qs[0], st.median(xs), qs[-1]]


def compute_summary(streams: Streams, axis: TimeAxis) -> dict:
    summary: dict = {}

    # Verbose timeline
    if streams.verbose:
        t0 = streams.verbose[0]["timestamp"]
        t1 = streams.verbose[-1]["timestamp"]
        summary["duration_s"] = t1 - t0
        summary["ticks"] = len(streams.verbose)

        # Detect GS service restarts (timestamp regression > 5s)
        restarts = []
        for i in range(1, len(streams.verbose)):
            gap = streams.verbose[i]["timestamp"] - streams.verbose[i-1]["timestamp"]
            if gap < -5:
                restarts.append((i, -gap))
        summary["restarts"] = restarts

        # Starvation %
        starved = sum(1 for r in streams.verbose
                      if r["signals_snapshot"].get("link_starved_w"))
        summary["starved_pct"] = starved / len(streams.verbose) * 100

        # MCS distribution
        mcs_counts: dict[int, int] = {}
        for r in streams.verbose:
            mcs_counts[r["mcs"]] = mcs_counts.get(r["mcs"], 0) + 1
        summary["mcs_distribution"] = dict(sorted(mcs_counts.items()))

        # Controller-requested IDRs (encoder-emitted IDRs aren't logged
        # here — they show up indirectly as oversized video frames).
        summary["idr_requests"] = [
            {"ts_wall": r["timestamp"],
             "ts_rel_s": r["timestamp"] - t0,
             "reason": r.get("reason", "")}
            for r in streams.verbose if r.get("idr_request")
        ]

        # RSSI / SNR percentiles (alive windows only)
        rssi = [r["signals_snapshot"]["rssi"] for r in streams.verbose
                if r["signals_snapshot"].get("rssi") is not None
                and not r["signals_snapshot"].get("link_starved_w")]
        snr = [r["signals_snapshot"]["snr"] for r in streams.verbose
               if r["signals_snapshot"].get("snr") is not None
               and not r["signals_snapshot"].get("link_starved_w")]
        if rssi:
            summary["rssi_percentiles"] = _quantiles(rssi)
        if snr:
            summary["snr_percentiles"] = _quantiles(snr)

    # Events
    summary["events"] = len(streams.events)
    summary["emergencies"] = sum(1 for r in streams.events
                                 if "emergency" in (r.get("reason") or ""))
    summary["mcs_changes"] = sum(1 for r in streams.events
                                 if "mcs_" in (r.get("reason") or ""))

    # RTT
    if streams.latency:
        rtts = [r["rtt_us"] / 1000 for r in streams.latency]
        summary["rtt_ms"] = {
            "p50": st.median(rtts),
            "p95": _quantiles(rtts, 20)[-1],
            "p99": _quantiles(rtts, 100)[-1],
            "max": max(rtts),
        }
        summary["latency_outliers"] = sum(
            1 for r in streams.latency if r.get("outlier"))

    # Video
    if streams.video:
        total_lost = sum(r["lost_in_frame"] for r in streams.video)
        total_pkts = sum(r["packets"] for r in streams.video)
        total = total_pkts + total_lost
        summary["video"] = {
            "frames": len(streams.video),
            "loss_pct": total_lost / total * 100 if total else 0,
            "ssrcs": list({r["ssrc"] for r in streams.video}),
        }
        ia = [r["frame_interarrival_us"] / 1000
              for r in streams.video if r["frame_interarrival_us"] > 0]
        if ia:
            summary["frame_interarrival_ms"] = {
                "p50": st.median(ia),
                "p95": _quantiles(ia, 20)[-1],
                "p99": _quantiles(ia, 100)[-1],
                "max": max(ia),
            }
        # Drift baseline + range (skip first 100 frames as warmup)
        warm = streams.video[100:] if len(streams.video) > 200 else streams.video
        drifts = [r["latency_drift_us"] / 1000 for r in warm]
        if drifts:
            summary["drift_ms"] = {
                "p05": _quantiles(drifts)[0],
                "p50": st.median(drifts),
                "p95": _quantiles(drifts)[-1],
                "range": max(drifts) - min(drifts),
            }

    summary["drone_events"] = len(streams.drone)
    return summary


def compute_anomalies(streams: Streams, axis: TimeAxis,
                      top_n: int = 20) -> list[dict]:
    """Top-N anomaly windows. Score = video drift excursion × frame
    interarrival × (lost_in_frame + 1) × RTT factor.

    Buckets at 1-second granularity on the gs_mono axis. Each row
    also captures starvation and SSRC-change context for downstream
    classification (`classify_anomaly`)."""
    if not streams.video:
        return []

    drifts = np.array([r["latency_drift_us"] for r in streams.video])
    # Median Absolute Deviation of drift — robust scale estimator
    median_drift = float(np.median(drifts))
    mad_drift = float(np.median(np.abs(drifts - median_drift))) or 1.0

    # Pre-sort latency by mono for bisect lookups
    L_mono = np.array([r["ts_gs_mono_us"] for r in streams.latency]) \
        if streams.latency else np.array([])
    L_rtt = np.array([r["rtt_us"] for r in streams.latency]) \
        if streams.latency else np.array([])
    median_rtt = float(np.median(L_rtt)) if L_rtt.size else 1.0

    # Bucket frames at 1 s, tracking SSRC for restart detection
    buckets: dict[int, dict] = {}
    prev_ssrc: str | None = None
    for v in streams.video:
        t = v["ts_gs_mono_us"]
        bucket = t // 1_000_000
        b = buckets.setdefault(bucket, {
            "ts_mono_us": int(bucket * 1_000_000),
            "peak_ts_mono_us": int(bucket * 1_000_000),
            "frames": 0,
            "max_drift_excursion": 0.0,
            "max_interarrival_us": 0,
            "lost": 0,
            "rtt_factor": 1.0,
            "ssrc_changed": False,
            "starved_ticks": 0,
        })
        b["frames"] += 1
        exc = abs(v["latency_drift_us"] - median_drift) / mad_drift
        # Track *when* the worst drift sample landed in the bucket so
        # downstream markers / leaderboard rows align with the visible
        # spike on the timeline rather than the bucket's left edge.
        # Drift is the most pilot-relevant metric and dominates the
        # score; we use its peak time as the representative `ts`.
        if exc > b["max_drift_excursion"]:
            b["max_drift_excursion"] = exc
            b["peak_ts_mono_us"] = int(t)
        b["max_interarrival_us"] = max(b["max_interarrival_us"],
                                       v["frame_interarrival_us"])
        b["lost"] += v["lost_in_frame"]
        if prev_ssrc is not None and v["ssrc"] != prev_ssrc:
            b["ssrc_changed"] = True
        prev_ssrc = v["ssrc"]
        if L_mono.size:
            i = int(np.searchsorted(L_mono, t))
            i = min(max(i, 0), L_mono.size - 1)
            b["rtt_factor"] = max(b["rtt_factor"],
                                  float(L_rtt[i] / median_rtt))

    # Mark which buckets contain a starved verbose tick. The verbose
    # log uses wall_us; convert each tick's wall→mono once.
    if streams.verbose:
        for r in streams.verbose:
            if not r["signals_snapshot"].get("link_starved_w"):
                continue
            mono = axis.wall_to_mono(int(r["timestamp"] * 1e6))
            bucket = mono // 1_000_000
            if bucket in buckets:
                buckets[bucket]["starved_ticks"] += 1

    # Score
    rows = []
    for b in buckets.values():
        score = (b["max_drift_excursion"] *
                 (b["max_interarrival_us"] / 16700.0) *
                 (b["lost"] + 1) *
                 b["rtt_factor"])
        rows.append({
            "score": score,
            "ts_mono_us": b["peak_ts_mono_us"],
            "ts_rel_s": axis.relative_seconds(b["peak_ts_mono_us"]),
            "max_drift_excursion": b["max_drift_excursion"],
            "max_interarrival_ms": b["max_interarrival_us"] / 1000,
            "lost": b["lost"],
            "rtt_factor": b["rtt_factor"],
            "ssrc_changed": b["ssrc_changed"],
            "starved_ticks": b["starved_ticks"],
        })
    rows.sort(key=lambda r: r["score"], reverse=True)
    return rows[:top_n]


# ----------------------------------------------------------------------
# Diagnosis — classify each anomaly so the report answers
# "why did I see stutter at moment X?" not just "here are the metrics"
# ----------------------------------------------------------------------

# Heuristic thresholds. All in the tunable spirit — these are rules of
# thumb derived from §B.2 of docs/phase3-implementation.md, not
# theoretical limits. Operators should tweak after a few real flights.
_TH_RTT_FACTOR  = 2.0   # 2× normal control-plane RTT = "elevated"
_TH_DRIFT_MAD   = 3.0   # 3× MAD of drift = "elevated"
_TH_IA_VISIBLE  = 50.0  # ms — frame interarrival visible to a pilot
_TH_IA_FRAME    = 33.0  # ms — beyond one missed frame at 60 fps


def classify_anomaly(row: dict) -> dict:
    """Return {verdict, severity, why, felt} for one anomaly bucket.

    Priority order (most-specific → least-specific):
      Drone TX silent  > Encoder restart  > Link congestion
      > Encoder pipeline > Asymmetric link > Random RF drop > Mild
    """
    rtt_hi    = row["rtt_factor"]    > _TH_RTT_FACTOR
    drift_hi  = row["max_drift_excursion"] > _TH_DRIFT_MAD
    ia_hi     = row["max_interarrival_ms"] > _TH_IA_VISIBLE
    has_loss  = row["lost"] > 0
    starved   = row["starved_ticks"] > 0
    ssrc_chg  = row["ssrc_changed"]

    # 1. Drone went silent — link metrics may look fine but pkt=0
    if starved:
        return {
            "verdict": "Drone TX silent",
            "severity": "high",
            "why": (
                f"Verbose log shows {row['starved_ticks']} starved tick(s) "
                f"in this 1 s window. Drone is no longer sending packets "
                f"the GS can decode."
            ),
            "felt": (
                "Hard freeze in the goggles. Source side stopped — could "
                "be VTX power-off, FW crash, RX desense at very close "
                "range, or severe interference jamming the band."
            ),
        }

    # 2. Encoder restart — SSRC changed mid-window
    if ssrc_chg:
        return {
            "verdict": "Encoder restart",
            "severity": "medium",
            "why": "RTP SSRC changed mid-window. The drone-side encoder "
                   "(majestic / waybeam) restarted.",
            "felt": "Brief black/glitch on the goggles. The drift "
                   "baseline reset to zero on the new SSRC.",
        }

    # 3. Link congestion — RTT and drift both elevated. Loss is a
    # severity-multiplier, not a gate: when FEC absorbs the damage we
    # still want this classified as link distress (packets arrived
    # late, just successfully). The +230ms-drift / 2× RTT / 0-loss
    # case from real bench data was previously falling through to
    # "unclassified" and that was the bug worth fixing.
    if rtt_hi and drift_hi:
        if has_loss:
            return {
                "verdict": "Link congestion",
                "severity": "high",
                "why": (
                    f"Control-plane RTT was {row['rtt_factor']:.1f}× the "
                    f"flight median, video drift jumped "
                    f"{row['max_drift_excursion']:.1f} MADs above baseline, "
                    f"and {row['lost']} packet(s) were lost. Link itself "
                    f"ran out of capacity."
                ),
                "felt": (
                    "Visible video stutter, possibly with image freeze or "
                    "tearing. The controller likely reacted by stepping MCS "
                    "down within ~100 ms."
                ),
            }
        # No decoded loss — FEC absorbed the damage. Pilot still felt
        # it because frames arrived late, but data integrity preserved.
        return {
            "verdict": "Link stress (FEC absorbed)",
            "severity": "medium",
            "why": (
                f"Control-plane RTT was {row['rtt_factor']:.1f}× the "
                f"flight median and video drift jumped "
                f"{row['max_drift_excursion']:.1f} MADs above baseline, "
                f"but FEC recovered every packet (decoded loss = 0). "
                f"The link congested; the controller's redundancy budget "
                f"saved the pixels."
            ),
            "felt": (
                "Visible jitter or stutter even though no frames were "
                "actually lost — late arrival is enough to be felt. If "
                "this happens often, your FEC overhead may be carrying "
                "more weight than it should."
            ),
        }

    # 4. Encoder pipeline — drift elevated, RTT not. Loss optional;
    # packets dropped because the encoder couldn't keep up is even
    # stronger evidence of pipeline distress, not a different category.
    if drift_hi and not rtt_hi:
        loss_note = (f" {row['lost']} packet(s) were also dropped — "
                     f"likely the pipeline catching up by skipping frames."
                     if has_loss else "")
        return {
            "verdict": "Encoder pipeline",
            "severity": "medium",
            "why": (
                f"Video drift jumped {row['max_drift_excursion']:.1f} MADs "
                f"but control-plane RTT was normal "
                f"({row['rtt_factor']:.1f}× median).{loss_note} Distress "
                f"is downstream of the link — drone-side encoder queue or "
                f"codec hiccup."
            ),
            "felt": (
                "Goggle stutter or freeze with the link reporting healthy. "
                "Worth checking encoder bitrate ceilings and drone CPU "
                "load — the controller's apply_fail / ENC_RESPONSE_BAD "
                "events on the drone SD log may also flag this."
            ),
        }

    # 5. Asymmetric link — RTT spike alone
    if rtt_hi and not drift_hi and not has_loss:
        return {
            "verdict": "Asymmetric link",
            "severity": "low",
            "why": (
                f"Control-plane RTT was {row['rtt_factor']:.1f}× the "
                f"median, but video drift and frame interarrival were "
                f"normal. The forward (drone → GS) path is fine; the "
                f"return path (GS → drone) had a transient."
            ),
            "felt": (
                "Likely invisible to the pilot. Reflected in the controller "
                "being slow to react to a subsequent loss event, since the "
                "GS's decision packet took longer than usual to reach the "
                "drone."
            ),
        }

    # 6. Random RF drop — packets lost without RTT/drift signature
    if has_loss and not rtt_hi and not drift_hi:
        return {
            "verdict": "Random RF drop",
            "severity": "low",
            "why": (
                f"{row['lost']} packet(s) lost, but control-plane RTT and "
                f"video drift stayed normal. Looks like brief external "
                f"interference rather than capacity exhaustion."
            ),
            "felt": (
                "Probably 1 missed frame; FEC may have masked it. "
                "If this category shows up frequently across many flights "
                "in the same physical area, investigate co-channel users."
            ),
        }

    # 7. Single-frame stutter — interarrival spike with little else
    if ia_hi and not has_loss and row["max_drift_excursion"] < _TH_DRIFT_MAD:
        return {
            "verdict": "Single-frame hiccup",
            "severity": "low",
            "why": (
                f"One frame arrived {row['max_interarrival_ms']:.0f} ms "
                f"after the previous one (vs ~16.7 ms expected at 60 fps), "
                f"but no loss and no sustained drift change."
            ),
            "felt": "Single juddery frame, easy to miss.",
        }

    # 8. Fallback — high score but no clear pattern
    return {
        "verdict": "Mild anomaly (unclassified)",
        "severity": "low",
        "why": "Score elevated but the metrics don't fit any known "
               "pattern. Inspect manually in the linked-timeline view.",
        "felt": "Probably nothing visible.",
    }


def compute_diagnosis(streams: Streams, axis: TimeAxis,
                      anomalies: list[dict]) -> dict:
    """Flight-level summary aggregating anomaly classifications."""
    classified = [(a, classify_anomaly(a)) for a in anomalies]
    counts: dict[str, int] = {}
    for _, c in classified:
        counts[c["verdict"]] = counts.get(c["verdict"], 0) + 1

    # Emergencies straight out of the event log
    emerg = [r for r in streams.events
             if "emergency" in (r.get("reason") or "")]

    # Controller-requested IDRs (read off the verbose log directly so we
    # don't depend on what the events stream filters in).
    t0 = streams.verbose[0]["timestamp"] if streams.verbose else 0.0
    idrs = [
        {"timestamp": r["timestamp"],
         "ts_rel_s": r["timestamp"] - t0,
         "reason": r.get("reason", "")}
        for r in streams.verbose if r.get("idr_request")
    ]

    return {
        "verdict_counts": dict(sorted(counts.items())),
        "classified": classified,
        "emergencies": emerg,
        "idr_requests": idrs,
    }


# ----------------------------------------------------------------------
# Plotting
# ----------------------------------------------------------------------

# Per-color dash patterns for cross-panel marker rules. Different
# patterns let two markers at the same x stay visually distinguishable
# (an emergency that also triggered an IDR shows red long-dashes plus
# green dots overlapping at the same vertical line).
_DASH_FOR_COLOR = {
    "red":    "longdash",  # emergencies — strong, easy to spot
    "orange": "dashdot",   # watchdog
    "green":  "dot",       # IDR — visible "through" any other marker
    "purple": "dash",      # drone failure
}


def _event_markers(streams: Streams, axis: TimeAxis) -> list[dict]:
    """High-signal events worth a vertical line on every panel:
    emergencies, watchdogs, controller IDR requests, drone-side
    failures. MCS up/down events are skipped intentionally — there are
    dozens per flight and they're already rendered as the MCS step
    trace in row 1."""
    out = []
    for r in streams.events:
        reason = r.get("reason") or ""
        is_emergency = "emergency" in reason
        is_watchdog = "watchdog" in reason.lower()
        if not (is_emergency or is_watchdog):
            continue
        wall_us = int(r["timestamp"] * 1e6)
        mono = axis.wall_to_mono(wall_us)
        x = axis.relative_seconds(mono)
        color = "red" if is_emergency else "orange"
        out.append({"x": x, "label": reason, "color": color})
    # Controller IDR requests — not necessarily anomalies, but useful
    # to align against video drift spikes (a tame IDR shouldn't visibly
    # stress the link; an oversized encoder IDR with no marker here is
    # a hint the encoder emitted one on its own schedule).
    #
    # When an emergency at the same tick also triggered the IDR
    # (`+IDR` in the reason), both events get a marker. They're drawn
    # with distinct dash patterns (see `_DASH_FOR_COLOR`) so coincident
    # markers stay visually distinguishable instead of one clobbering
    # the other.
    for r in streams.verbose:
        if not r.get("idr_request"):
            continue
        wall_us = int(r["timestamp"] * 1e6)
        mono = axis.wall_to_mono(wall_us)
        x = axis.relative_seconds(mono)
        out.append({"x": x, "label": f"IDR: {r.get('reason','')}",
                    "color": "green"})
    # Drone-side events (re-stamped onto GS-mono via offset). Skip if no
    # latency stream — we'd be plotting at raw drone-mono which lies.
    if streams.latency and streams.drone:
        L_mono = np.array([r["drone_mono_recv_us"] for r in streams.latency])
        L_off = np.array([r["offset_us"] for r in streams.latency])
        for r in streams.drone:
            i = int(np.searchsorted(L_mono, r["t"]))
            i = min(max(i, 0), L_mono.size - 1)
            # Sign convention: gs_mono = drone_mono - offset_us
            mono = r["t"] - int(L_off[i])
            x = axis.relative_seconds(mono)
            out.append({
                "x": x, "label": f"DRONE: {r.get('reason')}",
                "color": "purple",
            })
    return out


def make_timeline_figure(streams: Streams, axis: TimeAxis,
                         diagnosis: dict | None = None,
                         top_events_n: int = 8) -> go.Figure:
    """Eight stacked subplots, shared x-axis.

    If `diagnosis` is supplied, the top `top_events_n` classified
    anomalies are overlaid as hoverable diamond markers on row 1
    (color by severity) plus thin gray cross-panel rules. Hovering a
    marker shows the verdict + score + why + felt blocks — this is
    the only place the per-event "why / felt" reasoning appears in
    the report; the markdown / HTML body keeps just the verdict
    counts and the leaderboard.
    """
    fig = make_subplots(
        rows=8, cols=1, shared_xaxes=True,
        vertical_spacing=0.020,
        row_heights=[0.14, 0.13, 0.10, 0.13, 0.11, 0.13, 0.14, 0.12],
        subplot_titles=(
            "MCS / bitrate",
            "RSSI / SNR (alive windows)",
            "SNR slope (dB/tick; negative = SNR collapsing)",
            "residual_loss / fec_work",
            "FEC k / n (data shards / total shards)",
            "Control-plane RTT",
            "Video latency drift (relative to baseline)",
            "Frame interarrival",
        ),
        specs=[
            [{"secondary_y": True}],
            [{"secondary_y": True}],
            [{"secondary_y": False}],
            [{"secondary_y": False}],
            [{"secondary_y": False}],
            [{"secondary_y": False}],
            [{"secondary_y": True}],   # drift + bitrate overlay
            [{"secondary_y": False}],
        ],
    )

    # Row 1: MCS step + bitrate line
    if streams.verbose:
        wall_us = np.array([int(r["timestamp"] * 1e6) for r in streams.verbose])
        mono_us = np.array([axis.wall_to_mono(int(w)) for w in wall_us])
        x = (mono_us - axis.flight_start_mono_us) / 1e6
        mcs = np.array([r["mcs"] for r in streams.verbose])
        bitrate = np.array([r["bitrate_kbps"] for r in streams.verbose])
        fig.add_trace(go.Scatter(x=x, y=mcs, mode="lines",
                                 line={"shape": "hv", "color": "#1f77b4"},
                                 name="MCS"),
                      row=1, col=1, secondary_y=False)
        fig.add_trace(go.Scatter(x=x, y=bitrate, mode="lines",
                                 line={"color": "#ff7f0e", "width": 1},
                                 name="bitrate (kbps)"),
                      row=1, col=1, secondary_y=True)

        # Observed RX-side traces (dashed, same hue) on row 1 and row 5.
        # NaN-filled when no observed block on that row, so older bundles
        # produce empty traces (Plotly hides them from view but they show
        # in the legend; that's acceptable noise for a feature-flagged
        # field). Skip the add_trace entirely when zero observed rows
        # exist so older reports stay visually unchanged.
        has_observed = any(r.get("observed") for r in streams.verbose)
        if has_observed:
            obs_mcs = np.array([
                (r.get("observed") or {}).get("mcs", np.nan)
                for r in streams.verbose
            ], dtype=float)
            obs_br = np.array([
                (r.get("observed") or {}).get("bitrate_kbps", np.nan)
                for r in streams.verbose
            ], dtype=float)
            fig.add_trace(go.Scatter(
                x=x, y=obs_mcs, mode="lines",
                line={"shape": "hv", "color": "#1f77b4",
                      "dash": "dash", "width": 1},
                name="MCS (observed)", connectgaps=False, opacity=0.7,
            ), row=1, col=1, secondary_y=False)
            fig.add_trace(go.Scatter(
                x=x, y=obs_br, mode="lines",
                line={"color": "#ff7f0e", "dash": "dash", "width": 1},
                name="bitrate observed (kbps)", connectgaps=False,
                opacity=0.7,
            ), row=1, col=1, secondary_y=True)

        # Row 2: RSSI / SNR (NaN out starved windows so the line breaks)
        rssi_y = np.array([
            r["signals_snapshot"]["rssi"]
            if (r["signals_snapshot"].get("rssi") is not None
                and not r["signals_snapshot"].get("link_starved_w"))
            else np.nan
            for r in streams.verbose
        ])
        snr_y = np.array([
            r["signals_snapshot"]["snr"]
            if (r["signals_snapshot"].get("snr") is not None
                and not r["signals_snapshot"].get("link_starved_w"))
            else np.nan
            for r in streams.verbose
        ])
        fig.add_trace(go.Scatter(x=x, y=rssi_y, mode="lines",
                                 line={"color": "#1f77b4"}, name="RSSI dBm",
                                 connectgaps=False),
                      row=2, col=1, secondary_y=False)
        fig.add_trace(go.Scatter(x=x, y=snr_y, mode="lines",
                                 line={"color": "#2ca02c"}, name="SNR dB",
                                 connectgaps=False),
                      row=2, col=1, secondary_y=True)

        # Row 3: SNR slope. Reference lines at 0 and -0.3 (the
        # empirical predictive threshold from the t=141 / t=312 / etc.
        # residual_loss events on the bench flight).
        slope = np.array([
            r["signals_snapshot"].get("snr_slope") or 0
            for r in streams.verbose
        ])
        fig.add_trace(go.Scatter(x=x, y=slope, mode="lines",
                                 line={"color": "#2ca02c", "width": 1},
                                 name="snr_slope"),
                      row=3, col=1)
        fig.add_hline(y=0, line={"width": 1, "dash": "dot",
                                 "color": "rgba(0,0,0,0.3)"},
                      row=3, col=1)
        fig.add_hline(y=-0.3, line={"width": 1, "dash": "dash",
                                    "color": "rgba(214,39,40,0.5)"},
                      row=3, col=1)

        # Row 4: loss / fec
        loss = np.array([
            r["signals_snapshot"].get("residual_loss_w") or 0
            for r in streams.verbose
        ])
        fec = np.array([
            r["signals_snapshot"].get("fec_work") or 0
            for r in streams.verbose
        ])
        fig.add_trace(go.Scatter(x=x, y=loss, mode="lines",
                                 line={"color": "#d62728"}, name="residual_loss"),
                      row=4, col=1)
        fig.add_trace(go.Scatter(x=x, y=fec, mode="lines",
                                 line={"color": "#9467bd"}, name="fec_work"),
                      row=4, col=1)

        # Row 5: FEC shards. Step lines (shape="hv") so transitions are
        # honest — k and n are integer ladder positions, not continuous.
        k_vals = np.array([r["k"] for r in streams.verbose])
        n_vals = np.array([r["n"] for r in streams.verbose])
        fig.add_trace(go.Scatter(x=x, y=k_vals, mode="lines",
                                 line={"shape": "hv", "color": "#1f77b4"},
                                 name="k (data)"),
                      row=5, col=1)
        fig.add_trace(go.Scatter(x=x, y=n_vals, mode="lines",
                                 line={"shape": "hv", "color": "#ff7f0e"},
                                 name="n (data+fec)"),
                      row=5, col=1)
        if has_observed:
            obs_k = np.array([
                (r.get("observed") or {}).get("fec_k", np.nan)
                for r in streams.verbose
            ], dtype=float)
            obs_n = np.array([
                (r.get("observed") or {}).get("fec_n", np.nan)
                for r in streams.verbose
            ], dtype=float)
            fig.add_trace(go.Scatter(
                x=x, y=obs_k, mode="lines",
                line={"shape": "hv", "color": "#1f77b4",
                      "dash": "dash", "width": 1},
                name="k (observed)", connectgaps=False, opacity=0.7,
            ), row=5, col=1)
            fig.add_trace(go.Scatter(
                x=x, y=obs_n, mode="lines",
                line={"shape": "hv", "color": "#ff7f0e",
                      "dash": "dash", "width": 1},
                name="n (observed)", connectgaps=False, opacity=0.7,
            ), row=5, col=1)

    # Row 6: RTT (split outliers vs clean for color)
    if streams.latency:
        x_lat = np.array([
            axis.relative_seconds(r["ts_gs_mono_us"]) for r in streams.latency
        ])
        rtt_ms = np.array([r["rtt_us"] / 1000 for r in streams.latency])
        outlier = np.array([bool(r.get("outlier")) for r in streams.latency])
        fig.add_trace(go.Scatter(
            x=x_lat[~outlier], y=rtt_ms[~outlier],
            mode="markers", marker={"size": 4, "color": "#1f77b4"},
            name="RTT clean",
        ), row=6, col=1)
        if outlier.any():
            fig.add_trace(go.Scatter(
                x=x_lat[outlier], y=rtt_ms[outlier],
                mode="markers", marker={"size": 7, "color": "red", "symbol": "x"},
                name="RTT outlier",
            ), row=6, col=1)

    # Row 7: drift
    if streams.video:
        x_v = np.array([
            axis.relative_seconds(r["ts_gs_mono_us"]) for r in streams.video
        ])
        drift_ms = np.array([r["latency_drift_us"] / 1000
                             for r in streams.video])
        # Plot drift as deviation from median (so we see *changes*, not the
        # arbitrary baseline)
        warm = drift_ms[100:] if drift_ms.size > 200 else drift_ms
        baseline = float(np.median(warm)) if warm.size else 0.0
        fig.add_trace(go.Scatter(
            x=x_v, y=drift_ms - baseline, mode="lines",
            line={"color": "#17becf", "width": 1},
            name=f"drift − {baseline:+.0f}ms",
        ), row=7, col=1, secondary_y=False)
        fig.add_hline(y=0, line={"width": 1, "dash": "dot",
                                 "color": "rgba(0,0,0,0.3)"},
                      row=7, col=1)
        # Overlay bitrate (kbps) on the drift panel's right axis so
        # MCS/bitrate retargets line up visually with their drift
        # consequences. Same data series as row 1's secondary y;
        # repeated here purely for alignment.
        if streams.verbose:
            fig.add_trace(go.Scatter(
                x=x, y=bitrate, mode="lines",
                line={"color": "#ff7f0e", "width": 1},
                name="bitrate (kbps)",
                legendgroup="bitrate",
                showlegend=False,    # already in legend from row 1
                opacity=0.7,
            ), row=7, col=1, secondary_y=True)

        # Row 8: interarrival
        ia_ms = np.array([r["frame_interarrival_us"] / 1000
                          for r in streams.video])
        ia_ms = np.where(ia_ms > 0, ia_ms, np.nan)
        fig.add_trace(go.Scatter(
            x=x_v, y=ia_ms, mode="lines",
            line={"color": "#8c564b", "width": 1},
            name="frame interarrival ms",
        ), row=8, col=1)
        # 60fps reference
        fig.add_hline(y=16.67, line={"width": 1, "dash": "dot",
                                     "color": "rgba(0,0,0,0.3)"},
                      row=8, col=1)

    # Vertical event markers — batched as a single shapes list update
    # rather than 6 × N add_vline calls (each triggers a layout
    # rebuild). On a 4-min flight with 6 emergencies that's the
    # difference between 30s and <1s.
    markers = _event_markers(streams, axis)

    # Layout shapes don't get a Plotly legend entry, so for each color
    # group also emit one invisible-line + visible-marker scatter on
    # the MCS panel. That gives a clickable legend and a hover label,
    # while the dashed shapes stay as the actual cross-panel rule.
    LEGEND_NAMES = {
        "red":    "emergency",
        "orange": "watchdog",
        "green":  "IDR request",
        "purple": "drone failure",
    }
    by_color: dict[str, list[dict]] = {}
    for m in markers:
        by_color.setdefault(m["color"], []).append(m)
    for color, ms in by_color.items():
        fig.add_trace(go.Scatter(
            x=[m["x"] for m in ms],
            y=[0] * len(ms),
            mode="markers",
            marker={"color": color, "symbol": "triangle-up", "size": 9,
                    "line": {"color": color, "width": 1}},
            text=[m["label"] for m in ms],
            hovertemplate="t=%{x:.2f}s<br>%{text}<extra></extra>",
            name=LEGEND_NAMES.get(color, color),
            legendgroup=color,
            showlegend=True,
        ), row=1, col=1, secondary_y=False)

    # Top events overlay — diamond markers per severity, plus thin
    # gray dotted cross-panel rules. Hover any marker to see the
    # classifier's verdict / why / felt for that 1-second window.
    top_event_shapes: list[dict] = []
    if diagnosis and diagnosis.get("classified"):
        SEV_COLOR = {"high": "#d62728", "medium": "#ff7f0e", "low": "#888"}
        SEV_RANK = {"high": 0, "medium": 1, "low": 2}
        sev_buckets: dict[str, list[tuple[int, dict, dict]]] = {
            "high": [], "medium": [], "low": []
        }
        for rank, (a, c) in enumerate(diagnosis["classified"][:top_events_n], 1):
            sev = c.get("severity", "low")
            if sev in sev_buckets:
                sev_buckets[sev].append((rank, a, c))

        for sev in sorted(sev_buckets, key=lambda s: SEV_RANK[s]):
            entries = sev_buckets[sev]
            if not entries:
                continue
            # Plain text in `text` field — Plotly's hovertemplate splits
            # on <br>, so we pre-wrap the multi-line "why" / "felt"
            # blocks at ~70 chars to keep the hover box readable.
            def wrap(s: str, w: int = 70) -> str:
                out, line = [], ""
                for word in s.split():
                    if line and len(line) + 1 + len(word) > w:
                        out.append(line); line = word
                    else:
                        line = (line + " " + word) if line else word
                if line: out.append(line)
                return "<br>".join(out)

            xs = [a["ts_rel_s"] for _, a, _ in entries]
            txt = [
                f"<b>#{rank}: {c['verdict']}</b> ({c['severity']})<br>"
                f"score={a['score']:.1f} · "
                f"RTT×{a['rtt_factor']:.2f} · "
                f"drift {a['max_drift_excursion']:.1f} MAD · "
                f"ia {a['max_interarrival_ms']:.1f}ms · "
                f"lost={a['lost']}<br>"
                f"<i>Why:</i><br>{wrap(c['why'])}<br>"
                f"<i>Felt:</i><br>{wrap(c['felt'])}"
                for rank, a, c in entries
            ]
            color = SEV_COLOR[sev]
            fig.add_trace(go.Scatter(
                x=xs, y=[0] * len(xs),
                mode="markers",
                marker={"color": color, "symbol": "diamond", "size": 11,
                        "line": {"color": "#222", "width": 1}},
                text=txt,
                hovertemplate="t=%{x:.2f}s<br>%{text}<extra></extra>",
                name=f"top event ({sev})",
                legendgroup=f"top-{sev}",
                showlegend=True,
            ), row=1, col=1, secondary_y=False)

            # Thin cross-panel rule per top event. Gray + dotted so
            # they don't compete with emergency / IDR rules but stay
            # alignable across all panels.
            for x in xs:
                top_event_shapes.append({"x": x, "color": "#888"})

    shapes = []
    # Rows 1 and 2 each carry a secondary_y axis, so Plotly numbers the
    # eight rows' primary y-axes y, y3, y5, y6, y7, y8, y9, y10 — not
    # y1..y8. Mapping the wrong axis silently drops the rule on the
    # drift / interarrival panels, where you most need it.
    ROW_YREF = {1: "y", 2: "y3", 3: "y5", 4: "y6",
                5: "y7", 6: "y8", 7: "y9", 8: "y11"}
    for m in markers:
        # `xref="x domain"` would lock to one subplot; we want full
        # vertical sweep, so use one shape per row with explicit yref.
        dash = _DASH_FOR_COLOR.get(m["color"], "dash")
        for row in range(1, 9):
            yref = f"{ROW_YREF[row]} domain"
            xref = "x" if row == 1 else f"x{row}"
            shapes.append({
                "type": "line", "xref": xref, "yref": yref,
                "x0": m["x"], "x1": m["x"],
                "y0": 0, "y1": 1,
                "line": {"color": m["color"], "width": 1, "dash": dash},
            })
    # Thin semi-transparent dotted rules for top events. Layout shapes
    # don't accept an `opacity` property — bake the alpha into rgba.
    for m in top_event_shapes:
        for row in range(1, 9):
            yref = f"{ROW_YREF[row]} domain"
            xref = "x" if row == 1 else f"x{row}"
            shapes.append({
                "type": "line", "xref": xref, "yref": yref,
                "x0": m["x"], "x1": m["x"],
                "y0": 0, "y1": 1,
                "line": {"color": "rgba(136,136,136,0.4)",
                         "width": 1, "dash": "dot"},
            })

    fig.update_xaxes(title_text="Time (s, gs_mono since flight start)",
                     row=8, col=1)
    fig.update_yaxes(title_text="MCS", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="kbps", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="dBm", row=2, col=1, secondary_y=False)
    fig.update_yaxes(title_text="dB", row=2, col=1, secondary_y=True)
    fig.update_yaxes(title_text="dB/tick", row=3, col=1)
    fig.update_yaxes(title_text="rate", row=4, col=1)
    fig.update_yaxes(title_text="shards", row=5, col=1)
    fig.update_yaxes(title_text="ms", row=6, col=1)
    fig.update_yaxes(title_text="Δ ms", row=7, col=1, secondary_y=False)
    fig.update_yaxes(title_text="kbps", row=7, col=1, secondary_y=True)
    fig.update_yaxes(title_text="ms", row=8, col=1)

    # Plotly's update_layout(shapes=...) does a positional merge with
    # the existing layout.shapes (the add_hline / add_vline rules
    # registered above). If we pass the marker list straight in, the
    # first N shapes get clobbered by the per-row hlines. Concatenate
    # so both sets survive.
    fig.update_layout(
        height=1550,
        margin={"l": 60, "r": 30, "t": 60, "b": 50},
        legend={"orientation": "h", "y": 1.04, "x": 0},
        hovermode="x unified",
        shapes=list(fig.layout.shapes) + shapes,
    )
    return fig


def make_distribution_figure(streams: Streams) -> go.Figure:
    """Three histograms (RTT / interarrival / drift) plus a fec_work
    vs residual_loss scatter that visualises whether FEC ramps up
    *before* loss spills past it."""
    fig = make_subplots(
        rows=1, cols=4,
        subplot_titles=("RTT (ms)", "Frame interarrival (ms)",
                        "Latency drift (ms)",
                        "fec_work vs residual_loss"),
        horizontal_spacing=0.07,
    )

    if streams.latency:
        rtts = [r["rtt_us"] / 1000 for r in streams.latency]
        fig.add_trace(go.Histogram(x=rtts, nbinsx=80, name="RTT",
                                    marker={"color": "#1f77b4"}),
                      row=1, col=1)
        for q, label in [(0.5, "p50"), (0.95, "p95"), (0.99, "p99")]:
            v = float(np.quantile(rtts, q))
            fig.add_vline(x=v, line={"color": "red", "dash": "dot", "width": 1},
                          annotation_text=f"{label}={v:.1f}",
                          row=1, col=1)

    if streams.video:
        ia = [r["frame_interarrival_us"] / 1000 for r in streams.video
              if r["frame_interarrival_us"] > 0]
        if ia:
            fig.add_trace(go.Histogram(x=ia, nbinsx=80, name="ia",
                                        marker={"color": "#8c564b"}),
                          row=1, col=2)
            for q, label in [(0.5, "p50"), (0.95, "p95"), (0.99, "p99")]:
                v = float(np.quantile(ia, q))
                fig.add_vline(x=v, line={"color": "red", "dash": "dot", "width": 1},
                              annotation_text=f"{label}={v:.1f}",
                              row=1, col=2)

        warm = streams.video[100:] if len(streams.video) > 200 else streams.video
        drifts = [r["latency_drift_us"] / 1000 for r in warm]
        baseline = float(np.median(drifts)) if drifts else 0.0
        deltas = [d - baseline for d in drifts]
        if deltas:
            fig.add_trace(go.Histogram(x=deltas, nbinsx=80, name="drift",
                                        marker={"color": "#17becf"}),
                          row=1, col=3)
            for q, label in [(0.05, "p05"), (0.5, "p50"), (0.95, "p95")]:
                v = float(np.quantile(deltas, q))
                fig.add_vline(x=v, line={"color": "red", "dash": "dot", "width": 1},
                              annotation_text=f"{label}={v:.1f}",
                              row=1, col=3)

    if streams.verbose:
        fec = np.array([(r.get("signals_snapshot") or {}).get("fec_work") or 0
                        for r in streams.verbose])
        rl = np.array([(r.get("signals_snapshot") or {}).get("residual_loss_w") or 0
                       for r in streams.verbose])
        # Mark the ±5-tick neighbourhood of every residual_loss>0 sample
        # so the predictive cluster around real loss events stands out
        # visually against the background noise.
        near = np.zeros(len(fec), dtype=bool)
        for i in np.where(rl > 0)[0]:
            near[max(0, i - 5):min(len(fec), i + 6)] = True
        fig.add_trace(go.Scatter(
            x=fec[~near], y=rl[~near],
            mode="markers",
            marker={"size": 4, "color": "rgba(120,120,120,0.35)"},
            name="background tick",
        ), row=1, col=4)
        if near.any():
            fig.add_trace(go.Scatter(
                x=fec[near], y=rl[near],
                mode="markers",
                marker={"size": 7, "color": "#d62728",
                        "line": {"color": "#7a1014", "width": 1}},
                name="±5 ticks of loss",
            ), row=1, col=4)

    fig.update_xaxes(title_text="ms", row=1, col=1)
    fig.update_xaxes(title_text="ms", row=1, col=2)
    fig.update_xaxes(title_text="Δms vs baseline", row=1, col=3)
    fig.update_xaxes(title_text="fec_work", row=1, col=4)
    fig.update_yaxes(title_text="residual_loss", row=1, col=4)
    fig.update_layout(height=420,
                      legend={"orientation": "h", "y": -0.15, "x": 0.55},
                      margin={"l": 50, "r": 30, "t": 50, "b": 50})
    return fig


# ----------------------------------------------------------------------
# Rendering
# ----------------------------------------------------------------------

def _fmt_duration(seconds: float) -> str:
    if seconds < 60:
        return f"{seconds:.1f} s"
    m = int(seconds // 60)
    s = seconds % 60
    return f"{m} min {s:.1f} s"


def _summary_html(summary: dict) -> str:
    rows = []
    if "duration_s" in summary:
        rows.append(("Duration", _fmt_duration(summary["duration_s"])))
        rows.append(("Verbose ticks", str(summary["ticks"])))
    if summary.get("restarts"):
        rows.append((
            "GS service restarts",
            ", ".join(f"tick #{i} (−{g:.1f}s)" for i, g in summary["restarts"]),
        ))
    if "starved_pct" in summary:
        rows.append(("Starved windows", f"{summary['starved_pct']:.1f}%"))
    if "mcs_distribution" in summary:
        mcs = summary["mcs_distribution"]
        total = sum(mcs.values())
        parts = [f"MCS{k}={v} ({v/total*100:.0f}%)" for k, v in mcs.items()]
        rows.append(("MCS distribution", "&nbsp; ".join(parts)))
    if "rssi_percentiles" in summary:
        p = summary["rssi_percentiles"]
        rows.append(("RSSI dBm (alive)", f"p05={p[0]:.1f}, p50={p[1]:.1f}, p95={p[2]:.1f}"))
    if "snr_percentiles" in summary:
        p = summary["snr_percentiles"]
        rows.append(("SNR dB (alive)", f"p05={p[0]:.1f}, p50={p[1]:.1f}, p95={p[2]:.1f}"))
    rows.append(("Knob-change events", str(summary.get("events", 0))))
    if summary.get("emergencies"):
        rows.append(("Emergency events", str(summary["emergencies"])))
    if summary.get("mcs_changes"):
        rows.append(("MCS changes", str(summary["mcs_changes"])))
    if summary.get("idr_requests"):
        rows.append(("IDR requests (controller)",
                     str(len(summary["idr_requests"]))))
    if "rtt_ms" in summary:
        r = summary["rtt_ms"]
        rows.append((
            "RTT ms",
            f"p50={r['p50']:.1f}, p95={r['p95']:.1f}, p99={r['p99']:.1f}, max={r['max']:.1f}",
        ))
    if "latency_outliers" in summary:
        rows.append(("RTT outliers", str(summary["latency_outliers"])))
    if "video" in summary:
        v = summary["video"]
        rows.append(("Video frames", str(v["frames"])))
        rows.append(("Video loss", f"{v['loss_pct']:.3f}%"))
        ssrcs_str = ", ".join(v["ssrcs"])
        if len(v["ssrcs"]) > 1:
            ssrcs_str += " ⚠ encoder restarted"
        rows.append(("Video SSRCs", ssrcs_str))
    if "frame_interarrival_ms" in summary:
        f = summary["frame_interarrival_ms"]
        rows.append((
            "Frame interarrival ms",
            f"p50={f['p50']:.1f}, p95={f['p95']:.1f}, p99={f['p99']:.1f}, max={f['max']:.1f}",
        ))
    if "drift_ms" in summary:
        d = summary["drift_ms"]
        rows.append((
            "Drift ms (after warmup)",
            f"p05={d['p05']:.1f}, p50={d['p50']:.1f}, p95={d['p95']:.1f}, range={d['range']:.1f}",
        ))
    if summary.get("drone_events"):
        rows.append(("Drone failure events", str(summary["drone_events"])))

    out = ['<table class="summary"><tbody>']
    for k, v in rows:
        out.append(f"<tr><th>{html.escape(k)}</th><td>{v}</td></tr>")
    out.append("</tbody></table>")
    return "\n".join(out)


def _anomalies_html(anomalies: list[dict]) -> str:
    if not anomalies:
        return "<p><em>No video data — anomaly detection skipped.</em></p>"
    out = ['<table class="anomalies"><thead>']
    out.append("<tr><th>#</th><th>t (s)</th><th>verdict</th>"
               "<th>score</th><th>RTT factor</th>"
               "<th>drift excursion</th><th>worst ia (ms)</th>"
               "<th>lost</th></tr></thead><tbody>")
    for i, a in enumerate(anomalies, 1):
        c = classify_anomaly(a)
        sev = c["severity"]
        sev_class = {
            "high": ' class="sev-high"',
            "medium": ' class="sev-med"',
            "low": ' class="sev-low"',
        }.get(sev, "")
        out.append(
            f"<tr{sev_class}><td>{i}</td>"
            f"<td>{a['ts_rel_s']:.2f}</td>"
            f"<td>{html.escape(c['verdict'])}</td>"
            f"<td>{a['score']:.2f}</td>"
            f"<td>{a['rtt_factor']:.2f}×</td>"
            f"<td>{a['max_drift_excursion']:.1f}σ</td>"
            f"<td>{a['max_interarrival_ms']:.1f}</td>"
            f"<td>{a['lost']}</td></tr>"
        )
    out.append("</tbody></table>")
    return "\n".join(out)


def _diagnosis_html(diagnosis: dict, top_n: int = 8) -> str:
    """Plain-language verdict counts + emergency / IDR lists.
    The per-event "why / felt" blocks moved onto the timeline
    (hover the diamond markers) — see `make_timeline_figure`."""
    out = []
    counts = diagnosis["verdict_counts"]
    if counts:
        out.append("<p>Anomaly classification across the top windows:</p>")
        out.append('<ul class="diagnosis">')
        for verdict, n in counts.items():
            out.append(f"<li><strong>{n}× {html.escape(verdict)}</strong></li>")
        out.append("</ul>")
    else:
        out.append("<p>No anomalies above threshold. Flight looks clean.</p>")

    if diagnosis["emergencies"]:
        out.append(f"<p>Plus <strong>{len(diagnosis['emergencies'])} "
                   f"emergency events</strong> in the controller log:</p>")
        out.append('<ul class="emergencies">')
        for r in diagnosis["emergencies"]:
            out.append(f"<li>t={r['timestamp']:.2f} (wall): "
                       f"<code>{html.escape(r.get('reason',''))}</code></li>")
        out.append("</ul>")

    if diagnosis.get("idr_requests"):
        out.append(f"<p>Controller requested <strong>"
                   f"{len(diagnosis['idr_requests'])} IDR keyframes</strong> "
                   f"(FEC ramp-up / emergency recovery). Encoder-emitted "
                   f"IDRs are not in this list &mdash; they show up "
                   f"indirectly as oversized video frames.</p>")
        out.append('<ul class="idr-requests">')
        for r in diagnosis["idr_requests"]:
            out.append(f"<li>t={r['ts_rel_s']:.2f}s: "
                       f"<code>{html.escape(r.get('reason',''))}</code></li>")
        out.append("</ul>")

    return "\n".join(out)


HTML_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>{title}</title>
<style>
  body {{ font-family: ui-sans-serif, system-ui, sans-serif;
         max-width: 1400px; margin: 1em auto; padding: 0 1em;
         color: #222; }}
  h1 {{ margin-bottom: 0.2em; }}
  h2 {{ margin-top: 2em; border-bottom: 1px solid #ddd; padding-bottom: 4px; }}
  table.summary, table.anomalies {{ border-collapse: collapse;
         font-size: 0.9em; }}
  table.summary th {{ text-align: right; padding: 4px 12px 4px 0;
         color: #666; font-weight: normal; vertical-align: top;
         min-width: 180px; }}
  table.summary td {{ padding: 4px 0; }}
  table.anomalies {{ width: 100%; margin-top: 1em; }}
  table.anomalies th, table.anomalies td {{ padding: 4px 8px;
         text-align: right; border-bottom: 1px solid #eee; }}
  table.anomalies th {{ background: #f4f4f4; }}
  table.anomalies td:nth-child(1) {{ color: #888; }}
  table.anomalies tr.sev-high td {{ background: #fff3f3; }}
  table.anomalies tr.sev-med  td {{ background: #fff8e8; }}
  .meta {{ color: #888; font-size: 0.85em; }}
  ul.diagnosis li {{ margin-bottom: 4px; }}
  ul.emergencies code {{ font-size: 0.85em; }}
  div.event {{ border-left: 4px solid #ccc; padding: 4px 12px;
              margin: 12px 0; background: #fafafa; }}
  div.event.sev-hig {{ border-left-color: #d62728; background: #fff3f3; }}
  div.event.sev-med {{ border-left-color: #ff7f0e; background: #fff8e8; }}
  div.event.sev-low {{ border-left-color: #ccc; }}
  div.event h4 {{ margin: 0 0 6px 0; }}
  div.event h4 .severity {{ color: #999; font-weight: normal;
                           font-size: 0.85em; }}
  div.event ul {{ margin: 4px 0; }}
  div.event p {{ margin: 4px 0; }}
</style>
</head>
<body>
<h1>{title}</h1>
<p class="meta">Generated by dl-report from {bundle_path}</p>

<h2>TL;DR — diagnosis</h2>
{diagnosis_html}

<h2>Summary</h2>
{summary_table}

<h2>Intended vs observed lag</h2>
{lag_html}

<h2>Anomaly leaderboard (top {top_n})</h2>
<p class="meta">Score = drift_excursion × interarrival_factor × (lost+1) × rtt_factor.
Verdict is heuristic — see <code>classify_anomaly</code> for thresholds.</p>
{anomaly_table}

<h2>Linked timeline (zoom and pan stay in sync across rows)</h2>
{timeline_html}

<h2>Distributions</h2>
{distribution_html}

</body>
</html>
"""


# ----------------------------------------------------------------------
# Markdown rendering — agent-friendly, no JS, parseable as plain text.
# ----------------------------------------------------------------------

def _md_summary(summary: dict) -> str:
    out = ["## Summary stats", ""]
    rows = []
    if "duration_s" in summary:
        rows.append(("Duration", _fmt_duration(summary["duration_s"])))
        rows.append(("Verbose ticks", str(summary["ticks"])))
    if summary.get("restarts"):
        rows.append((
            "GS service restarts",
            ", ".join(f"tick #{i} (-{g:.1f}s)" for i, g in summary["restarts"]),
        ))
    if "starved_pct" in summary:
        rows.append(("Starved windows", f"{summary['starved_pct']:.1f}%"))
    if "mcs_distribution" in summary:
        m = summary["mcs_distribution"]
        total = sum(m.values()) or 1
        rows.append((
            "MCS distribution",
            "; ".join(f"MCS{k}={v} ({v/total*100:.0f}%)"
                      for k, v in m.items()),
        ))
    if "rssi_percentiles" in summary:
        p = summary["rssi_percentiles"]
        rows.append(("RSSI dBm (alive)",
                     f"p05={p[0]:.1f}, p50={p[1]:.1f}, p95={p[2]:.1f}"))
    if "snr_percentiles" in summary:
        p = summary["snr_percentiles"]
        rows.append(("SNR dB (alive)",
                     f"p05={p[0]:.1f}, p50={p[1]:.1f}, p95={p[2]:.1f}"))
    rows.append(("Knob-change events", str(summary.get("events", 0))))
    if summary.get("emergencies"):
        rows.append(("Emergency events", str(summary["emergencies"])))
    if summary.get("mcs_changes"):
        rows.append(("MCS changes", str(summary["mcs_changes"])))
    if summary.get("idr_requests"):
        rows.append(("IDR requests (controller)",
                     str(len(summary["idr_requests"]))))
    if "rtt_ms" in summary:
        r = summary["rtt_ms"]
        rows.append((
            "RTT ms",
            f"p50={r['p50']:.1f}, p95={r['p95']:.1f}, "
            f"p99={r['p99']:.1f}, max={r['max']:.1f}",
        ))
    if "latency_outliers" in summary:
        rows.append(("RTT outliers", str(summary["latency_outliers"])))
    if "video" in summary:
        v = summary["video"]
        rows.append(("Video frames", str(v["frames"])))
        rows.append(("Video loss", f"{v['loss_pct']:.3f}%"))
        ssrcs = ", ".join(v["ssrcs"])
        if len(v["ssrcs"]) > 1:
            ssrcs += " ⚠ encoder restarted"
        rows.append(("Video SSRCs", ssrcs))
    if "frame_interarrival_ms" in summary:
        f = summary["frame_interarrival_ms"]
        rows.append((
            "Frame interarrival ms",
            f"p50={f['p50']:.1f}, p95={f['p95']:.1f}, "
            f"p99={f['p99']:.1f}, max={f['max']:.1f}",
        ))
    if "drift_ms" in summary:
        d = summary["drift_ms"]
        rows.append((
            "Drift ms (after warmup)",
            f"p05={d['p05']:.1f}, p50={d['p50']:.1f}, "
            f"p95={d['p95']:.1f}, range={d['range']:.1f}",
        ))
    if summary.get("drone_events"):
        rows.append(("Drone failure events", str(summary["drone_events"])))
    out.append("| Metric | Value |")
    out.append("|---|---|")
    for k, v in rows:
        out.append(f"| {k} | {v} |")
    out.append("")
    return "\n".join(out)


def _md_diagnosis(diagnosis: dict) -> str:
    out = ["## TL;DR — diagnosis", ""]
    counts = diagnosis["verdict_counts"]
    if not counts:
        out.append("No anomalies above threshold. Flight looks clean.")
        out.append("")
        return "\n".join(out)
    out.append("Anomaly classification across the top windows:")
    out.append("")
    for verdict, n in counts.items():
        out.append(f"- **{n}× {verdict}**")
    out.append("")
    if diagnosis["emergencies"]:
        out.append(f"Plus **{len(diagnosis['emergencies'])} emergency events** "
                   f"in the controller log:")
        out.append("")
        for r in diagnosis["emergencies"]:
            out.append(f"- t={r['timestamp']:.2f} (wall): "
                       f"`{r.get('reason','')}`")
        out.append("")
    if diagnosis.get("idr_requests"):
        out.append(f"Controller requested **{len(diagnosis['idr_requests'])} "
                   f"IDR keyframes** (FEC ramp-up after loss; emergency "
                   f"recovery). Encoder-emitted IDRs are not in this list — "
                   f"they show up indirectly as oversized video frames.")
        out.append("")
        for r in diagnosis["idr_requests"]:
            out.append(f"- t={r['ts_rel_s']:.2f}s: `{r.get('reason','')}`")
        out.append("")
    return "\n".join(out)


def _md_anomaly_table(anomalies: list[dict], diagnosis: dict) -> str:
    out = ["## Anomaly leaderboard (full)", ""]
    if not anomalies:
        out.append("(no anomalies)")
        out.append("")
        return "\n".join(out)
    # Pre-build a verdict map for the full list
    verdicts = {id(a): classify_anomaly(a)["verdict"] for a in anomalies}
    out.append("| # | t (s) | verdict | RTT × | drift MAD | worst ia ms | lost |")
    out.append("|---|---|---|---|---|---|---|")
    for i, a in enumerate(anomalies, 1):
        out.append(
            f"| {i} | {a['ts_rel_s']:.2f} | {verdicts[id(a)]} "
            f"| {a['rtt_factor']:.2f} "
            f"| {a['max_drift_excursion']:.1f} "
            f"| {a['max_interarrival_ms']:.1f} "
            f"| {a['lost']} |"
        )
    out.append("")
    return "\n".join(out)


def compute_lag(verbose: list[dict]) -> dict:
    """Pair intended-knob changes with first matching observed value.

    For each tracked knob, walks `verbose` once and emits a list of
    pairing events. Each entry has a `lag_ms` and a `matched` flag —
    matched=False means observation never caught up before the next
    intent change (or before the log ended), and `lag_ms` then carries
    the pending lag at close-out.

    The mapping below is `intent_key_in_row -> observed_key_in_block`.
    They differ on the FEC knobs because the verbose row uses `k`/`n`
    while the wfb-ng `SessionInfo` exposes `fec_k`/`fec_n`.
    """
    knob_map = {"mcs": "mcs", "k": "fec_k", "n": "fec_n"}

    has_observed = any(row.get("observed") for row in verbose)
    if not has_observed:
        return {}

    result: dict[str, dict] = {}
    for intent_key, obs_key in knob_map.items():
        events: list[dict] = []
        prev_intent = None
        pending: dict | None = None
        last_ts: float | None = None
        for row in verbose:
            ts = row.get("timestamp")
            if ts is None:
                continue
            last_ts = ts
            cur_intent = row.get(intent_key)
            obs_block = row.get("observed") or {}
            cur_observed = obs_block.get(obs_key)

            if prev_intent is not None and cur_intent != prev_intent:
                if pending is not None:
                    events.append({
                        **pending,
                        "lag_ms": (ts - pending["t_intent_s"]) * 1000.0,
                        "matched": False,
                    })
                pending = {
                    "t_intent_s": ts,
                    "intent_old": prev_intent,
                    "intent_new": cur_intent,
                }
            prev_intent = cur_intent

            if pending is not None and cur_observed == pending["intent_new"]:
                events.append({
                    **pending,
                    "lag_ms": (ts - pending["t_intent_s"]) * 1000.0,
                    "matched": True,
                })
                pending = None

        if pending is not None and last_ts is not None:
            events.append({
                **pending,
                "lag_ms": (last_ts - pending["t_intent_s"]) * 1000.0,
                "matched": False,
            })

        matched = [e["lag_ms"] for e in events if e["matched"]]
        result[obs_key] = {
            "events": events,
            "n_paired": len(matched),
            "n_unmatched": sum(1 for e in events if not e["matched"]),
            "p50": st.median(matched) if matched else None,
            "p95": (st.quantiles(matched, n=20)[-1]
                    if len(matched) >= 20 else
                    (max(matched) if matched else None)),
            "max": max(matched) if matched else None,
        }
    return result


def _md_lag(lag: dict) -> str:
    total = sum(v["n_paired"] + v["n_unmatched"] for v in lag.values())
    if total == 0:
        return ""  # no observed data — skip the section entirely
    out = ["## Intended vs observed lag", ""]
    out.append("Lag between when the controller's intended knob "
               "changed and when the first observed RX-side change "
               "reflects the new value. In observer mode the values "
               "reflect intent-vs-reality decoupling rather than "
               "reconfig latency.")
    out.append("")
    out.append("| knob | paired | unmatched | p50 ms | p95 ms | max ms |")
    out.append("|---|---:|---:|---:|---:|---:|")
    for knob in ("mcs", "fec_k", "fec_n"):
        v = lag.get(knob)
        if v is None:
            continue
        def f(x):
            return f"{x:.1f}" if isinstance(x, (int, float)) else "—"
        out.append(f"| {knob} | {v['n_paired']} | {v['n_unmatched']} | "
                   f"{f(v['p50'])} | {f(v['p95'])} | {f(v['max'])} |")
    out.append("")

    # Outliers and unmatched: list the worst 5 + any unmatched events.
    for knob in ("mcs", "fec_k", "fec_n"):
        v = lag.get(knob)
        if v is None or not v["events"]:
            continue
        outliers = sorted(
            (e for e in v["events"] if e["matched"]),
            key=lambda e: -e["lag_ms"],
        )[:5]
        unmatched = [e for e in v["events"] if not e["matched"]]
        if not outliers and not unmatched:
            continue
        out.append(f"### {knob} — outliers and unmatched")
        out.append("")
        out.append("| t (s) | old → new | lag ms | matched |")
        out.append("|---:|---|---:|:---:|")
        for e in outliers + unmatched:
            mark = "✓" if e["matched"] else "—"
            out.append(f"| {e['t_intent_s']:.2f} "
                       f"| {e['intent_old']} → {e['intent_new']} "
                       f"| {e['lag_ms']:.1f} | {mark} |")
        out.append("")
    return "\n".join(out)


def render_markdown(streams: Streams, axis: TimeAxis, summary: dict,
                    anomalies: list[dict], diagnosis: dict,
                    bundle_path: Path, out_path: Path) -> None:
    lag = compute_lag(streams.verbose)
    parts = [
        f"# Flight report — {bundle_path.name}",
        "",
        f"_Generated by `dl-report` from `{bundle_path}`._",
        "",
        _md_diagnosis(diagnosis),
        _md_summary(summary),
        _md_lag(lag),
        _md_anomaly_table(anomalies, diagnosis),
    ]
    out_path.write_text("\n".join(parts))


# ----------------------------------------------------------------------
# HTML rendering
# ----------------------------------------------------------------------

def _lag_html(lag: dict) -> str:
    total = sum(v["n_paired"] + v["n_unmatched"] for v in lag.values())
    if total == 0:
        return ("<p class='meta'>No observed RX-side data in this bundle. "
                "Re-run with a GS that emits the <code>observed</code> "
                "block in <code>gs.verbose.jsonl</code>.</p>")
    rows = [
        "<table>",
        "<thead><tr><th>knob</th><th>paired</th><th>unmatched</th>"
        "<th>p50 ms</th><th>p95 ms</th><th>max ms</th></tr></thead>",
        "<tbody>",
    ]
    def f(x):
        return f"{x:.1f}" if isinstance(x, (int, float)) else "—"
    for knob in ("mcs", "fec_k", "fec_n"):
        v = lag.get(knob)
        if v is None:
            continue
        rows.append(
            f"<tr><td>{knob}</td><td>{v['n_paired']}</td>"
            f"<td>{v['n_unmatched']}</td><td>{f(v['p50'])}</td>"
            f"<td>{f(v['p95'])}</td><td>{f(v['max'])}</td></tr>"
        )
    rows.append("</tbody></table>")
    parts = ["<p class='meta'>Lag between when the controller's intended "
             "knob changed and when the first observed RX-side change "
             "reflects the new value. In observer mode the values reflect "
             "intent-vs-reality decoupling rather than reconfig latency.</p>",
             "\n".join(rows)]

    for knob in ("mcs", "fec_k", "fec_n"):
        v = lag.get(knob)
        if v is None or not v["events"]:
            continue
        outliers = sorted(
            (e for e in v["events"] if e["matched"]),
            key=lambda e: -e["lag_ms"],
        )[:5]
        unmatched = [e for e in v["events"] if not e["matched"]]
        if not outliers and not unmatched:
            continue
        parts.append(f"<h3>{knob} — outliers and unmatched</h3>")
        sub = ["<table>",
               "<thead><tr><th>t (s)</th><th>old → new</th>"
               "<th>lag ms</th><th>matched</th></tr></thead>",
               "<tbody>"]
        for e in outliers + unmatched:
            mark = "✓" if e["matched"] else "—"
            sub.append(
                f"<tr><td>{e['t_intent_s']:.2f}</td>"
                f"<td>{e['intent_old']} → {e['intent_new']}</td>"
                f"<td>{e['lag_ms']:.1f}</td><td>{mark}</td></tr>"
            )
        sub.append("</tbody></table>")
        parts.append("\n".join(sub))
    return "\n".join(parts)


def render_html(streams: Streams, axis: TimeAxis, summary: dict,
                anomalies: list[dict], diagnosis: dict,
                bundle_path: Path,
                out_path: Path, *, plotlyjs: str = "inline") -> None:
    timeline = make_timeline_figure(streams, axis, diagnosis=diagnosis)
    dist = make_distribution_figure(streams)
    lag = compute_lag(streams.verbose)

    timeline_html = timeline.to_html(
        full_html=False, include_plotlyjs=plotlyjs, div_id="timeline")
    dist_html = dist.to_html(
        full_html=False, include_plotlyjs=False, div_id="distribution")

    body = HTML_TEMPLATE.format(
        title=f"Flight report — {bundle_path.name}",
        bundle_path=html.escape(str(bundle_path)),
        diagnosis_html=_diagnosis_html(diagnosis),
        summary_table=_summary_html(summary),
        lag_html=_lag_html(lag),
        top_n=len(anomalies),
        anomaly_table=_anomalies_html(anomalies),
        timeline_html=timeline_html,
        distribution_html=dist_html,
    )
    out_path.write_text(body)


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------

def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="dl-report")
    p.add_argument("--bundle", required=True, type=Path,
                   help="Bundle dir or tarball")
    p.add_argument("--drone-events", type=Path,
                   help="Optional drone-side dl-events.jsonl")
    p.add_argument("-o", "--output", type=Path, default=Path("report.html"),
                   help="Output path. Extension picks default format if "
                        "--format not given (.md → markdown, else html). "
                        "With --format both, this is the HTML path and the "
                        "markdown gets the same stem with .md extension.")
    p.add_argument("--format", choices=("html", "markdown", "both"),
                   default=None,
                   help="Output format. Default: inferred from -o extension.")
    p.add_argument("--top-n", type=int, default=20,
                   help="Anomaly leaderboard size (default 20)")
    p.add_argument("--cdn", action="store_true",
                   help="Reference plotly.js via CDN instead of inlining "
                        "(smaller HTML, requires net to view)")
    p.add_argument("--log-level", default="INFO")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    # Resolve format from --format or -o extension.
    fmt = args.format
    if fmt is None:
        fmt = "markdown" if args.output.suffix.lower() in (".md", ".markdown") \
              else "html"

    log.info("loading bundle %s", args.bundle)
    streams, _tmp = load_bundle(args.bundle, args.drone_events)
    log.info("loaded: events=%d verbose=%d latency=%d video=%d drone=%d",
             len(streams.events), len(streams.verbose),
             len(streams.latency), len(streams.video), len(streams.drone))

    axis = TimeAxis.build(streams)
    summary = compute_summary(streams, axis)
    anomalies = compute_anomalies(streams, axis, top_n=args.top_n)
    diagnosis = compute_diagnosis(streams, axis, anomalies)

    plotlyjs = "cdn" if args.cdn else "inline"
    if fmt in ("html", "both"):
        html_path = args.output
        if fmt == "both" and html_path.suffix.lower() not in (".html", ".htm"):
            html_path = html_path.with_suffix(".html")
        render_html(streams, axis, summary, anomalies, diagnosis,
                    args.bundle, html_path, plotlyjs=plotlyjs)
        log.info("wrote %s (%.1f KB)",
                 html_path, html_path.stat().st_size / 1024)
    if fmt in ("markdown", "both"):
        md_path = args.output if fmt == "markdown" \
            else args.output.with_suffix(".md")
        render_markdown(streams, axis, summary, anomalies, diagnosis,
                        args.bundle, md_path)
        log.info("wrote %s (%.1f KB)",
                 md_path, md_path.stat().st_size / 1024)
    return 0


if __name__ == "__main__":
    sys.exit(main())
