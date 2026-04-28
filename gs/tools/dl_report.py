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
        b["max_drift_excursion"] = max(b["max_drift_excursion"], exc)
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
            "ts_mono_us": b["ts_mono_us"],
            "ts_rel_s": axis.relative_seconds(b["ts_mono_us"]),
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

    return {
        "verdict_counts": dict(sorted(counts.items())),
        "classified": classified,
        "emergencies": emerg,
    }


# ----------------------------------------------------------------------
# Plotting
# ----------------------------------------------------------------------

def _event_markers(streams: Streams, axis: TimeAxis) -> list[dict]:
    """High-signal events worth a vertical line on every panel:
    emergencies, watchdogs, drone-side failures. MCS up/down events
    are skipped intentionally — there are dozens per flight and they're
    already rendered as the MCS step trace in row 1."""
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


def make_timeline_figure(streams: Streams, axis: TimeAxis) -> go.Figure:
    """Six stacked subplots, shared x-axis."""
    fig = make_subplots(
        rows=6, cols=1, shared_xaxes=True,
        vertical_spacing=0.025,
        row_heights=[0.18, 0.16, 0.16, 0.16, 0.18, 0.16],
        subplot_titles=(
            "MCS / bitrate",
            "RSSI / SNR (alive windows)",
            "residual_loss / fec_work",
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

        # Row 3: loss / fec
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
                      row=3, col=1)
        fig.add_trace(go.Scatter(x=x, y=fec, mode="lines",
                                 line={"color": "#9467bd"}, name="fec_work"),
                      row=3, col=1)

    # Row 4: RTT (split outliers vs clean for color)
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
        ), row=4, col=1)
        if outlier.any():
            fig.add_trace(go.Scatter(
                x=x_lat[outlier], y=rtt_ms[outlier],
                mode="markers", marker={"size": 7, "color": "red", "symbol": "x"},
                name="RTT outlier",
            ), row=4, col=1)

    # Row 5: drift
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
        ), row=5, col=1)
        fig.add_hline(y=0, line={"width": 1, "dash": "dot",
                                 "color": "rgba(0,0,0,0.3)"},
                      row=5, col=1)

        # Row 6: interarrival
        ia_ms = np.array([r["frame_interarrival_us"] / 1000
                          for r in streams.video])
        ia_ms = np.where(ia_ms > 0, ia_ms, np.nan)
        fig.add_trace(go.Scatter(
            x=x_v, y=ia_ms, mode="lines",
            line={"color": "#8c564b", "width": 1},
            name="frame interarrival ms",
        ), row=6, col=1)
        # 60fps reference
        fig.add_hline(y=16.67, line={"width": 1, "dash": "dot",
                                     "color": "rgba(0,0,0,0.3)"},
                      row=6, col=1)

    # Vertical event markers — batched as a single shapes list update
    # rather than 6 × N add_vline calls (each triggers a layout
    # rebuild). On a 4-min flight with 6 emergencies that's the
    # difference between 30s and <1s.
    markers = _event_markers(streams, axis)
    shapes = []
    for m in markers:
        # `xref="x domain"` would lock to one subplot; we want full
        # vertical sweep, so use one shape per row with explicit yref.
        for axis_n in range(1, 7):
            yref = f"y{axis_n} domain" if axis_n > 1 else "y domain"
            xref = f"x{axis_n}" if axis_n > 1 else "x"
            shapes.append({
                "type": "line", "xref": xref, "yref": yref,
                "x0": m["x"], "x1": m["x"],
                "y0": 0, "y1": 1,
                "line": {"color": m["color"], "width": 1, "dash": "dash"},
            })

    fig.update_xaxes(title_text="Time (s, gs_mono since flight start)",
                     row=6, col=1)
    fig.update_yaxes(title_text="MCS", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="kbps", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="dBm", row=2, col=1, secondary_y=False)
    fig.update_yaxes(title_text="dB", row=2, col=1, secondary_y=True)
    fig.update_yaxes(title_text="rate", row=3, col=1)
    fig.update_yaxes(title_text="ms", row=4, col=1)
    fig.update_yaxes(title_text="Δ ms", row=5, col=1)
    fig.update_yaxes(title_text="ms", row=6, col=1)

    fig.update_layout(
        height=1200,
        margin={"l": 60, "r": 30, "t": 60, "b": 50},
        legend={"orientation": "h", "y": 1.04, "x": 0},
        hovermode="x unified",
        shapes=shapes,
    )
    return fig


def make_distribution_figure(streams: Streams) -> go.Figure:
    """Three histograms with p50/p95/p99 marked."""
    fig = make_subplots(
        rows=1, cols=3,
        subplot_titles=("RTT (ms)", "Frame interarrival (ms)",
                        "Latency drift (ms)"),
        horizontal_spacing=0.08,
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

    fig.update_xaxes(title_text="ms", row=1, col=1)
    fig.update_xaxes(title_text="ms", row=1, col=2)
    fig.update_xaxes(title_text="Δms vs baseline", row=1, col=3)
    fig.update_layout(height=420, showlegend=False,
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
    """Plain-language verdict + top events explained.
    Mirrors the markdown output but rendered with the page's CSS."""
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

    classified = diagnosis["classified"][:top_n]
    if classified:
        out.append(f"<h3>Top {len(classified)} events explained</h3>")
        for i, (a, c) in enumerate(classified, 1):
            sev_class = f"sev-{c['severity'][:3]}"   # high/med/low → hig/med/low
            out.append(f'<div class="event {sev_class}">')
            out.append(
                f"<h4>{i}. t={a['ts_rel_s']:.2f}s — "
                f"{html.escape(c['verdict'])} "
                f'<span class="severity">({c["severity"]})</span></h4>'
            )
            out.append("<ul>")
            metrics = (
                f"score={a['score']:.2f}, "
                f"RTT factor={a['rtt_factor']:.2f}×, "
                f"drift excursion={a['max_drift_excursion']:.1f} MAD, "
                f"worst interarrival={a['max_interarrival_ms']:.1f}ms, "
                f"lost={a['lost']}"
                + (f", starved_ticks={a['starved_ticks']}"
                   if a['starved_ticks'] else "")
                + (", ssrc_changed=true" if a['ssrc_changed'] else "")
            )
            out.append(f"<li>{html.escape(metrics)}</li>")
            out.append("</ul>")
            out.append(f"<p><strong>Why:</strong> {html.escape(c['why'])}</p>")
            out.append(f"<p><strong>What you probably felt:</strong> "
                       f"{html.escape(c['felt'])}</p>")
            out.append("</div>")
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
    return "\n".join(out)


def _md_per_event_diagnosis(diagnosis: dict, top_n: int = 8) -> str:
    out = [f"## Top {top_n} events explained", ""]
    classified = diagnosis["classified"][:top_n]
    if not classified:
        out.append("(no anomalies)")
        out.append("")
        return "\n".join(out)
    for i, (a, c) in enumerate(classified, 1):
        sev_marker = {"high": "🔴", "medium": "🟠", "low": "🟡"}.get(
            c["severity"], "⚪")
        out.append(f"### {i}. t={a['ts_rel_s']:.2f}s — {c['verdict']} "
                   f"{sev_marker} ({c['severity']})")
        out.append("")
        # Metric line — concise, agent-parseable
        out.append(
            f"- score={a['score']:.2f}, "
            f"RTT factor={a['rtt_factor']:.2f}×, "
            f"drift excursion={a['max_drift_excursion']:.1f} MAD, "
            f"worst interarrival={a['max_interarrival_ms']:.1f}ms, "
            f"lost={a['lost']}"
            + (f", starved_ticks={a['starved_ticks']}"
               if a['starved_ticks'] else "")
            + (", ssrc_changed=true" if a['ssrc_changed'] else "")
        )
        out.append("")
        out.append(f"**Why**: {c['why']}")
        out.append("")
        out.append(f"**What you probably felt**: {c['felt']}")
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


def render_markdown(streams: Streams, axis: TimeAxis, summary: dict,
                    anomalies: list[dict], diagnosis: dict,
                    bundle_path: Path, out_path: Path) -> None:
    parts = [
        f"# Flight report — {bundle_path.name}",
        "",
        f"_Generated by `dl-report` from `{bundle_path}`._",
        "",
        _md_diagnosis(diagnosis),
        _md_summary(summary),
        _md_per_event_diagnosis(diagnosis),
        _md_anomaly_table(anomalies, diagnosis),
    ]
    out_path.write_text("\n".join(parts))


# ----------------------------------------------------------------------
# HTML rendering
# ----------------------------------------------------------------------

def render_html(streams: Streams, axis: TimeAxis, summary: dict,
                anomalies: list[dict], diagnosis: dict,
                bundle_path: Path,
                out_path: Path, *, plotlyjs: str = "inline") -> None:
    timeline = make_timeline_figure(streams, axis)
    dist = make_distribution_figure(streams)

    timeline_html = timeline.to_html(
        full_html=False, include_plotlyjs=plotlyjs, div_id="timeline")
    dist_html = dist.to_html(
        full_html=False, include_plotlyjs=False, div_id="distribution")

    body = HTML_TEMPLATE.format(
        title=f"Flight report — {bundle_path.name}",
        bundle_path=html.escape(str(bundle_path)),
        diagnosis_html=_diagnosis_html(diagnosis),
        summary_table=_summary_html(summary),
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
