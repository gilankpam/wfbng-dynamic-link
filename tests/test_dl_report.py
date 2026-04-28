"""Tests for gs/tools/dl_report.py."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gs"))

from tools import dl_report  # noqa: E402


# ------------------------------------------------------------------
# Bundle fixture (richer than dl_review's — has drift / loss / RTT
# variation so analysis has something to chew on)
# ------------------------------------------------------------------

def _write_jsonl(path: Path, recs: list[dict]) -> None:
    with open(path, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")


def _make_bundle(tmp_path: Path) -> Path:
    d = tmp_path / "bundle"
    d.mkdir()

    # Verbose log: 60 ticks (6 s @ 10 Hz), MCS hopping, one starvation
    verbose = []
    events = []
    for i in range(60):
        ts = 100.0 + i * 0.1
        starved = 30 <= i < 35
        rssi = None if starved else -55.0 + (i % 5) * 2
        snr = None if starved else 28.0 + (i % 3)
        mcs = 5 if not starved else 0
        bitrate = 20800 if not starved else 2340
        knobs = []
        reason = ""
        if i == 30:
            knobs = ["mcs", "bitrate", "tx_power"]
            reason = "emergency starved mcs 5->0"
        elif i == 5:
            knobs = ["mcs", "bitrate", "tx_power"]
            reason = "mcs_up snr=30.0 4->5"
        rec = {
            "timestamp": ts,
            "mcs": mcs, "bandwidth": 20, "tx_power_dBm": 18,
            "k": 8, "n": 12, "depth": 1, "bitrate_kbps": bitrate,
            "idr_request": False, "reason": reason, "knobs_changed": knobs,
            "signals_snapshot": {
                "rssi": rssi, "snr": snr,
                "residual_loss_w": 0.0, "fec_work": 0.0,
                "packet_rate_w": 0 if starved else 4000,
                "link_starved_w": starved,
            },
        }
        verbose.append(rec)
        if knobs:
            events.append(rec)
    _write_jsonl(d / "gs.verbose.jsonl", verbose)
    _write_jsonl(d / "gs.jsonl", events)

    # Latency: 30 PONGs over 6s, with one outlier
    latency = []
    for i in range(30):
        ts_mono = 1_000_000 + i * 200_000  # 5 Hz
        rtt = 25_000 + (i * 100)  # mild trend
        if i == 14:
            rtt = 200_000  # outlier
        latency.append({
            "ts_gs_mono_us": ts_mono,
            "ts_gs_wall_us": int((100.0 + i * 0.2) * 1_000_000),
            "gs_seq": i,
            "rtt_us": rtt,
            "drone_mono_recv_us": ts_mono - 50_000_000,  # offset
            "drone_mono_send_us": ts_mono - 50_000_000 + 50,
            "offset_us": 50_000_000,
            "offset_stddev_us": 100,
            "outlier": i == 14,
        })
    _write_jsonl(d / "latency.jsonl", latency)

    # Video: 360 frames over 6s (60 fps), one drift spike at t=3s
    video = []
    for i in range(360):
        ts_mono = 1_000_000 + i * 16_667  # 60fps
        # Drift: stable at -100ms baseline, with a spike at i=180
        drift = -100_000
        ia = 16_667
        if 178 <= i <= 184:
            drift = -100_000 + (i - 180) * 30_000  # 30ms ramps each side
            ia = 30_000 if i == 180 else 16_667
        video.append({
            "ts_gs_mono_us": ts_mono,
            "ts_gs_wall_us": int((100.0 + i * 0.01667) * 1_000_000),
            "rtp_seq_first": i * 10,
            "rtp_seq_last": i * 10 + 9,
            "rtp_ts": i * 1500,
            "ssrc": "0x12345678",
            "packets": 10,
            "expected": 10,
            "lost_in_frame": 0,
            "latency_drift_us": drift,
            "frame_interarrival_us": ia if i > 0 else 0,
        })
    _write_jsonl(d / "video_rtp.jsonl", video)

    return d


# ------------------------------------------------------------------
# Loader / corruption tolerance
# ------------------------------------------------------------------

def test_load_bundle_dir(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    streams, _tmp = dl_report.load_bundle(bundle, None)
    assert len(streams.verbose) == 60
    assert len(streams.events) == 2
    assert len(streams.latency) == 30
    assert len(streams.video) == 360
    assert streams.drone == []


def test_load_jsonl_skips_bad_lines(tmp_path: Path):
    p = tmp_path / "broken.jsonl"
    p.write_text(
        '{"ok":1}\n'
        '\n'                              # empty — silently skipped
        '{"this is not json\n'           # invalid
        '{"ok":2}\n'
    )
    rows = dl_report._load_jsonl_resilient(p)
    assert len(rows) == 2
    assert rows[0]["ok"] == 1


# ------------------------------------------------------------------
# Time axis
# ------------------------------------------------------------------

def test_time_axis_wall_to_mono_interpolates(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    streams, _ = dl_report.load_bundle(bundle, None)
    axis = dl_report.TimeAxis.build(streams)

    # Pick a wall timestamp halfway between two latency samples and
    # verify wall_to_mono interpolates linearly
    L = streams.latency
    w0, w1 = L[5]["ts_gs_wall_us"], L[6]["ts_gs_wall_us"]
    m0, m1 = L[5]["ts_gs_mono_us"], L[6]["ts_gs_mono_us"]
    midw = (w0 + w1) // 2
    expected = (m0 + m1) // 2
    got = axis.wall_to_mono(midw)
    # Allow a few µs slop from int arithmetic
    assert abs(got - expected) < 100


def test_time_axis_falls_back_when_no_latency(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    # Wipe latency.jsonl
    (bundle / "latency.jsonl").write_text("")
    streams, _ = dl_report.load_bundle(bundle, None)
    axis = dl_report.TimeAxis.build(streams)
    # Should not crash; identity-ish behaviour
    assert axis.wall_to_mono(99999) == 99999


# ------------------------------------------------------------------
# Summary stats
# ------------------------------------------------------------------

def test_summary_captures_starvation_and_mcs(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    streams, _ = dl_report.load_bundle(bundle, None)
    axis = dl_report.TimeAxis.build(streams)
    s = dl_report.compute_summary(streams, axis)

    assert abs(s["starved_pct"] - (5/60*100)) < 0.1
    # MCS distribution should have both 0 and 5
    assert s["mcs_distribution"][5] == 55
    assert s["mcs_distribution"][0] == 5
    # 2 events, 1 emergency
    assert s["events"] == 2
    assert s["emergencies"] == 1


def test_summary_rtt_percentiles(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    streams, _ = dl_report.load_bundle(bundle, None)
    axis = dl_report.TimeAxis.build(streams)
    s = dl_report.compute_summary(streams, axis)
    # RTTs are 25–28 ms with one 200ms outlier
    assert 25 <= s["rtt_ms"]["p50"] <= 35
    assert s["rtt_ms"]["max"] == 200.0
    assert s["latency_outliers"] == 1


def test_summary_detects_gs_restart(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    # Mutate the verbose log: insert a backwards jump mid-file
    verbose_path = bundle / "gs.verbose.jsonl"
    lines = verbose_path.read_text().splitlines()
    rec = json.loads(lines[30])
    rec["timestamp"] = rec["timestamp"] - 60   # 60s regression
    lines[30] = json.dumps(rec)
    verbose_path.write_text("\n".join(lines) + "\n")

    streams, _ = dl_report.load_bundle(bundle, None)
    s = dl_report.compute_summary(streams, dl_report.TimeAxis.build(streams))
    assert len(s["restarts"]) == 1


# ------------------------------------------------------------------
# Anomaly leaderboard
# ------------------------------------------------------------------

def test_anomalies_surface_drift_spike(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    streams, _ = dl_report.load_bundle(bundle, None)
    axis = dl_report.TimeAxis.build(streams)
    rows = dl_report.compute_anomalies(streams, axis, top_n=5)
    # The synthetic spike is at frame 180 → t_mono ≈ 1s + 180*16.667ms = 4s
    assert rows, "expected at least one anomaly"
    top = rows[0]
    # Top anomaly's bucket should be near the spike (within 1s)
    spike_t_mono = 1_000_000 + 180 * 16_667
    spike_t_rel = axis.relative_seconds(spike_t_mono)
    assert abs(top["ts_rel_s"] - spike_t_rel) < 1.5
    assert top["max_drift_excursion"] > 1.0  # >1 MAD


def test_anomalies_empty_when_no_video(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    (bundle / "video_rtp.jsonl").write_text("")
    streams, _ = dl_report.load_bundle(bundle, None)
    axis = dl_report.TimeAxis.build(streams)
    rows = dl_report.compute_anomalies(streams, axis)
    assert rows == []


# ------------------------------------------------------------------
# Render: end-to-end smoke
# ------------------------------------------------------------------

def test_render_writes_self_contained_html(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    out = tmp_path / "report.html"
    rc = dl_report.main(["--bundle", str(bundle), "-o", str(out)])
    assert rc == 0
    assert out.exists()
    body = out.read_text()
    # Sanity-check the report mentions key facts
    assert "Flight report" in body
    assert "MCS distribution" in body
    assert "Anomaly leaderboard" in body
    # Plotly JS should be inlined (default)
    assert "plotly" in body.lower()
    # The synthetic data has ~360 video frames and 30 pongs
    assert "360" in body or "frames" in body.lower()


def test_render_cdn_mode_smaller(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    out_inline = tmp_path / "inline.html"
    out_cdn = tmp_path / "cdn.html"
    dl_report.main(["--bundle", str(bundle), "-o", str(out_inline)])
    dl_report.main(["--bundle", str(bundle), "-o", str(out_cdn), "--cdn"])
    # CDN form should be much smaller (no plotly.js inlined)
    assert out_cdn.stat().st_size < out_inline.stat().st_size / 2


def test_render_handles_missing_streams(tmp_path: Path):
    """Bundle with only verbose log — no latency, no video.
    Should still render a usable report."""
    d = tmp_path / "minimal"
    d.mkdir()
    _write_jsonl(d / "gs.verbose.jsonl", [{
        "timestamp": 100.0, "mcs": 1, "bandwidth": 20,
        "tx_power_dBm": 18, "k": 8, "n": 12, "depth": 1,
        "bitrate_kbps": 5200, "idr_request": False,
        "reason": "", "knobs_changed": [],
        "signals_snapshot": {
            "rssi": -55, "snr": 30, "residual_loss_w": 0,
            "fec_work": 0, "link_starved_w": False, "packet_rate_w": 1000,
        },
    }])
    out = tmp_path / "min.html"
    rc = dl_report.main(["--bundle", str(d), "-o", str(out)])
    assert rc == 0
    assert "No video data" in out.read_text() \
        or "anomaly detection skipped" in out.read_text().lower()


# ------------------------------------------------------------------
# Anomaly classification — the diagnostic layer
# ------------------------------------------------------------------

def _row(**overrides):
    """Fixture for classify_anomaly() inputs."""
    base = {
        "score": 1.0, "ts_mono_us": 0, "ts_rel_s": 0.0,
        "max_drift_excursion": 0.0,
        "max_interarrival_ms": 17.0,
        "lost": 0,
        "rtt_factor": 1.0,
        "ssrc_changed": False,
        "starved_ticks": 0,
    }
    base.update(overrides)
    return base


def test_classify_link_congestion():
    c = dl_report.classify_anomaly(_row(
        rtt_factor=4.7, max_drift_excursion=11.0, lost=10,
    ))
    assert c["verdict"] == "Link congestion"
    assert c["severity"] == "high"
    assert "RTT" in c["why"] and "drift" in c["why"]


def test_classify_encoder_pipeline():
    c = dl_report.classify_anomaly(_row(
        max_drift_excursion=29.0,
        max_interarrival_ms=117.5,
        lost=0,
        rtt_factor=1.1,
    ))
    assert c["verdict"] == "Encoder pipeline"
    assert c["severity"] == "medium"
    assert "downstream" in c["why"].lower() or "encoder" in c["why"].lower()


def test_classify_encoder_pipeline_with_loss():
    """Drift up + RTT normal + some packets lost = encoder fell behind
    and dropped frames to catch up. Real bench data hit this and used
    to fall through to 'unclassified' (rank #3 of the t=43 case)."""
    c = dl_report.classify_anomaly(_row(
        max_drift_excursion=9.8, rtt_factor=1.25, lost=14,
        max_interarrival_ms=40.1,
    ))
    assert c["verdict"] == "Encoder pipeline"
    # The loss note should be present in the explanation
    assert "drop" in c["why"].lower() or "skip" in c["why"].lower()


def test_classify_link_stress_when_fec_absorbed():
    """RTT and drift both elevated but no decoded loss — pilot still
    felt the late frames, but FEC saved the pixels. Real bench data
    used to fall through to 'unclassified' here; this test pins the
    fix."""
    c = dl_report.classify_anomaly(_row(
        rtt_factor=2.5, max_drift_excursion=29.0,
        max_interarrival_ms=117.5, lost=0,
    ))
    assert c["verdict"].startswith("Link stress")
    assert c["severity"] == "medium"
    assert "fec" in c["why"].lower()


def test_classify_drone_tx_silent():
    """Starved ticks override everything else — that's the strongest
    signal that the source has stopped."""
    c = dl_report.classify_anomaly(_row(
        starved_ticks=10,
        # Even with otherwise-fine metrics, starvation wins
        rtt_factor=1.0, max_drift_excursion=0.5,
    ))
    assert c["verdict"] == "Drone TX silent"
    assert c["severity"] == "high"


def test_classify_encoder_restart():
    c = dl_report.classify_anomaly(_row(ssrc_changed=True))
    assert c["verdict"] == "Encoder restart"


def test_classify_asymmetric_link():
    c = dl_report.classify_anomaly(_row(
        rtt_factor=3.5, max_drift_excursion=0.5, lost=0,
    ))
    assert c["verdict"] == "Asymmetric link"
    assert c["severity"] == "low"


def test_classify_random_rf_drop():
    c = dl_report.classify_anomaly(_row(
        rtt_factor=1.0, max_drift_excursion=0.5, lost=3,
    ))
    assert c["verdict"] == "Random RF drop"


def test_classify_single_frame_hiccup():
    c = dl_report.classify_anomaly(_row(
        max_interarrival_ms=80.0, lost=0,
        rtt_factor=1.1, max_drift_excursion=1.0,
    ))
    assert c["verdict"] == "Single-frame hiccup"


def test_classify_unclassified_fallback():
    c = dl_report.classify_anomaly(_row())
    assert "unclassified" in c["verdict"].lower() or c["verdict"].startswith("Mild")


def test_compute_diagnosis_aggregates_verdicts(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    streams, _ = dl_report.load_bundle(bundle, None)
    axis = dl_report.TimeAxis.build(streams)
    anomalies = dl_report.compute_anomalies(streams, axis)
    diag = dl_report.compute_diagnosis(streams, axis, anomalies)
    assert isinstance(diag["verdict_counts"], dict)
    assert sum(diag["verdict_counts"].values()) == len(anomalies)
    assert len(diag["classified"]) == len(anomalies)
    # The synthetic bundle has 1 emergency event
    assert len(diag["emergencies"]) == 1


# ------------------------------------------------------------------
# Markdown rendering
# ------------------------------------------------------------------

def test_markdown_format_via_extension(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    out = tmp_path / "report.md"
    rc = dl_report.main(["--bundle", str(bundle), "-o", str(out)])
    assert rc == 0
    body = out.read_text()
    # Should be plain markdown — no HTML tags
    assert "<html" not in body.lower()
    assert "<body" not in body.lower()
    # Required sections
    assert "# Flight report" in body
    assert "## TL;DR" in body
    assert "## Summary stats" in body
    assert "## Anomaly leaderboard" in body
    # Markdown table syntax present
    assert "| Metric | Value |" in body
    assert "|---|---|" in body


def test_markdown_format_explicit_flag(tmp_path: Path):
    """--format markdown overrides extension inference."""
    bundle = _make_bundle(tmp_path)
    out = tmp_path / "report.txt"   # extension says no
    rc = dl_report.main(["--bundle", str(bundle), "-o", str(out),
                         "--format", "markdown"])
    assert rc == 0
    body = out.read_text()
    assert "# Flight report" in body
    assert "<html" not in body.lower()


def test_format_both_writes_two_files(tmp_path: Path):
    bundle = _make_bundle(tmp_path)
    out = tmp_path / "flight.html"
    rc = dl_report.main(["--bundle", str(bundle), "-o", str(out),
                         "--format", "both"])
    assert rc == 0
    assert out.exists()
    md = out.with_suffix(".md")
    assert md.exists()
    assert "<html>" in out.read_text() or "<!DOCTYPE" in out.read_text()
    assert "# Flight report" in md.read_text()


def test_markdown_has_no_per_event_section(tmp_path: Path):
    """The per-event 'why / felt' blocks were moved onto the timeline
    diamond markers; the markdown body must not re-render them."""
    bundle = _make_bundle(tmp_path)
    out = tmp_path / "r.md"
    dl_report.main(["--bundle", str(bundle), "-o", str(out)])
    body = out.read_text()
    assert "events explained" not in body.lower()
    assert "**Why**" not in body
    assert "**What you probably felt**" not in body


def test_html_has_diagnosis_section(tmp_path: Path):
    """HTML should also surface the diagnosis up front, not just
    the data tables."""
    bundle = _make_bundle(tmp_path)
    out = tmp_path / "r.html"
    dl_report.main(["--bundle", str(bundle), "-o", str(out)])
    body = out.read_text()
    assert "TL;DR — diagnosis" in body
    # Verdict column should be present in the leaderboard
    assert "verdict" in body.lower()
