"""Tests for gs/tools/dl_review.py."""
from __future__ import annotations

import json
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gs"))

from tools import dl_review  # noqa: E402


def _write_jsonl(path: Path, records: list[dict]) -> None:
    with open(path, "w") as fd:
        for r in records:
            fd.write(json.dumps(r) + "\n")


def _make_bundle_dir(tmp_path: Path) -> Path:
    """Build a synthetic flight bundle directory."""
    d = tmp_path / "bundle"
    d.mkdir()
    _write_jsonl(d / "gs.jsonl", [
        {"timestamp": 100.000, "mcs": 5, "bandwidth": 20,
         "tx_power_dBm": 18, "k": 8, "n": 14, "depth": 2,
         "bitrate_kbps": 12000, "idr_request": False,
         "reason": "boot", "knobs_changed": ["mcs", "k", "n"],
         "signals_snapshot": {}},
    ])
    _write_jsonl(d / "gs.verbose.jsonl", [
        {"timestamp": 99.900, "mcs": 5, "bandwidth": 20,
         "tx_power_dBm": 18, "k": 8, "n": 14, "depth": 2,
         "bitrate_kbps": 12000, "idr_request": False,
         "reason": "tick", "knobs_changed": [], "signals_snapshot": {}},
        {"timestamp": 100.000, "mcs": 5, "bandwidth": 20,
         "tx_power_dBm": 18, "k": 8, "n": 14, "depth": 2,
         "bitrate_kbps": 12000, "idr_request": False,
         "reason": "boot", "knobs_changed": ["mcs"], "signals_snapshot": {}},
    ])
    _write_jsonl(d / "latency.jsonl", [
        {"ts_gs_mono_us": 100_000_000, "ts_gs_wall_us": 1, "gs_seq": 1,
         "rtt_us": 4500, "drone_mono_recv_us": 50_000_000,
         "drone_mono_send_us": 50_000_050,
         "offset_us": 50_000_000, "offset_stddev_us": 100,
         "outlier": False},
    ])
    _write_jsonl(d / "video_rtp.jsonl", [
        {"ts_gs_mono_us": 99_950_000, "ts_gs_wall_us": 1,
         "rtp_seq_first": 100, "rtp_seq_last": 102, "rtp_ts": 1234,
         "ssrc": "0xdeadbeef", "packets": 3, "expected": 3,
         "lost_in_frame": 0, "latency_drift_us": 200,
         "frame_interarrival_us": 16667},
    ])
    return d


def test_unified_timeline_merges_all_streams(tmp_path: Path, capsys):
    bundle = _make_bundle_dir(tmp_path)
    drone = tmp_path / "dl-events.jsonl"
    _write_jsonl(drone, [
        {"t": 50_000_000, "seq": 0, "sev": "warn",
         "reason": "ENC_RESPONSE_BAD",
         "detail": {"http": 500, "body": "oops"}},
    ])
    rc = dl_review.main([
        "--bundle", str(bundle),
        "--drone-events", str(drone),
    ])
    assert rc == 0
    out = capsys.readouterr().out
    # Each source should appear at least once.
    for src in ("gs", "gs.verbose", "latency", "video", "drone"):
        assert f"[{src}" in out
    # Drone event was timeline-translated: drone_t=50_000_000 +
    # offset=50_000_000 → gs_mono=100_000_000 → 100.000 seconds.
    assert "100.000000" in out
    assert "ENC_RESPONSE_BAD" in out


def test_around_window_filters(tmp_path: Path, capsys):
    bundle = _make_bundle_dir(tmp_path)
    rc = dl_review.main([
        "--bundle", str(bundle),
        "--around", "100.0",
        "--window", "0.01",
    ])
    assert rc == 0
    out = capsys.readouterr().out
    # The video frame at 99.95 is outside the ±0.01 window
    assert "video" not in out
    # The 100.000 events stay.
    assert "[gs " in out


def test_source_filter(tmp_path: Path, capsys):
    bundle = _make_bundle_dir(tmp_path)
    rc = dl_review.main([
        "--bundle", str(bundle),
        "--sources", "latency",
    ])
    assert rc == 0
    out = capsys.readouterr().out
    assert "latency" in out
    assert "gs.verbose" not in out
    assert "video" not in out


def test_drone_event_without_offset_falls_back(tmp_path: Path, capsys):
    """If latency.jsonl is empty (ping_pong was off), drone events
    still print — just at the raw drone-mono timestamp."""
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "latency.jsonl").write_text("")  # empty
    drone = tmp_path / "dl-events.jsonl"
    _write_jsonl(drone, [
        {"t": 12345, "seq": 0, "sev": "warn",
         "reason": "WATCHDOG_TRIPPED", "detail": {}},
    ])
    rc = dl_review.main([
        "--bundle", str(bundle),
        "--drone-events", str(drone),
    ])
    assert rc == 0
    out = capsys.readouterr().out
    assert "WATCHDOG_TRIPPED" in out
    assert "no offset" in out


def test_tarball_input(tmp_path: Path):
    bundle = _make_bundle_dir(tmp_path)
    tarball = tmp_path / "flight.tar.gz"
    with tarfile.open(tarball, "w:gz") as tf:
        for f in bundle.iterdir():
            tf.add(f, arcname=f.name)
    rc = dl_review.main(["--bundle", str(tarball)])
    assert rc == 0


def test_dl_bundle_script_runs_against_a_log_dir(tmp_path: Path):
    """Smoke-test the shell wrapper. Builds a fake LOG_DIR with a
    couple of JSONL files and confirms the resulting tarball
    contains them."""
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    (log_dir / "gs.jsonl").write_text("{\"ts\":1}\n")
    (log_dir / "latency.jsonl").write_text("{\"ts\":2}\n")
    out_dir = tmp_path / "out"
    out_dir.mkdir()

    # Resolve the script path relative to the repo, not the cwd.
    script = Path(__file__).resolve().parent.parent / "gs" / "tools" / "dl-bundle"

    result = subprocess.run(
        [str(script), "flight-test"],
        env={"LOG_DIR": str(log_dir), "OUT_DIR": str(out_dir),
             "TAG": "flight-test", "PATH": "/usr/bin:/bin"},
        capture_output=True, text=True, check=True,
    )
    tar = out_dir / "flight-test.tar.gz"
    assert tar.exists()
    with tarfile.open(tar, "r:gz") as tf:
        names = set(tf.getnames())
    assert "gs.jsonl" in names
    assert "latency.jsonl" in names
    # video_rtp.jsonl wasn't present → not in tar
    assert "video_rtp.jsonl" not in names
