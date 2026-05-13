"""Phase-3 end-to-end smoke test.

Drives the *real* drone applier (configured with debug_enable=1 and
a tmp dbg_log_dir) via the *real* dl-replay tool, then asserts the
SD failure log contains the expected ENC_RESPONSE_BAD record when
the mock encoder returns HTTP 500.

This is the closure of the user's stated workflow:
  1. Replay a captured GS verbose.jsonl into a bench drone.
  2. Compare the bench-side dl-events.jsonl against a flight log.
  3. dl-events-diff surfaces behavioural divergence.

We don't actually compare against a flight log here; we just verify
the loop is wired end-to-end.
"""
from __future__ import annotations

import asyncio
import json
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

# Reuse fixtures from the existing e2e harness.
from tests.test_drone_e2e import (
    APPLIER,
    INJECT,
    FakeMavlinkSink,
    FakeWfbTx,
    build_drone,            # noqa: F401  (autouse session fixture)
)

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gs"))
from tools import dl_replay  # noqa: E402


class _500Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        body = b'{"error":"intentional 500"}'
        self.send_response(500)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
        self.server.recorded.append(self.path)  # type: ignore[attr-defined]

    def log_message(self, *a, **kw):
        pass


def _make_500_encoder() -> HTTPServer:
    srv = HTTPServer(("127.0.0.1", 0), _500Handler)
    srv.recorded = []  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def _verbose_record(*, timestamp: float, mcs: int = 5, k: int = 8,
                    n: int = 14, depth: int = 2,
                    bitrate: int = 12000, tx: int = 18) -> dict:
    return {
        "timestamp": timestamp,
        "mcs": mcs, "bandwidth": 20, "tx_power_dBm": tx,
        "k": k, "n": n, "depth": depth,
        "bitrate_kbps": bitrate, "idr_request": False,
        "reason": "test", "knobs_changed": [],
        "signals_snapshot": {},
    }


@pytest.mark.asyncio
async def test_replay_to_bench_captures_encoder_500(tmp_path: Path):
    """Full loop: replay → applier → mock encoder returns 500 →
    dl-events.jsonl on tmp 'SD' contains ENC_RESPONSE_BAD."""
    wfb = FakeWfbTx()
    wfb.start()
    enc = _make_500_encoder()
    enc_port = enc.server_address[1]
    mav = FakeMavlinkSink()
    mav.start()

    sd_dir = tmp_path / "sdcard"
    sd_dir.mkdir()

    # Pick a free port for the applier's UDP listen.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    listen_port = s.getsockname()[1]
    s.close()

    cfg = tmp_path / "drone.conf"
    cfg.write_text(
        f"listen_addr = 127.0.0.1\n"
        f"listen_port = {listen_port}\n"
        f"wfb_tx_ctrl_addr = 127.0.0.1\n"
        f"wfb_tx_ctrl_port = {wfb.port}\n"
        f"encoder_host = 127.0.0.1\n"
        f"encoder_port = {enc_port}\n"
        f"encoder_kind = majestic\n"
        f"osd_enable = 0\n"
        f"min_idr_interval_ms = 500\n"
        f"health_timeout_ms = 5000\n"
        f"wlan_dev = dl-nonexistent0\n"
        f"safe_k = 8\nsafe_n = 12\nsafe_depth = 1\n"
        f"safe_mcs = 1\nsafe_bandwidth = 20\n"
        f"safe_tx_power_dBm = 20\nsafe_bitrate_kbps = 2000\n"
        f"mavlink_enable = 1\nmavlink_addr = 127.0.0.1\n"
        f"mavlink_port = {mav.port}\nmavlink_sysid = 250\nmavlink_compid = 191\n"
        f"debug_enable = 1\n"
        f"dbg_log_dir = {sd_dir}\n"
        f"dbg_max_bytes = 65536\n"
    )

    proc = subprocess.Popen(
        [str(APPLIER), "--config", str(cfg), "--debug"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    # Wait for the applier to bind.
    deadline = time.monotonic() + 2.0
    bound = False
    while time.monotonic() < deadline:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("127.0.0.1", listen_port))
            probe.close()
            time.sleep(0.02)
        except OSError:
            probe.close()
            bound = True
            break
    assert bound, "dl-applier did not bind"

    try:
        # Build a tiny verbose.jsonl: two ticks, both set bitrate so
        # the applier hits the encoder both times.
        src = tmp_path / "verbose.jsonl"
        with open(src, "w") as fd:
            fd.write(json.dumps(_verbose_record(timestamp=0.0)) + "\n")
            fd.write(json.dumps(_verbose_record(timestamp=0.05,
                                                bitrate=10000)) + "\n")

        # Replay them to the applier as fast as possible.
        rc = await dl_replay._replay(dl_replay.parse_args([
            "--source", str(src),
            "--target", f"127.0.0.1:{listen_port}",
            "--speed", "100.0",
        ]))
        assert rc == 0

        # Give the applier a moment to process and write to SD.
        await asyncio.sleep(0.5)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        wfb.stop()
        mav.stop()
        enc.shutdown()
        enc.server_close()

    # Inspect the SD log.
    events_file = sd_dir / "dl-events.jsonl"
    assert events_file.exists(), "applier didn't create dl-events.jsonl"
    lines = [json.loads(l) for l in events_file.read_text().splitlines() if l]
    reasons = [r["reason"] for r in lines]
    # The mock encoder always returns 500 → at least one
    # ENC_RESPONSE_BAD record.
    assert "ENC_RESPONSE_BAD" in reasons
    bad = [r for r in lines if r["reason"] == "ENC_RESPONSE_BAD"][0]
    assert bad["detail"]["http"] == 500
    assert "intentional 500" in bad["detail"]["body"]
