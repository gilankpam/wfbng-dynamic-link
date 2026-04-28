"""Tests for gs/tools/dl_replay.py."""
from __future__ import annotations

import asyncio
import json
import socket
from pathlib import Path

import pytest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gs"))

from tools import dl_replay  # noqa: E402
from dynamic_link.wire import decode_pong  # noqa: E402  (only used to confirm tests can find wire too)
from dynamic_link import wire  # noqa: E402


def _verbose_record(seq: int, *, timestamp: float, mcs: int = 5,
                    bandwidth: int = 20, tx: int = 18,
                    k: int = 8, n: int = 14, depth: int = 2,
                    bitrate: int = 12000, idr: bool = False) -> dict:
    return {
        "timestamp": timestamp,
        "mcs": mcs, "bandwidth": bandwidth, "tx_power_dBm": tx,
        "k": k, "n": n, "depth": depth, "bitrate_kbps": bitrate,
        "idr_request": idr,
        "reason": "test",
        "knobs_changed": [],
        "signals_snapshot": {},
        # Phase-3 extras the LogSink may stamp on top — round-tripper
        # must tolerate them.
        "offset_us": 12345, "offset_stddev_us": 42,
    }


def _write_jsonl(path: Path, records: list[dict]) -> None:
    with open(path, "w") as fd:
        for r in records:
            fd.write(json.dumps(r) + "\n")


@pytest.mark.asyncio
async def test_replay_sends_decoded_decisions(tmp_path: Path):
    src = tmp_path / "verbose.jsonl"
    _write_jsonl(src, [
        _verbose_record(1, timestamp=10.000),
        _verbose_record(2, timestamp=10.100, mcs=4, bitrate=10000),
        _verbose_record(3, timestamp=10.200, depth=1),
    ])

    # Bind a UDP socket to receive the replayed packets.
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.setblocking(False)
    port = rx.getsockname()[1]

    try:
        rc = await dl_replay._replay(dl_replay.parse_args([
            "--source", str(src),
            "--target", f"127.0.0.1:{port}",
            "--speed", "100.0",     # blast through the cadence
        ]))
        assert rc == 0
        # Drain the socket.
        await asyncio.sleep(0.05)
        received = []
        while True:
            try:
                buf, _ = rx.recvfrom(4096)
                received.append(buf)
            except BlockingIOError:
                break
        assert len(received) == 3
        # Each packet should be a valid decision wire frame.
        for b in received:
            assert wire.peek_kind(b) == "decision"
    finally:
        rx.close()


@pytest.mark.asyncio
async def test_replay_respects_from_until_filters(tmp_path: Path):
    src = tmp_path / "verbose.jsonl"
    _write_jsonl(src, [
        _verbose_record(1, timestamp=5.0),
        _verbose_record(2, timestamp=10.0),
        _verbose_record(3, timestamp=15.0),
        _verbose_record(4, timestamp=20.0),
    ])

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.setblocking(False)
    port = rx.getsockname()[1]
    try:
        rc = await dl_replay._replay(dl_replay.parse_args([
            "--source", str(src),
            "--target", f"127.0.0.1:{port}",
            "--speed", "1000.0",
            "--from-ts", "9.0",
            "--until-ts", "16.0",
        ]))
        assert rc == 0
        await asyncio.sleep(0.05)
        n = 0
        while True:
            try:
                rx.recvfrom(4096)
                n += 1
            except BlockingIOError:
                break
        # records at 10.0 and 15.0 only.
        assert n == 2
    finally:
        rx.close()


@pytest.mark.asyncio
async def test_replay_skips_bad_records(tmp_path: Path):
    src = tmp_path / "verbose.jsonl"
    with open(src, "w") as fd:
        fd.write("not json\n")
        fd.write(json.dumps(_verbose_record(1, timestamp=0)) + "\n")
        fd.write("{\"missing\": \"fields\"}\n")
        fd.write(json.dumps(_verbose_record(2, timestamp=0.001)) + "\n")

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.setblocking(False)
    port = rx.getsockname()[1]
    try:
        rc = await dl_replay._replay(dl_replay.parse_args([
            "--source", str(src),
            "--target", f"127.0.0.1:{port}",
            "--speed", "1000.0",
        ]))
        assert rc == 0
        await asyncio.sleep(0.05)
        n = 0
        while True:
            try:
                rx.recvfrom(4096)
                n += 1
            except BlockingIOError:
                break
        assert n == 2
    finally:
        rx.close()
