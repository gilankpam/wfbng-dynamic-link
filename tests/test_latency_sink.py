"""Tests for the latency.jsonl writer."""
from __future__ import annotations

import io
import json

from dynamic_link.latency_sink import LatencySink
from dynamic_link.timesync import LatencySample


def test_writes_one_record_per_sample():
    buf = io.StringIO()
    sink = LatencySink(buf)
    sink.write(LatencySample(
        gs_seq=1, ts_gs_mono_us=1_000_000,
        rtt_us=2500,
        drone_mono_recv_us=42, drone_mono_send_us=92,
        offset_us=10_000, offset_stddev_us=42,
        outlier=False,
    ))
    sink.write(LatencySample(
        gs_seq=2, ts_gs_mono_us=1_200_000,
        rtt_us=99000,
        drone_mono_recv_us=200_000, drone_mono_send_us=200_050,
        offset_us=10_000, offset_stddev_us=42,
        outlier=True,
    ))
    assert sink.stats["written"] == 2

    lines = buf.getvalue().rstrip("\n").split("\n")
    assert len(lines) == 2
    a = json.loads(lines[0])
    assert a["gs_seq"] == 1
    assert a["rtt_us"] == 2500
    assert a["offset_us"] == 10_000
    assert a["outlier"] is False
    assert a["ts_gs_wall_us"] > 0
    b = json.loads(lines[1])
    assert b["outlier"] is True
