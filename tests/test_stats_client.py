"""Tests for the JSON stats-API client parser + reconnect behaviour."""
from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from dynamic_link.stats_client import (
    ContractVersionError,
    ReplayClient,
    RxEvent,
    SessionEvent,
    SettingsEvent,
    TxEvent,
    iter_events_from_reader,
    parse_record,
)


def _rx_record(contract_version: int = 2) -> dict:
    return {
        "type": "rx",
        "timestamp": 1.0,
        "id": "video rx",
        "tx_wlan": 0,
        "packets": {
            "all": [100, 1000],
            "all_bytes": [140000, 1400000],
            "dec_err": [0, 0],
            "session": [1, 10],
            "data": [140, 1400],
            "uniq": [140, 1400],
            "fec_rec": [3, 30],
            "lost": [1, 10],
            "bad": [0, 0],
            "out": [99, 990],
            "out_bytes": [138600, 1386000],
            "bursts_rec": [0, 0],
            "holdoff": [0, 0],
            "late_deadline": [0, 0],
        },
        "rx_ant_stats": [
            {
                "ant": 1, "freq": 5765, "mcs": 7, "bw": 20, "pkt_recv": 140,
                "rssi_min": -70, "rssi_avg": -68, "rssi_max": -65,
                "snr_min": 20, "snr_avg": 22, "snr_max": 24,
            }
        ],
        "session": {
            "fec_type": "VDM_RS", "fec_k": 8, "fec_n": 12, "epoch": 1,
            "interleave_depth": 1, "contract_version": contract_version,
        },
    }


def test_parse_rx_record():
    ev = parse_record(_rx_record())
    assert isinstance(ev, RxEvent)
    # packets_window pulls just the window value (index 0).
    assert ev.packets_window["lost"] == 1
    assert ev.packets_window["out"] == 99
    assert ev.packets_window["fec_rec"] == 3
    assert len(ev.rx_ant_stats) == 1
    assert ev.rx_ant_stats[0].rssi_min == -70
    assert ev.session.fec_k == 8
    assert ev.session.contract_version == 2


def test_parse_rx_accepts_contract_version_1():
    """Vanilla wfb-ng emits contract_version=1; we must accept it."""
    ev = parse_record(_rx_record(contract_version=1))
    assert isinstance(ev, RxEvent)
    assert ev.session is not None
    assert ev.session.contract_version == 1


def test_parse_rx_rejects_unknown_contract_version():
    """Versions outside {1, 2} still raise."""
    with pytest.raises(ContractVersionError):
        parse_record(_rx_record(contract_version=99))


def test_parse_session_defaults_interleave_depth_when_missing():
    """Vanilla wfb-ng's session record omits interleave_depth — default
    it to 1 rather than raising KeyError."""
    raw = {
        "type": "rx",
        "timestamp": 1.0,
        "id": "video rx",
        "tx_wlan": 0,
        "packets": {"all": [10, 100]},
        "rx_ant_stats": [],
        "session": {
            "fec_type": "VDM_RS",
            "fec_k": 8, "fec_n": 12, "epoch": 1,
            "contract_version": 1,
            # interleave_depth deliberately absent
        },
    }
    ev = parse_record(raw)
    assert isinstance(ev, RxEvent)
    assert ev.session is not None
    assert ev.session.interleave_depth == 1


def test_parse_new_session():
    raw = {
        "type": "new_session",
        "timestamp": 2.0,
        "id": "video rx",
        "fec_type": "VDM_RS",
        "fec_k": 8, "fec_n": 14, "epoch": 2,
        "interleave_depth": 2, "contract_version": 2,
    }
    ev = parse_record(raw)
    assert isinstance(ev, SessionEvent)
    assert ev.session.fec_n == 14


def test_parse_new_session_rejects_bad_contract_version():
    raw = {
        "type": "new_session",
        "timestamp": 2.0, "id": "video rx",
        "fec_type": "VDM_RS", "fec_k": 8, "fec_n": 14, "epoch": 2,
        "interleave_depth": 2, "contract_version": 99,
    }
    with pytest.raises(ContractVersionError):
        parse_record(raw)


def test_parse_settings():
    raw = {
        "type": "settings",
        "profile": "gs", "is_cluster": False,
        "wlans": ["wlan0"],
        "settings": {"common": {"log_interval": 100}},
    }
    ev = parse_record(raw)
    assert isinstance(ev, SettingsEvent)
    assert ev.wlans == ["wlan0"]


def test_parse_tx_record():
    raw = {
        "type": "tx",
        "timestamp": 1.0,
        "id": "video tx",
        "packets": {"fec_timeouts": [0, 0], "incoming": [100, 1000]},
    }
    ev = parse_record(raw)
    assert isinstance(ev, TxEvent)
    assert ev.packets_window["incoming"] == 100


def test_parse_unknown_type_returns_none():
    assert parse_record({"type": "not_a_real_type"}) is None


async def test_iter_events_from_reader_reads_jsonl():
    lines = b"\n".join([
        json.dumps({"type": "settings", "profile": "gs", "is_cluster": False,
                    "wlans": [], "settings": {}}).encode(),
        json.dumps(_rx_record()).encode(),
        json.dumps(_rx_record()).encode(),
    ]) + b"\n"
    reader = asyncio.StreamReader()
    reader.feed_data(lines)
    reader.feed_eof()

    events = [ev async for ev in iter_events_from_reader(reader)]
    assert len(events) == 3
    assert isinstance(events[0], SettingsEvent)
    assert isinstance(events[1], RxEvent)
    assert isinstance(events[2], RxEvent)


async def test_iter_events_skips_malformed_lines():
    lines = b"\n".join([
        b"not valid json",
        json.dumps(_rx_record()).encode(),
    ]) + b"\n"
    reader = asyncio.StreamReader()
    reader.feed_data(lines)
    reader.feed_eof()
    events = [ev async for ev in iter_events_from_reader(reader)]
    assert len(events) == 1
    assert isinstance(events[0], RxEvent)


async def test_replay_client_consumes_file(tmp_path: Path):
    path = tmp_path / "capture.jsonl"
    with open(path, "w") as fd:
        fd.write(json.dumps({"type": "settings", "profile": "gs",
                             "is_cluster": False, "wlans": [],
                             "settings": {}}) + "\n")
        fd.write(json.dumps(_rx_record()) + "\n")
        fd.write(json.dumps(_rx_record()) + "\n")

    got = []
    client = ReplayClient(str(path), on_event=lambda ev: got.append(ev))
    await client.run()
    assert len(got) == 3
    assert isinstance(got[0], SettingsEvent)
    assert isinstance(got[1], RxEvent)
