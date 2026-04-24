"""Phase 2 "turns on" end-to-end test.

Wires together:
  - Real `dl-applier` binary (built by Phase 1).
  - Mock `wfb_tx` + mock encoder HTTP from test_drone_e2e.py.
  - FakeMavlinkSink from test_drone_e2e.py.
  - GS-side wire.Encoder + return_link.ReturnLink, sending real
    decision packets over UDP to the applier's listen port.

Does NOT spin up the full GS service — that requires a live stats
feed. This test proves the GS → drone wire is byte-compatible end
to end: a decision encoded by wire.py is decoded by the applier
and dispatched to all three backends.
"""
from __future__ import annotations

import time
from pathlib import Path

import pytest

from dynamic_link.decision import Decision
from dynamic_link.return_link import ReturnLink
from dynamic_link.wire import Encoder

# Reuse fixtures + helpers from the Phase 1 e2e module.
from tests.test_drone_e2e import _sandbox, _wait_until


def _decision(**overrides) -> Decision:
    base = Decision(
        timestamp=0.0,
        mcs=5, bandwidth=20, tx_power_dBm=18,
        k=8, n=14, depth=2,
        bitrate_kbps=12000,
        idr_request=False,
    )
    for k, v in overrides.items():
        setattr(base, k, v)
    return base


def test_gs_wire_drives_drone_applier(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        rl = ReturnLink(s["listen_addr"], s["listen_port"])
        enc = Encoder()

        decision = _decision()
        packet = enc.encode(decision)
        assert rl.send(packet)

        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        assert _wait_until(lambda: len(s["encoder"].recorded) >= 1)

        cmds = {r["cmd_id"]: r for r in s["wfb"].received}
        assert cmds[1]["k"] == 8  # CMD_SET_FEC
        assert cmds[1]["n"] == 14
        assert cmds[5]["depth"] == 2  # CMD_SET_INTERLEAVE_DEPTH
        assert cmds[2]["mcs_index"] == 5  # CMD_SET_RADIO
        assert cmds[2]["short_gi"] is False  # pinned

        paths = s["encoder"].recorded
        assert any("video0.bitrate=12000" in p for p in paths), paths

        rl.close()


def test_gs_wire_idr_flag_triggers_idr_request(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        rl = ReturnLink(s["listen_addr"], s["listen_port"])
        enc = Encoder()

        decision = _decision(idr_request=True)
        rl.send(enc.encode(decision))

        assert _wait_until(lambda: any(
            p == "/request/idr" for p in s["encoder"].recorded
        ), timeout_s=1.0), s["encoder"].recorded

        rl.close()


def test_gs_wire_sequence_dedup(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        rl = ReturnLink(s["listen_addr"], s["listen_port"])
        enc = Encoder()

        first = _decision()
        pkt1 = enc.encode(first, sequence=42)
        rl.send(pkt1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        n_after_first = len(s["wfb"].received)

        # Replay same sequence — applier should dedup.
        rl.send(pkt1)
        time.sleep(0.2)
        assert len(s["wfb"].received) == n_after_first

        # Higher sequence — applier accepts (different knobs so
        # backends emit again).
        pkt2 = enc.encode(_decision(mcs=6, k=8, n=12, depth=1),
                          sequence=43)
        rl.send(pkt2)
        assert _wait_until(
            lambda: len(s["wfb"].received) > n_after_first,
            timeout_s=1.0,
        )

        rl.close()


def test_gs_wire_monotonic_sequence_counter(tmp_path: Path):
    """Encoder's default counter auto-increments — successive
    send()s should produce accepted-in-order packets."""
    with _sandbox(tmp_path) as s:
        rl = ReturnLink(s["listen_addr"], s["listen_port"])
        enc = Encoder()

        for i in range(3):
            rl.send(enc.encode(_decision(mcs=3 + i, k=8, n=12, depth=1)))
            time.sleep(0.05)

        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        # Last MCS in wfb_tx should match the last sent.
        radio_cmds = [r for r in s["wfb"].received if r["cmd_id"] == 2]
        assert radio_cmds[-1]["mcs_index"] == 5

        rl.close()
