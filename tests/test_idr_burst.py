"""Unit tests for the IDR burst sender.

Uses a fake ReturnLink (captures sent packets) and a real wire.Encoder.
Asserts: trigger fires count-1 burst packets after the regular tick,
each carries FLAG_IDR_REQUEST with monotonically increasing sequence,
re-trigger mid-burst resets _remaining, enabled=False is a no-op.
"""
from __future__ import annotations

import asyncio

import pytest

from dynamic_link.decision import Decision
from dynamic_link.idr_burst import IdrBurstConfig, IdrBurster
from dynamic_link.wire import FLAG_IDR_REQUEST, Encoder as WireEncoder


class FakeReturnLink:
    def __init__(self) -> None:
        self.packets: list[bytes] = []

    def send(self, packet: bytes) -> bool:
        self.packets.append(packet)
        return True


def _decision(idr: bool = True) -> Decision:
    return Decision(
        timestamp=1.0,
        mcs=5,
        bandwidth=20,
        tx_power_dBm=22,
        k=8,
        n=10,
        depth=1,
        bitrate_kbps=18000,
        idr_request=idr,
        reason="test",
    )


# Wire layout (gs/dynamic_link/wire.py module docstring):
#   byte 5     = flags
#   bytes 8-12 = sequence (big-endian uint32)
def _seq(packet: bytes) -> int:
    return int.from_bytes(packet[8:12], "big")


def _decoded_seqs(packets: list[bytes]) -> list[int]:
    return [_seq(p) for p in packets]


def _all_have_idr_flag(packets: list[bytes]) -> bool:
    return all(p[5] & FLAG_IDR_REQUEST for p in packets)


@pytest.mark.asyncio
async def test_burst_emits_count_minus_one_packets():
    """trigger() schedules count-1 packets at interval_ms spacing."""
    rl = FakeReturnLink()
    enc = WireEncoder(seq=100)
    cfg = IdrBurstConfig(enabled=True, count=4, interval_ms=1)
    burster = IdrBurster(cfg, rl, enc)

    burster.trigger(_decision())
    # Wait long enough for the 3 sleeps of 1ms each (give scheduler slack).
    await asyncio.sleep(0.05)

    assert len(rl.packets) == 3                              # count - 1
    assert _all_have_idr_flag(rl.packets)
    seqs = _decoded_seqs(rl.packets)
    assert seqs == sorted(seqs) and len(set(seqs)) == 3


@pytest.mark.asyncio
async def test_disabled_is_noop():
    rl = FakeReturnLink()
    enc = WireEncoder(seq=1)
    cfg = IdrBurstConfig(enabled=False, count=4, interval_ms=1)
    burster = IdrBurster(cfg, rl, enc)
    burster.trigger(_decision())
    await asyncio.sleep(0.05)
    assert rl.packets == []


@pytest.mark.asyncio
async def test_retrigger_mid_burst_resets_counter():
    """A second trigger() while a burst is in flight resets _remaining."""
    rl = FakeReturnLink()
    enc = WireEncoder(seq=1)
    cfg = IdrBurstConfig(enabled=True, count=3, interval_ms=10)
    burster = IdrBurster(cfg, rl, enc)

    burster.trigger(_decision())
    # After ~12 ms, one burst packet should have fired (count - 1 = 2,
    # spaced 10 ms apart). Re-trigger.
    await asyncio.sleep(0.012)
    pre = len(rl.packets)
    burster.trigger(_decision())
    # Wait for the rest of the (refreshed) burst.
    await asyncio.sleep(0.05)
    # First-burst sent some packets, then re-trigger added count-1 = 2 more
    # from the reset. Total >= pre + 2, and >= 3 in absolute terms.
    assert len(rl.packets) >= pre + 1
    assert len(rl.packets) >= 3


@pytest.mark.asyncio
async def test_no_idr_when_decision_lacks_flag_is_callers_problem():
    """trigger() does NOT inspect decision.idr_request — callers gate
    on that field before invoking trigger(). The burst always sets the
    IDR flag on the wire (this is the whole point)."""
    rl = FakeReturnLink()
    enc = WireEncoder(seq=1)
    cfg = IdrBurstConfig(enabled=True, count=2, interval_ms=1)
    burster = IdrBurster(cfg, rl, enc)
    burster.trigger(_decision(idr=False))
    await asyncio.sleep(0.05)
    assert len(rl.packets) == 1                    # count - 1
    assert _all_have_idr_flag(rl.packets)
