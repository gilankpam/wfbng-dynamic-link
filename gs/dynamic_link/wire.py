"""Phase 2 — decision-packet serialiser.

Byte-for-byte mirror of `drone/src/dl_wire.c`. The authority is the
C implementation; this module must match it exactly. The test at
`tests/test_wire_contract.py` cross-checks by running
`drone/build/dl-inject --dry-run` and diffing its hex output against
what this module produces for the same inputs.

Wire layout (big-endian, 32 bytes on-wire = 28 payload + 4 CRC32):

    off  size  field
     0    4    magic       = 0x444C4B31 ('DLK1')
     4    1    version     = 1
     5    1    flags
     6    2    _pad
     8    4    sequence
    12    4    timestamp_ms
    16    1    mcs
    17    1    bandwidth
    18    1    tx_power_dBm (signed int8)
    19    1    k
    20    1    n
    21    1    depth
    22    2    bitrate_kbps
    24    1    roi_qp
    25    1    fps
    26    2    _pad2
    28    4    crc32(bytes[0..27])
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from .decision import Decision

MAGIC            = 0x444C4B31    # 'DLK1'
VERSION          = 1
PAYLOAD_SIZE     = 28
ON_WIRE_SIZE     = 32
FLAG_IDR_REQUEST = 0x01


def _crc32(data: bytes) -> int:
    """Reflected CRC-32 (IEEE 802.3 / zlib-compatible) — same as
    `dl_wire_crc32` in the C implementation. We use Python's binascii
    for speed; the C version computes the same polynomial bit-by-bit.
    """
    import binascii
    return binascii.crc32(data) & 0xFFFFFFFF


@dataclass
class Encoder:
    """Stateful encoder with a monotonic sequence counter.

    Start at `seq_start` (default 1; 0 is reserved as the "never seen"
    sentinel on the drone side's dedup logic, though any value works
    — the drone simply stores the first-seen sequence and dedups
    against it).
    """
    seq: int = 1

    def encode(
        self,
        decision: Decision,
        *,
        idr_request: bool | None = None,
        timestamp_ms: int | None = None,
        sequence: int | None = None,
    ) -> bytes:
        """Serialise one Decision to the 32-byte on-wire form.

        `idr_request` / `timestamp_ms` / `sequence` override the
        corresponding Decision fields if provided; useful for tests
        and for the contract harness. Otherwise `idr_request` comes
        from `decision.idr_request`, `sequence` from the internal
        counter (which is then post-incremented), and `timestamp_ms`
        defaults to the sequence value (matches dl-inject behaviour
        when no explicit timestamp is supplied — the drone doesn't
        parse it meaningfully).
        """
        if sequence is None:
            sequence = self.seq
            self.seq = (self.seq + 1) & 0xFFFFFFFF
        if timestamp_ms is None:
            timestamp_ms = sequence
        if idr_request is None:
            idr_request = bool(decision.idr_request)

        flags = FLAG_IDR_REQUEST if idr_request else 0
        return _encode_raw(
            version=VERSION,
            flags=flags,
            sequence=sequence,
            timestamp_ms=timestamp_ms,
            mcs=int(decision.mcs),
            bandwidth=int(decision.bandwidth),
            tx_power_dBm=int(decision.tx_power_dBm),
            k=int(decision.k),
            n=int(decision.n),
            depth=int(decision.depth),
            bitrate_kbps=int(decision.bitrate_kbps),
            roi_qp=0,  # policy engine doesn't set ROI yet
            fps=0,     # ditto for fps (sentinel 0 = "leave alone")
        )


def _encode_raw(
    *,
    version: int,
    flags: int,
    sequence: int,
    timestamp_ms: int,
    mcs: int,
    bandwidth: int,
    tx_power_dBm: int,
    k: int,
    n: int,
    depth: int,
    bitrate_kbps: int,
    roi_qp: int,
    fps: int,
) -> bytes:
    payload = bytearray(PAYLOAD_SIZE)
    struct.pack_into(">I", payload, 0, MAGIC)
    payload[4] = version & 0xFF
    payload[5] = flags & 0xFF
    # [6..7] = _pad
    struct.pack_into(">I", payload, 8,  sequence & 0xFFFFFFFF)
    struct.pack_into(">I", payload, 12, timestamp_ms & 0xFFFFFFFF)
    payload[16] = mcs & 0xFF
    payload[17] = bandwidth & 0xFF
    payload[18] = tx_power_dBm & 0xFF     # int8 two's complement
    payload[19] = k & 0xFF
    payload[20] = n & 0xFF
    payload[21] = depth & 0xFF
    struct.pack_into(">H", payload, 22, bitrate_kbps & 0xFFFF)
    payload[24] = roi_qp & 0xFF
    payload[25] = fps & 0xFF
    # [26..27] = _pad2
    crc = _crc32(bytes(payload))
    return bytes(payload) + struct.pack(">I", crc)


def encode(decision: Decision, sequence: int) -> bytes:
    """Stateless convenience — encode with an explicit sequence."""
    return Encoder(seq=sequence).encode(decision, sequence=sequence)
