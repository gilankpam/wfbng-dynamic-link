"""Wire-format contract test: GS Python encoder must agree byte-for-byte
with the drone C encoder (`drone/src/dl_wire.c`).

Runs `drone/build/dl-inject --dry-run` (which prints the encoded
bytes as hex to stdout) with the same inputs we pass to
`gs.dynamic_link.wire.encode`, and diffs.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from dynamic_link.decision import Decision
from dynamic_link.wire import Ping, encode, encode_ping
from dynamic_link.wire import (
    Hello,
    HelloAck,
    encode_hello,
    encode_hello_ack,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
DL_INJECT = REPO_ROOT / "drone" / "build" / "dl-inject"


@pytest.fixture(scope="module", autouse=True)
def ensure_dl_inject_built():
    if not DL_INJECT.exists():
        subprocess.run(
            ["make", "-C", str(REPO_ROOT / "drone")],
            check=True,
        )
    assert DL_INJECT.exists()


def _dl_inject_hex(**kwargs) -> bytes:
    """Call dl-inject --dry-run with named flags; return the 32 bytes."""
    args = [str(DL_INJECT), "--dry-run"]
    for k, v in kwargs.items():
        if isinstance(v, bool):
            if v:
                args.append(f"--{k.replace('_', '-')}")
        else:
            args.extend([f"--{k.replace('_', '-')}", str(v)])
    out = subprocess.check_output(args, text=True).strip()
    assert len(out) == 64, f"expected 64 hex chars, got {len(out)}: {out!r}"
    return bytes.fromhex(out)


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


def test_contract_golden_values():
    # Identical inputs → identical bytes.
    c_bytes = _dl_inject_hex(
        mcs=5, bandwidth=20, tx_power=18,
        k=8, n=14, depth=2, bitrate=12000, sequence=1,
    )
    py_bytes = encode(_decision(), sequence=1)
    assert c_bytes == py_bytes, (
        f"contract mismatch:\n  C : {c_bytes.hex()}\n  Py: {py_bytes.hex()}"
    )


def test_contract_idr_flag():
    c_bytes = _dl_inject_hex(
        mcs=7, bandwidth=40, tx_power=20,
        k=8, n=12, depth=1, bitrate=26000, sequence=42, idr=True,
    )
    py_bytes = encode(
        _decision(mcs=7, bandwidth=40, tx_power_dBm=20,
                  k=8, n=12, depth=1, bitrate_kbps=26000,
                  idr_request=True),
        sequence=42,
    )
    assert c_bytes == py_bytes


def test_contract_signed_tx_power():
    c_bytes = _dl_inject_hex(
        mcs=0, bandwidth=20, tx_power=-10,
        k=2, n=4, depth=1, bitrate=2000, sequence=7,
    )
    py_bytes = encode(
        _decision(mcs=0, bandwidth=20, tx_power_dBm=-10,
                  k=2, n=4, depth=1, bitrate_kbps=2000),
        sequence=7,
    )
    assert c_bytes == py_bytes
    # Byte 18 is two's complement -10 = 0xF6.
    assert py_bytes[18] == 0xF6


def test_contract_max_values():
    c_bytes = _dl_inject_hex(
        mcs=7, bandwidth=40, tx_power=30,
        k=8, n=16, depth=3, bitrate=65000, sequence=0xFFFFFFFF,
    )
    py_bytes = encode(
        _decision(mcs=7, bandwidth=40, tx_power_dBm=30,
                  k=8, n=16, depth=3, bitrate_kbps=65000),
        sequence=0xFFFFFFFF,
    )
    assert c_bytes == py_bytes


def test_contract_magic_and_version():
    py_bytes = encode(_decision(), sequence=1)
    # First 4 bytes = "DLK1"
    assert py_bytes[:4] == b"DLK1"
    # Byte 4 = version 1
    assert py_bytes[4] == 1


def _dl_inject_ping_hex(*, gs_seq: int, gs_mono_us: int) -> bytes:
    out = subprocess.check_output(
        [str(DL_INJECT), "--dry-run", "--ping",
         "--gs-seq", str(gs_seq),
         "--gs-mono", str(gs_mono_us)],
        text=True,
    ).strip()
    # PING is 24 bytes → 48 hex chars.
    assert len(out) == 48, f"expected 48 hex chars, got {len(out)}: {out!r}"
    return bytes.fromhex(out)


def test_contract_ping_zero():
    c = _dl_inject_ping_hex(gs_seq=0, gs_mono_us=0)
    py = encode_ping(Ping(gs_seq=0, gs_mono_us=0))
    assert c == py


def test_contract_ping_typical():
    c = _dl_inject_ping_hex(gs_seq=1234567, gs_mono_us=1_700_000_000_000_000)
    py = encode_ping(Ping(gs_seq=1234567, gs_mono_us=1_700_000_000_000_000))
    assert c == py


def test_contract_ping_max_values():
    c = _dl_inject_ping_hex(gs_seq=0xFFFFFFFF, gs_mono_us=(1 << 64) - 1)
    py = encode_ping(Ping(gs_seq=0xFFFFFFFF, gs_mono_us=(1 << 64) - 1))
    assert c == py


def test_contract_ping_magic():
    py = encode_ping(Ping(gs_seq=1, gs_mono_us=1))
    assert py[:4] == b"DLPG"
    assert py[4] == 1   # version


def test_contract_hello_basic():
    c_bytes = _dl_inject_hex(
        hello=True,
        gen_id="0xcafebabe",
        mtu=3994,
        fps=60,
        build_sha="0xdeadbeef",
    )
    py_bytes = encode_hello(
        Hello(generation_id=0xCAFEBABE,
              mtu_bytes=3994,
              fps=60,
              applier_build_sha=0xDEADBEEF)
    )
    assert c_bytes == py_bytes, (
        f"hello mismatch:\n  C : {c_bytes.hex()}\n  Py: {py_bytes.hex()}"
    )


def test_contract_hello_max_values():
    c_bytes = _dl_inject_hex(
        hello=True,
        gen_id="0xffffffff",
        mtu=65535,
        fps=65535,
        build_sha="0xffffffff",
    )
    py_bytes = encode_hello(
        Hello(generation_id=0xFFFFFFFF,
              mtu_bytes=0xFFFF,
              fps=0xFFFF,
              applier_build_sha=0xFFFFFFFF)
    )
    assert c_bytes == py_bytes


def test_contract_hello_min_values():
    c_bytes = _dl_inject_hex(
        hello=True, gen_id="0", mtu=1, fps=1, build_sha="0",
    )
    py_bytes = encode_hello(
        Hello(generation_id=0, mtu_bytes=1, fps=1, applier_build_sha=0)
    )
    assert c_bytes == py_bytes


def test_contract_hello_ack():
    c_bytes = _dl_inject_hex(
        hello_ack=True, gen_id="0x12345678",
    )
    py_bytes = encode_hello_ack(HelloAck(generation_id_echo=0x12345678))
    assert c_bytes == py_bytes


def test_hello_flag_vanilla_constant():
    from dynamic_link.wire import HELLO_FLAG_VANILLA_WFB_NG
    assert HELLO_FLAG_VANILLA_WFB_NG == 0x01


def test_contract_hello_vanilla_flag():
    """Round-trip a HELLO with the vanilla bit set; C and Python encoders
    must produce byte-identical output."""
    from dynamic_link.wire import HELLO_FLAG_VANILLA_WFB_NG
    c_bytes = _dl_inject_hex(
        hello=True,
        gen_id="0xcafebabe",
        mtu=3994,
        fps=60,
        build_sha="0xdeadbeef",
        hello_flags=f"0x{HELLO_FLAG_VANILLA_WFB_NG:02x}",
    )
    py_bytes = encode_hello(
        Hello(generation_id=0xCAFEBABE,
              mtu_bytes=3994,
              fps=60,
              applier_build_sha=0xDEADBEEF,
              flags=HELLO_FLAG_VANILLA_WFB_NG)
    )
    assert c_bytes == py_bytes, (
        f"hello-vanilla mismatch:\n  C : {c_bytes.hex()}\n  Py: {py_bytes.hex()}"
    )
    # Byte 5 is the flags byte.
    assert py_bytes[5] == HELLO_FLAG_VANILLA_WFB_NG
