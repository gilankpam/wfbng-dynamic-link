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
from dynamic_link.wire import encode

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
