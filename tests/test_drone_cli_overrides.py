"""End-to-end tests for dl-applier per-field CLI overrides.

Layered config: built-in defaults -> --config drone.conf -> CLI flags.
These tests exercise the CLI override layer end-to-end against the
real dl-applier binary, the mock wfb_tx control sink, and the rest
of the _sandbox plumbing from test_drone_e2e.
"""
from __future__ import annotations

import socket
import subprocess
import time
from pathlib import Path

import pytest

from tests.test_drone_e2e import (
    APPLIER,
    CMD_SET_FEC,
    CMD_SET_INTERLEAVE_DEPTH,
    CMD_SET_RADIO,
    _sandbox,
    _inject,
    _wait_until,
    build_drone,  # noqa: F401 — autouse session fixture
)


def _wait_for_safe_radio(wfb, timeout: float = 10.0) -> dict:
    """Block until the mock wfb_tx sees a CMD_SET_RADIO from the
    watchdog's safe_defaults push. Returns the decoded request dict."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for m in wfb.received:
            if m.get("cmd_id") == CMD_SET_RADIO:
                return m
        time.sleep(0.05)
    raise TimeoutError(
        f"no safe_defaults CMD_SET_RADIO in {timeout}s; got {wfb.received!r}"
    )


# --------------------------------------------------------------------
# --help surface: in-scope flags listed, Phase-3 debug-suite excluded.
# --------------------------------------------------------------------

def test_help_lists_in_scope_fields():
    out = subprocess.run(
        [str(APPLIER), "--help"],
        capture_output=True, text=True, timeout=5,
    )
    # --help may print to stdout or stderr depending on impl; check both.
    text = out.stderr + out.stdout

    for flag in (
        "--safe-mcs", "--safe-bandwidth", "--safe-tx-power-dBm",
        "--mavlink-port", "--mavlink-enable", "--osd-debug-latency",
        "--encoder-host", "--gs-tunnel-port", "--hello-keepalive-ms",
        "--roi-qp-threshold-kbps", "--listen-port",
    ):
        assert flag in text, f"expected {flag} in --help output"


def test_help_excludes_phase3_debug_suite():
    out = subprocess.run(
        [str(APPLIER), "--help"],
        capture_output=True, text=True, timeout=5,
    )
    text = out.stderr + out.stdout
    for flag in (
        "--debug-enable", "--dbg-log-enable", "--dbg-log-dir",
        "--dbg-max-bytes", "--dbg-fsync-each",
    ):
        assert flag not in text, f"{flag} should NOT be in --help output"


# --------------------------------------------------------------------
# Validation: out-of-range and unknown flags exit non-zero.
# --------------------------------------------------------------------

def test_bad_value_exits_nonzero():
    """safe-mcs is u8 in [0..7]; 99 must be rejected."""
    out = subprocess.run(
        [str(APPLIER), "--safe-mcs", "99"],
        capture_output=True, text=True, timeout=5,
    )
    assert out.returncode != 0
    assert "safe-mcs" in (out.stderr + out.stdout).lower()


def test_unknown_flag_exits_nonzero():
    """Phase-3 debug-suite flags are intentionally unknown to the
    CLI parser; passing one must exit non-zero."""
    out = subprocess.run(
        [str(APPLIER), "--debug-enable"],
        capture_output=True, text=True, timeout=5,
    )
    assert out.returncode != 0


# --------------------------------------------------------------------
# End-to-end: CLI override beats conf-file value.
# --------------------------------------------------------------------

def test_cli_safe_mcs_overrides_conf(tmp_path):
    """The sandbox conf sets safe_mcs=1. We override it to 4 on the
    CLI, send one decision to arm the watchdog, then go silent until
    health_timeout_ms expires and verify the safe-push uses MCS=4."""
    with _sandbox(tmp_path, health_timeout_ms=500,
                  cli_args=["--safe-mcs", "4"]) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        # Arm the watchdog with one decision.
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        s["wfb"].received.clear()
        # Go silent; watchdog fires after health_timeout_ms.
        radio = _wait_for_safe_radio(s["wfb"])
        assert radio["mcs_index"] == 4, (
            f"expected MCS=4 from CLI override, got {radio['mcs_index']}; "
            f"all wfb_tx requests: {s['wfb'].received!r}"
        )


def test_cli_safe_bandwidth_overrides_conf(tmp_path):
    """Same idea for safe_bandwidth to confirm the override is
    field-specific (not just safe_mcs)."""
    with _sandbox(tmp_path, health_timeout_ms=500,
                  cli_args=["--safe-bandwidth", "40"]) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        # Arm the watchdog with one decision.
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        s["wfb"].received.clear()
        # Go silent; watchdog fires after health_timeout_ms.
        radio = _wait_for_safe_radio(s["wfb"])
        assert radio.get("bandwidth") == 40, (
            f"expected bandwidth=40 from CLI override, got {radio!r}"
        )


# --------------------------------------------------------------------
# End-to-end: --config is optional. Boot from defaults + CLI alone.
# --------------------------------------------------------------------

def _wait_for_bind(addr: str, port: int, timeout: float = 3.0) -> bool:
    """Return True once `addr:port` is bound (a fresh bind to it
    fails with EADDRINUSE)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind((addr, port))
            probe.close()
            time.sleep(0.05)
        except OSError:
            probe.close()
            return True
    return False


def test_dl_applier_boots_without_config():
    """No --config flag. CLI provides a free listen-port and disables
    the IDR socket; everything else uses built-in defaults. We only
    verify the applier binds its listen socket — defaults like
    wlan_dev=wlan0, encoder_host=127.0.0.1:80 will fail downstream,
    but those failures are non-fatal at boot."""
    # Reserve a free UDP port.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()

    proc = subprocess.Popen(
        [str(APPLIER),
         "--listen-addr", "127.0.0.1",
         "--listen-port", str(port),
         "--idr-listen-port", "0",
         "--mavlink-enable",   # set to true; default is also true, no-op
         "--encoder-port", "1",   # connect-refused fast on encoder
         "--debug"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    try:
        assert _wait_for_bind("127.0.0.1", port, timeout=3.0), (
            "dl-applier did not bind --listen-port without --config"
        )
        # Verify the process is still alive (didn't crash on boot).
        assert proc.poll() is None, (
            f"dl-applier exited early: rc={proc.poll()}, "
            f"stderr={proc.stderr.read(1024)!r}"
        )
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


# --------------------------------------------------------------------
# Boolean CLI overrides: --flag=true|false|1|0 (P-bool-cli-args).
# --------------------------------------------------------------------

def test_interleaving_supported_false_via_cli(tmp_path):
    """--interleaving-supported=false from the CLI must put the
    applier into vanilla mode: no CMD_SET_INTERLEAVE_DEPTH (opcode 5)
    ever emitted, even when the depth in a decision changes."""
    with _sandbox(
        tmp_path,
        cli_args=["--interleaving-supported=false"],
    ) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"

        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 2), \
            f"wfb_tx got {s['wfb'].received}"

        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=3,
                bitrate=12000, fps=60, sequence=2)
        time.sleep(0.25)

        cmd_ids = {r["cmd_id"] for r in s["wfb"].received}
        assert CMD_SET_FEC in cmd_ids
        assert CMD_SET_RADIO in cmd_ids
        assert CMD_SET_INTERLEAVE_DEPTH not in cmd_ids, \
            f"--interleaving-supported=false should suppress opcode 5; " \
            f"got {s['wfb'].received}"


def test_interleaving_supported_zero_via_cli(tmp_path):
    """--interleaving-supported=0 must be equivalent to =false."""
    with _sandbox(
        tmp_path,
        cli_args=["--interleaving-supported=0"],
    ) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"

        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 2), \
            f"wfb_tx got {s['wfb'].received}"

        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=3,
                bitrate=12000, fps=60, sequence=2)
        time.sleep(0.25)

        cmd_ids = {r["cmd_id"] for r in s["wfb"].received}
        assert CMD_SET_FEC in cmd_ids
        assert CMD_SET_RADIO in cmd_ids
        assert CMD_SET_INTERLEAVE_DEPTH not in cmd_ids, \
            f"--interleaving-supported=0 should suppress opcode 5; " \
            f"got {s['wfb'].received}"


def test_bare_flag_still_means_true(tmp_path):
    """Bare --osd-enable (no value) must set the field to true,
    matching the pre-change behavior. We assert by running the
    full _sandbox lifecycle and confirming the applier boots and
    a CMD_SET_RADIO (safe_defaults push) arrives — proxy for
    'parse_args returned 0 and the watchdog is running'."""
    with _sandbox(
        tmp_path,
        health_timeout_ms=500,
        cli_args=["--osd-enable"],
    ) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, fps=60, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        s["wfb"].received.clear()
        _wait_for_safe_radio(s["wfb"])


def test_bool_bad_value_exits_nonzero(tmp_path):
    """--osd-enable=garbage must exit non-zero with a clear error
    on stderr. We invoke dl-applier directly (no _sandbox) since
    the process must crash before binding anything."""
    cfg = tmp_path / "drone.conf"
    cfg.write_text("listen_addr = 127.0.0.1\nlisten_port = 0\n")

    out = subprocess.run(
        [str(APPLIER), "--config", str(cfg), "--osd-enable=garbage"],
        capture_output=True, text=True, timeout=5,
    )
    assert out.returncode != 0, (
        f"expected non-zero exit, got rc={out.returncode}, "
        f"stderr={out.stderr!r}"
    )
    assert "--osd-enable" in out.stderr
    assert "bad value" in out.stderr
    assert "garbage" in out.stderr
