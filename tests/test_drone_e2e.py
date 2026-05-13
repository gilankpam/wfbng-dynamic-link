"""End-to-end test for the drone applier.

Spawns `dl-applier` with a generated test config that points at:
  - a mock wfb_tx UDP server (decodes cmd_req_t into dicts)
  - a mock encoder HTTP server (records requests)
  - a temp OSD path

Drives the applier with `dl-inject` and asserts that the expected
commands / HTTP paths land.

Requires `make -C drone` to have built `dl-applier` and `dl-inject`
at drone/build/. The `build_drone` fixture rebuilds on demand.
"""
from __future__ import annotations

import contextlib
import os
import socket
import struct
import subprocess
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
DRONE_DIR = REPO_ROOT / "drone"
APPLIER   = DRONE_DIR / "build" / "dl-applier"
INJECT    = DRONE_DIR / "build" / "dl-inject"


# -----------------------------------------------------------------
# Build fixture — ensure native binaries exist.
# -----------------------------------------------------------------

@pytest.fixture(scope="session", autouse=True)
def build_drone():
    if not APPLIER.exists() or not INJECT.exists():
        subprocess.run(["make", "-C", str(DRONE_DIR)], check=True)
    assert APPLIER.exists() and INJECT.exists()


# -----------------------------------------------------------------
# Mock wfb_tx control server (decodes tx_cmd.h requests).
# -----------------------------------------------------------------

CMD_SET_FEC              = 1
CMD_SET_RADIO            = 2
CMD_SET_INTERLEAVE_DEPTH = 5


@dataclass
class FakeWfbTx:
    port: int = 0
    received: list[dict] = field(default_factory=list)
    recv_times: list[float] = field(default_factory=list)
    _sock: socket.socket | None = None
    _thread: threading.Thread | None = None
    _stop: bool = False

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(0.25)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._sock:
            self._sock.close()

    def _run(self) -> None:
        assert self._sock is not None
        while not self._stop:
            try:
                data, addr = self._sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return
            parsed = self._parse(data)
            if parsed is None:
                continue
            self.received.append(parsed)
            self.recv_times.append(time.monotonic())
            # Reply with rc=0 echo (req_id + rc), 8 bytes.
            reply = struct.pack(">II", parsed["req_id_raw"], 0)
            self._sock.sendto(reply, addr)

    @staticmethod
    def _parse(data: bytes) -> dict | None:
        # cmd_req_t is packed: req_id (4, native uint32) + cmd_id (1) + union
        if len(data) < 5:
            return None
        # wfb-ng's wire order for req_id: sent as htonl() so big-endian on wire.
        req_id_net = struct.unpack(">I", data[:4])[0]
        cmd_id = data[4]
        payload = data[5:]
        out: dict[str, Any] = {"req_id_raw": struct.unpack(">I", data[:4])[0],
                               "req_id": req_id_net, "cmd_id": cmd_id}
        if cmd_id == CMD_SET_FEC and len(payload) >= 2:
            out["k"] = payload[0]
            out["n"] = payload[1]
        elif cmd_id == CMD_SET_INTERLEAVE_DEPTH and len(payload) >= 1:
            out["depth"] = payload[0]
        elif cmd_id == CMD_SET_RADIO and len(payload) >= 7:
            out["stbc"]       = payload[0]
            out["ldpc"]       = bool(payload[1])
            out["short_gi"]   = bool(payload[2])
            out["bandwidth"]  = payload[3]
            out["mcs_index"]  = payload[4]
            out["vht_mode"]   = bool(payload[5])
            out["vht_nss"]    = payload[6]
        return out


# -----------------------------------------------------------------
# Mock encoder HTTP server (records paths hit).
# -----------------------------------------------------------------

class _RecordingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.server.recorded.append(self.path)  # type: ignore[attr-defined]
        self.server.recv_times.append(time.monotonic())  # type: ignore[attr-defined]
        status = getattr(self.server, "response_status", 200)
        body = getattr(self.server, "response_body", b"{}")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args, **kwargs):
        pass  # silence


def make_encoder_server(status: int = 200, body: bytes = b"{}") -> HTTPServer:
    srv = HTTPServer(("127.0.0.1", 0), _RecordingHandler)
    srv.recorded = []  # type: ignore[attr-defined]
    srv.recv_times = []  # type: ignore[attr-defined]
    srv.response_status = status  # type: ignore[attr-defined]
    srv.response_body = body      # type: ignore[attr-defined]
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    srv._thread = t  # type: ignore[attr-defined]
    return srv


# -----------------------------------------------------------------
# Harness — config file, applier spawn, teardown.
# -----------------------------------------------------------------

class FakeMavlinkSink:
    """Mock UDP listener that captures STATUSTEXT bytes the drone sends.

    Binds an ephemeral port; the applier config points mavlink_port at
    this port. We store raw frames; tests decode them with a hand
    parser for STATUSTEXT.
    """
    def __init__(self):
        self.port = 0
        self.received: list[bytes] = []
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = False

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(0.25)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        if self._thread: self._thread.join(timeout=1.0)
        if self._sock: self._sock.close()

    def _run(self) -> None:
        assert self._sock is not None
        while not self._stop:
            try:
                data, _ = self._sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                return
            self.received.append(data)

    def statustexts(self) -> list[tuple[int, str]]:
        """Decode received STATUSTEXT frames to (severity, text)."""
        out = []
        for frame in self.received:
            if len(frame) < 8 or frame[0] != 0xFE or frame[5] != 253:
                continue
            plen = frame[1]
            if len(frame) < 6 + plen + 2:
                continue
            payload = frame[6:6 + plen]
            severity = payload[0]
            text = payload[1:].rstrip(b"\x00").decode("utf-8", errors="replace")
            out.append((severity, text))
        return out


class _Sandbox:
    """Container for sandbox state and per-test helpers.

    Supports dict-style access (`s["wfb"]`, `s["listen_port"]`) for the
    legacy tests, plus method-style helpers (`recv_one_hello`,
    `send_to_drone`, `restart_drone_applier`) for the P4a handshake e2e
    tests.
    """
    def __init__(self, state: dict, gs_tunnel_sock: socket.socket,
                 cfg_path: Path, cfg_dict: dict):
        self._state = state
        self._gs_tunnel_sock = gs_tunnel_sock
        self._cfg_path = cfg_path
        self._cfg_dict = cfg_dict

    def __getitem__(self, key):
        return self._state[key]

    def __getattr__(self, key):
        try:
            return self._state[key]
        except KeyError:
            raise AttributeError(key)

    def send_to_drone(self, data: bytes) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(data, (self._state["listen_addr"],
                               self._state["listen_port"]))
        finally:
            sock.close()

    def recv_one_hello(self, timeout: float = 2.0) -> bytes | None:
        """Drain the gs_tunnel socket until a 'hello' packet arrives or
        timeout elapses. Returns the raw bytes (32-byte DLHE) or None."""
        from dynamic_link.wire import peek_kind
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._gs_tunnel_sock.settimeout(remaining)
            try:
                data, _ = self._gs_tunnel_sock.recvfrom(256)
            except socket.timeout:
                return None
            if peek_kind(data) == "hello":
                return data
            # Drop anything else (pong, etc.) and keep waiting.

    def restart_drone_applier(self) -> None:
        """Kill and respawn dl-applier with the same config. Used to
        verify a fresh generation_id appears in the next HELLO."""
        proc = self._state["proc"]
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        listen_addr = self._state["listen_addr"]
        listen_port = self._state["listen_port"]
        # Same wait-for-bind dance as the initial spawn.
        new_proc = subprocess.Popen(
            [str(APPLIER), "--config", str(self._cfg_path), "--debug"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        deadline = time.monotonic() + 2.0
        bound = False
        while time.monotonic() < deadline:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                probe.bind((listen_addr, listen_port))
                probe.close()
                time.sleep(0.02)
            except OSError:
                probe.close()
                bound = True
                break
        if not bound:
            new_proc.terminate()
            raise RuntimeError("dl-applier did not rebind listen port within 2s")
        self._state["proc"] = new_proc


@contextlib.contextmanager
def _sandbox(tmp_path: Path, *, extra_drone_conf: dict | None = None,
             **overrides):
    wfb = FakeWfbTx()
    wfb.start()
    enc = make_encoder_server(
        status=overrides.pop("_encoder_status", 200),
    )
    enc_port = enc.server_address[1]
    mav = FakeMavlinkSink()
    mav.start()

    # Pick a free port for the applier's UDP listen.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    listen_port = s.getsockname()[1]
    s.close()

    # Pre-bind the GS-side tunnel listener. The applier sends PONGs and
    # (P4a) HELLOs here. Binding now guarantees the port is taken before
    # we tell the applier about it, so its first sendto doesn't lose to
    # a kernel-side "no listener" drop on some configurations.
    gs_tunnel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gs_tunnel_sock.bind(("127.0.0.1", 0))
    gs_tunnel_port = gs_tunnel_sock.getsockname()[1]

    osd_path = tmp_path / "MSPOSD.msg"
    cfg = tmp_path / "drone.conf"

    defaults = {
        "listen_addr":       "127.0.0.1",
        "listen_port":       listen_port,
        "wfb_tx_ctrl_addr":  "127.0.0.1",
        "wfb_tx_ctrl_port":  wfb.port,
        "encoder_host":      "127.0.0.1",
        "encoder_port":      enc_port,
        "encoder_kind":      "majestic",
        "osd_enable":        1,
        "osd_msg_path":      str(osd_path),
        "osd_update_interval_ms": 200,
        "min_idr_interval_ms":    500,
        "health_timeout_ms":      2000,
        # Nonexistent wlan to guarantee the `iw` backend fails softly
        # (we're not testing iw).
        "wlan_dev":          "dl-nonexistent0",
        "safe_k": 8, "safe_n": 12, "safe_depth": 1,
        "safe_mcs": 1, "safe_bandwidth": 20,
        "safe_tx_power_dBm": 20, "safe_bitrate_kbps": 2000,
        # MAVLink — point at the FakeMavlinkSink.
        "mavlink_enable": 1,
        "mavlink_addr":   "127.0.0.1",
        "mavlink_port":   mav.port,
        "mavlink_sysid":  250,
        "mavlink_compid": 191,
        # GS tunnel — point at the pre-bound socket above so PONGs and
        # HELLOs land somewhere a test can observe.
        "gs_tunnel_addr":  "127.0.0.1",
        "gs_tunnel_port":  gs_tunnel_port,
    }
    defaults.update(overrides)
    if extra_drone_conf:
        defaults.update(extra_drone_conf)
    with open(cfg, "w") as f:
        for k, v in defaults.items():
            f.write(f"{k} = {v}\n")

    proc = subprocess.Popen(
        [str(APPLIER), "--config", str(cfg), "--debug"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    # Wait for applier to bind the socket. Probe by trying to bind
    # the same port ourselves — if our bind succeeds, the applier
    # hasn't bound yet (so keep waiting); if it fails with EADDRINUSE,
    # the applier owns it and we're ready to send.
    deadline = time.monotonic() + 2.0
    bound = False
    while time.monotonic() < deadline:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind((defaults["listen_addr"], listen_port))
            probe.close()
            time.sleep(0.02)
        except OSError:
            probe.close()
            bound = True
            break
    if not bound:
        proc.terminate()
        raise RuntimeError("dl-applier did not bind listen port within 2s")

    state = {
        "wfb": wfb, "encoder": enc, "cfg": defaults,
        "mavlink": mav,
        "listen_addr": defaults["listen_addr"],
        "listen_port": listen_port,
        "gs_tunnel_port": gs_tunnel_port,
        "osd_path": osd_path,
        "proc": proc,
    }
    try:
        yield _Sandbox(state, gs_tunnel_sock, cfg, defaults)
    finally:
        proc = state["proc"]
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
        gs_tunnel_sock.close()


def _port_open(addr: str, port: int) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(b"\x00", (addr, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def _inject(target: str, **kwargs) -> None:
    args = [str(INJECT), "--target", target]
    for k, v in kwargs.items():
        if isinstance(v, bool):
            if v:
                args.append(f"--{k.replace('_', '-')}")
        else:
            args.extend([f"--{k.replace('_', '-')}", str(v)])
    subprocess.run(args, check=True, capture_output=True)


def _wait_until(predicate, timeout_s=2.0, interval=0.02):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


# -----------------------------------------------------------------
# The tests.
# -----------------------------------------------------------------

def test_golden_path_dispatches_all_backends(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3), \
            f"wfb_tx got {s['wfb'].received}"
        assert _wait_until(lambda: len(s["encoder"].recorded) >= 1), \
            f"encoder got {s['encoder'].recorded}"

        cmds_by_id = {r["cmd_id"]: r for r in s["wfb"].received}
        assert CMD_SET_FEC in cmds_by_id
        assert cmds_by_id[CMD_SET_FEC]["k"] == 8
        assert cmds_by_id[CMD_SET_FEC]["n"] == 14
        assert CMD_SET_INTERLEAVE_DEPTH in cmds_by_id
        assert cmds_by_id[CMD_SET_INTERLEAVE_DEPTH]["depth"] == 2
        assert CMD_SET_RADIO in cmds_by_id
        radio = cmds_by_id[CMD_SET_RADIO]
        assert radio["mcs_index"] == 5
        assert radio["bandwidth"] == 20
        assert radio["short_gi"] is False  # pinned

        # Encoder got a /api/v1/set?... request with the three knobs.
        paths = s["encoder"].recorded
        assert any("video0.bitrate=12000" in p and "video0.fps=60" in p
                   for p in paths), paths


def test_idr_throttle_drops_duplicates(tmp_path: Path):
    with _sandbox(tmp_path, min_idr_interval_ms=500) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        # First decision with IDR.
        _inject(target, mcs=5, bandwidth=20, tx_power=15,
                k=8, n=12, depth=1, bitrate=8000, idr=True, sequence=1)
        # Second IDR inside the throttle window.
        _inject(target, mcs=5, bandwidth=20, tx_power=15,
                k=8, n=12, depth=1, bitrate=8000, idr=True, sequence=2)
        time.sleep(0.25)
        idrs = [p for p in s["encoder"].recorded if p == "/request/idr"]
        assert len(idrs) == 1, s["encoder"].recorded


@pytest.mark.asyncio
async def test_idr_burst_delivers_multiple_packets(tmp_path: Path):
    """End-to-end: when IdrBurster fires count=4 packets at 20ms
    spacing and the drone-side throttle is shorter than the spacing,
    multiple IDR HTTP calls land on the camera. Verifies the burst
    actually reaches the applier (lossy-UDP survival path)."""
    import asyncio
    from dynamic_link.decision import Decision
    from dynamic_link.idr_burst import IdrBurstConfig, IdrBurster
    from dynamic_link.return_link import ReturnLink
    from dynamic_link.wire import Encoder as WireEncoder

    # Throttle below burst spacing so each packet fires its own
    # /request/idr — that lets us count arrivals via encoder.recorded.
    with _sandbox(tmp_path, min_idr_interval_ms=10) as s:
        target_addr = s["listen_addr"]
        target_port = s["listen_port"]

        return_link = ReturnLink(target_addr, target_port)
        encoder = WireEncoder(seq=1)
        cfg = IdrBurstConfig(enabled=True, count=4, interval_ms=20)
        burster = IdrBurster(cfg, return_link, encoder)

        decision = Decision(
            timestamp=1.0, mcs=5, bandwidth=20, tx_power_dBm=18,
            k=8, n=12, depth=1, bitrate_kbps=8000,
            idr_request=True, reason="test",
        )

        # Mirror service.py: send the regular tick packet, then
        # trigger the count-1 follow-up burst.
        return_link.send(encoder.encode(decision))
        burster.trigger(decision)

        # Burst window is interval_ms * (count - 1) = 60 ms.
        # Allow generous slack for asyncio + applier scheduling.
        await asyncio.sleep(0.25)

        idrs = [p for p in s["encoder"].recorded if p == "/request/idr"]
        # Tolerant: UDP/asyncio timing on CI is nondeterministic.
        # We sent 4 packets total; at min_idr_interval_ms=10 below
        # the 20ms burst spacing, all 4 should fire — but assert
        # >= 2 to stay green under timing pressure.
        assert len(idrs) >= 2, (
            f"expected >= 2 IDR calls in burst window, got {len(idrs)} "
            f"(encoder.recorded={s['encoder'].recorded})"
        )

        return_link.close()


def test_watchdog_pushes_safe_defaults_on_silence(tmp_path: Path):
    with _sandbox(tmp_path, health_timeout_ms=500) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        # Warm up with one decision so the watchdog has a reference point.
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        initial_count = len(s["wfb"].received)
        # Now go silent for > timeout + OSD tick.
        time.sleep(1.0)
        # Watchdog should have pushed safe_defaults (FEC + DEPTH + RADIO).
        assert len(s["wfb"].received) >= initial_count + 3, \
            f"wfb got {s['wfb'].received}"
        # And the OSD should say so.
        content = s["osd_path"].read_text() if s["osd_path"].exists() else ""
        assert "WATCHDOG" in content


def test_watchdog_emits_statustext(tmp_path: Path):
    with _sandbox(tmp_path, health_timeout_ms=500) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        time.sleep(1.0)  # wait past watchdog
        assert _wait_until(
            lambda: any("DL WATCHDOG" in t for _, t in s["mavlink"].statustexts()),
            timeout_s=1.0,
        ), s["mavlink"].statustexts()


def test_duplicate_sequence_dropped(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=5, bandwidth=20, tx_power=15,
                k=8, n=12, depth=1, bitrate=8000, sequence=100)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3)
        n_after_first = len(s["wfb"].received)
        # Replay exact same sequence number — applier should dedup.
        _inject(target, mcs=6, bandwidth=20, tx_power=16,
                k=8, n=14, depth=2, bitrate=9000, sequence=100)
        time.sleep(0.25)
        assert len(s["wfb"].received) == n_after_first, s["wfb"].received


# -----------------------------------------------------------------
# Direction-aware staggered apply.
#
# The cold-start decision is always single-shot (first-decision rule),
# so each test warms up first, then snapshots baselines and verifies
# the second decision lands in two phases ~50 ms apart.
# -----------------------------------------------------------------

# Loose timing window: includes the configured 50 ms gap, plus
# scheduling jitter, plus the iw posix_spawn+waitpid blocking on the
# nonexistent wlan dev that the sandbox uses.
STAGGER_GAP_MIN_S = 0.030
STAGGER_GAP_MAX_S = 0.300


def test_apply_stagger_up_after_warmup(tmp_path: Path):
    """Bitrate up: tx_cmd lands first, encoder ~50 ms later."""
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=3, bandwidth=20, tx_power=15,
                k=8, n=12, depth=1, bitrate=6000, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3 and
                                   len(s["encoder"].recorded) >= 1)
        wfb_n0 = len(s["wfb"].received)
        enc_n0 = len(s["encoder"].recorded)

        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, sequence=2)
        # Phase 2 is the encoder; wait for it.
        assert _wait_until(
            lambda: len(s["encoder"].recorded) >= enc_n0 + 1,
            timeout_s=2.0,
        )
        # All 3 phase-1 tx_cmd messages must be in by now too.
        assert len(s["wfb"].received) >= wfb_n0 + 3, s["wfb"].received
        last_wfb_t = max(s["wfb"].recv_times[wfb_n0:])
        first_enc_t = s["encoder"].recv_times[enc_n0]
        gap = first_enc_t - last_wfb_t
        assert STAGGER_GAP_MIN_S <= gap <= STAGGER_GAP_MAX_S, \
            f"UP gap was {gap:.3f}s"


def test_apply_stagger_down_after_warmup(tmp_path: Path):
    """Bitrate down: encoder lands first, tx_cmd ~50 ms later."""
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3 and
                                   len(s["encoder"].recorded) >= 1)
        wfb_n0 = len(s["wfb"].received)
        enc_n0 = len(s["encoder"].recorded)

        _inject(target, mcs=3, bandwidth=20, tx_power=15,
                k=8, n=12, depth=1, bitrate=6000, sequence=2)
        # Phase 2 is tx_cmd (3 messages).
        assert _wait_until(
            lambda: len(s["wfb"].received) >= wfb_n0 + 3,
            timeout_s=2.0,
        )
        assert len(s["encoder"].recorded) >= enc_n0 + 1, s["encoder"].recorded
        last_enc_t = max(s["encoder"].recv_times[enc_n0:])
        first_wfb_t = min(s["wfb"].recv_times[wfb_n0:wfb_n0 + 3])
        gap = first_wfb_t - last_enc_t
        assert STAGGER_GAP_MIN_S <= gap <= STAGGER_GAP_MAX_S, \
            f"DOWN gap was {gap:.3f}s"


def test_apply_stagger_equal_is_single_shot(tmp_path: Path):
    """Same bitrate, other knobs differ: tx_cmd fires in one tight
    burst with no 50 ms gap, and the encoder isn't called at all
    (its internal diff sees no bitrate/roi/fps change)."""
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=10000, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3 and
                                   len(s["encoder"].recorded) >= 1)
        wfb_n0 = len(s["wfb"].received)
        enc_n0 = len(s["encoder"].recorded)

        _inject(target, mcs=3, bandwidth=20, tx_power=15,
                k=8, n=12, depth=1, bitrate=10000, sequence=2)
        assert _wait_until(
            lambda: len(s["wfb"].received) >= wfb_n0 + 3,
            timeout_s=1.0,
        )
        wfb_burst = s["wfb"].recv_times[wfb_n0:wfb_n0 + 3]
        spread = max(wfb_burst) - min(wfb_burst)
        assert spread < STAGGER_GAP_MIN_S, \
            f"EQUAL tx_cmd spread {spread:.3f}s should be < {STAGGER_GAP_MIN_S}s"
        # Wait past the would-be gap; encoder must remain untouched.
        time.sleep(0.150)
        assert len(s["encoder"].recorded) == enc_n0, \
            f"encoder should be untouched: {s['encoder'].recorded}"


def test_apply_stagger_cancel_replaces_pending(tmp_path: Path):
    """A new decision in the gap window cancels the queued phase 2."""
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, sequence=1)
        assert _wait_until(lambda: len(s["wfb"].received) >= 3 and
                                   len(s["encoder"].recorded) >= 1)
        wfb_n0 = len(s["wfb"].received)
        enc_n0 = len(s["encoder"].recorded)

        # First DOWN — encoder fires, tx_cmd phase 2 queued.
        _inject(target, mcs=4, bandwidth=20, tx_power=18,
                k=8, n=12, depth=1, bitrate=8000, sequence=2)
        assert _wait_until(
            lambda: len(s["encoder"].recorded) >= enc_n0 + 1,
            timeout_s=1.0,
        )
        # Within the 50 ms gap, a second DOWN supersedes — phase 2 of
        # the first decision must NOT fire on its own.
        time.sleep(0.020)
        _inject(target, mcs=3, bandwidth=20, tx_power=15,
                k=8, n=12, depth=1, bitrate=6000, sequence=3)
        # Wait long enough for both phase-1 encoder + phase-2 tx_cmd.
        time.sleep(0.300)
        new_enc = len(s["encoder"].recorded) - enc_n0
        new_wfb = len(s["wfb"].received) - wfb_n0
        # One encoder request per decision (2 total) — the cancelled
        # decision's phase 2 was the tx_cmd batch, not a second encoder.
        assert new_enc == 2, \
            f"expected 2 enc reqs, got {new_enc}: {s['encoder'].recorded}"
        # tx_cmd should fire only once, for the second decision's
        # phase 2 (cancelled first one never reached phase 2). The diff
        # against last_tx (still at warmup state mcs=5, n=14, depth=2)
        # finds mcs/n/depth all changed -> 3 messages.
        assert new_wfb == 3, \
            f"expected 3 tx_cmd reqs, got {new_wfb}: {s['wfb'].received}"


# -----------------------------------------------------------------
# P4a — drone→GS config handshake (DLHE / DLHA).
# -----------------------------------------------------------------

def test_drone_e2e_handshake_then_decide(tmp_path: Path):
    """Drone sends HELLO -> GS ACKs -> drone enters KEEPALIVE and the
    same generation_id reappears (no fresh announce rotation)."""
    wfb_yaml = tmp_path / "wfb.yaml"
    wfb_yaml.write_text("wireless:\n  mlink: 3994\n")
    majestic_yaml = tmp_path / "majestic.yaml"
    majestic_yaml.write_text("video0:\n  fps: 60\n")

    with _sandbox(
        tmp_path,
        extra_drone_conf={
            "hello_wfb_yaml_path": str(wfb_yaml),
            "hello_majestic_yaml_path": str(majestic_yaml),
            "hello_announce_initial_ms": 100,
            "hello_keepalive_ms": 200,
        },
    ) as ctx:
        from dynamic_link.wire import (
            HelloAck, decode_hello, encode_hello_ack,
        )

        hello_bytes = ctx.recv_one_hello(timeout=2.0)
        assert hello_bytes is not None, "drone did not announce DLHE"
        h1 = decode_hello(hello_bytes)
        assert h1.mtu_bytes == 3994
        assert h1.fps == 60

        # ACK back; drone should transition ANNOUNCING -> KEEPALIVE.
        ack = encode_hello_ack(HelloAck(generation_id_echo=h1.generation_id))
        ctx.send_to_drone(ack)

        # In KEEPALIVE the next DLHE arrives ~hello_keepalive_ms later
        # (200 ms) with the same generation_id. Use a generous window
        # to absorb scheduling jitter.
        h2_bytes = ctx.recv_one_hello(timeout=2.0)
        assert h2_bytes is not None, "drone did not emit keepalive DLHE"
        h2 = decode_hello(h2_bytes)
        assert h2.generation_id == h1.generation_id
        assert h2.mtu_bytes == 3994
        assert h2.fps == 60


def test_drone_e2e_restart_triggers_new_generation_id(tmp_path: Path):
    """Restarting dl-applier mid-run produces a fresh generation_id."""
    wfb_yaml = tmp_path / "wfb.yaml"
    wfb_yaml.write_text("wireless:\n  mlink: 1400\n")
    majestic_yaml = tmp_path / "majestic.yaml"
    majestic_yaml.write_text("video0:\n  fps: 30\n")

    with _sandbox(
        tmp_path,
        extra_drone_conf={
            "hello_wfb_yaml_path": str(wfb_yaml),
            "hello_majestic_yaml_path": str(majestic_yaml),
            "hello_announce_initial_ms": 100,
        },
    ) as ctx:
        from dynamic_link.wire import decode_hello

        b1 = ctx.recv_one_hello(timeout=2.0)
        assert b1 is not None
        h1 = decode_hello(b1)
        assert h1.mtu_bytes == 1400
        assert h1.fps == 30

        ctx.restart_drone_applier()

        b2 = ctx.recv_one_hello(timeout=2.0)
        assert b2 is not None
        h2 = decode_hello(b2)
        assert h2.mtu_bytes == 1400
        assert h2.fps == 30
        assert h1.generation_id != h2.generation_id, (
            f"expected fresh generation_id after restart, "
            f"both were 0x{h1.generation_id:08x}"
        )


def test_drone_e2e_bad_yaml_means_no_hello(tmp_path: Path):
    """If /etc/wfb.yaml has no mlink key, dl_hello_init fails and the
    state machine stays DISABLED — no DLHE is ever sent."""
    wfb_yaml = tmp_path / "wfb.yaml"
    wfb_yaml.write_text("wireless:\n  txpower: 50\n")  # no mlink
    majestic_yaml = tmp_path / "majestic.yaml"
    majestic_yaml.write_text("video0:\n  fps: 30\n")

    with _sandbox(
        tmp_path,
        extra_drone_conf={
            "hello_wfb_yaml_path": str(wfb_yaml),
            "hello_majestic_yaml_path": str(majestic_yaml),
            "hello_announce_initial_ms": 100,
        },
    ) as ctx:
        assert ctx.recv_one_hello(timeout=1.5) is None


# -----------------------------------------------------------------
# debug OSD latency stats (osd_debug_latency flag).
# -----------------------------------------------------------------

def _osd_text(osd_path: Path) -> str:
    if not osd_path.exists():
        return ""
    return osd_path.read_text(errors="replace")


def test_osd_debug_latency_disabled_default(tmp_path: Path):
    with _sandbox(tmp_path) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60)
        assert _wait_until(
            lambda: "MCS" in _osd_text(s["osd_path"])), "no status line"
        txt = _osd_text(s["osd_path"])
        for tag in ("FEC  ", "DPTH ", "RADIO", "TXPWR", "ENC  ", "IDR  "):
            assert tag not in txt, f"unexpected debug tag {tag!r} in: {txt!r}"


def test_osd_debug_latency_enabled_renders_six_lines(tmp_path: Path):
    with _sandbox(tmp_path, osd_debug_latency=1) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60)
        assert _wait_until(
            lambda: all(tag in _osd_text(s["osd_path"])
                        for tag in ("FEC  ", "DPTH ", "RADIO",
                                    "TXPWR", "ENC  ", "IDR  ")),
            timeout_s=3.0,
        ), f"missing debug tag(s): {_osd_text(s['osd_path'])!r}"


def test_osd_debug_latency_error_counter_increments(tmp_path: Path):
    with _sandbox(tmp_path, osd_debug_latency=1, _encoder_status=500) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target,
                mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2,
                bitrate=12000, fps=60)

        def enc_line_has_error():
            txt = _osd_text(s["osd_path"])
            for line in txt.splitlines():
                if "ENC  " in line and "e=0" not in line and "e=" in line:
                    return True
            return False
        assert _wait_until(enc_line_has_error, timeout_s=3.0), \
            f"no ENC error in: {_osd_text(s['osd_path'])!r}"


def test_osd_debug_latency_idr_throttle_not_counted(tmp_path: Path):
    with _sandbox(tmp_path,
                  osd_debug_latency=1,
                  min_idr_interval_ms=10_000) as s:
        target = f"{s['listen_addr']}:{s['listen_port']}"
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, fps=60,
                idr=True, sequence=1)
        _inject(target, mcs=5, bandwidth=20, tx_power=18,
                k=8, n=14, depth=2, bitrate=12000, fps=60,
                idr=True, sequence=2)

        def idr_line():
            for line in _osd_text(s["osd_path"]).splitlines():
                if "IDR  " in line:
                    return line
            return None

        def idr_n_is_one():
            line = idr_line()
            return line is not None and "n=1" in line
        assert _wait_until(idr_n_is_one, timeout_s=3.0), \
            f"expected IDR n=1, got: {_osd_text(s['osd_path'])!r}"

        time.sleep(s["cfg"]["osd_update_interval_ms"] / 1000.0 + 0.3)
        line = idr_line()
        assert line is not None
        assert "n=1" in line, f"expected n=1 (throttled), got line: {line!r}"
