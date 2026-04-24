"""asyncio TCP client for wfb-ng's JSON stats API (§3).

The server is `StatisticsJSONProtocol` in wfb_ng/protocols.py:72.
Newline-delimited JSON, one record per line. First record on connect
is a 'settings' dump; subsequent records are 'rx', 'tx', or
'new_session' at `log_interval` cadence (we require 100 ms → 10 Hz).
"""
from __future__ import annotations

import asyncio
import json
import logging
from dataclasses import dataclass, field
from typing import AsyncIterator, Callable, Any
from urllib.parse import urlparse

log = logging.getLogger(__name__)

CONTRACT_VERSION_EXPECTED = 2


class ContractVersionError(RuntimeError):
    """Raised when the wfb-ng feed advertises a contract_version we don't speak."""


@dataclass
class SettingsEvent:
    profile: str
    is_cluster: bool
    wlans: list[str]
    settings: dict
    timestamp: float = 0.0


@dataclass
class RxAnt:
    ant: int
    freq: int
    mcs: int
    bw: int
    pkt_recv: int
    rssi_min: int
    rssi_avg: int
    rssi_max: int
    snr_min: int
    snr_avg: int
    snr_max: int


@dataclass
class SessionInfo:
    fec_type: str
    fec_k: int
    fec_n: int
    epoch: int
    interleave_depth: int
    contract_version: int


@dataclass
class RxEvent:
    """One 'rx' record — a 100 ms stats window from wfb_rx."""
    timestamp: float
    id: str
    # `packets` is {key: [window_count, cumulative_count]}; we keep window only.
    packets_window: dict[str, int] = field(default_factory=dict)
    rx_ant_stats: list[RxAnt] = field(default_factory=list)
    session: SessionInfo | None = None
    tx_wlan: int | None = None


@dataclass
class TxEvent:
    timestamp: float
    id: str
    packets_window: dict[str, int] = field(default_factory=dict)


@dataclass
class SessionEvent:
    """Emitted on-change and once per window (wfb_rx `SESSION`)."""
    timestamp: float
    id: str
    session: SessionInfo


Event = SettingsEvent | RxEvent | TxEvent | SessionEvent


def _parse_packets(d: dict) -> dict[str, int]:
    """Packets field: {key: [window, cumulative]} → {key: window}."""
    out: dict[str, int] = {}
    for k, v in d.items():
        if isinstance(v, (list, tuple)) and len(v) >= 1:
            out[k] = int(v[0])
        else:  # defensive — unknown shape
            out[k] = int(v)
    return out


def _parse_session(d: dict) -> SessionInfo:
    return SessionInfo(
        fec_type=str(d.get("fec_type", "")),
        fec_k=int(d["fec_k"]),
        fec_n=int(d["fec_n"]),
        epoch=int(d["epoch"]),
        interleave_depth=int(d["interleave_depth"]),
        contract_version=int(d["contract_version"]),
    )


def _parse_rx_ant(d: dict) -> RxAnt:
    return RxAnt(
        ant=int(d["ant"]),
        freq=int(d["freq"]),
        mcs=int(d["mcs"]),
        bw=int(d["bw"]),
        pkt_recv=int(d["pkt_recv"]),
        rssi_min=int(d["rssi_min"]),
        rssi_avg=int(d["rssi_avg"]),
        rssi_max=int(d["rssi_max"]),
        snr_min=int(d["snr_min"]),
        snr_avg=int(d["snr_avg"]),
        snr_max=int(d["snr_max"]),
    )


def parse_record(raw: dict) -> Event | None:
    """Turn one parsed JSON record into an Event. Returns None on unknown type."""
    rtype = raw.get("type")
    ts = float(raw.get("timestamp", 0.0))
    if rtype == "settings":
        return SettingsEvent(
            profile=str(raw.get("profile", "")),
            is_cluster=bool(raw.get("is_cluster", False)),
            wlans=list(raw.get("wlans", [])),
            settings=dict(raw.get("settings", {})),
            timestamp=ts,
        )
    if rtype == "rx":
        session = None
        if "session" in raw and raw["session"]:
            session = _parse_session(raw["session"])
            if session.contract_version != CONTRACT_VERSION_EXPECTED:
                raise ContractVersionError(
                    f"contract_version={session.contract_version}, "
                    f"expected {CONTRACT_VERSION_EXPECTED}"
                )
        return RxEvent(
            timestamp=ts,
            id=str(raw.get("id", "")),
            packets_window=_parse_packets(raw.get("packets", {})),
            rx_ant_stats=[_parse_rx_ant(a) for a in raw.get("rx_ant_stats", [])],
            session=session,
            tx_wlan=raw.get("tx_wlan"),
        )
    if rtype == "tx":
        return TxEvent(
            timestamp=ts,
            id=str(raw.get("id", "")),
            packets_window=_parse_packets(raw.get("packets", {})),
        )
    if rtype == "new_session":
        session = _parse_session(raw)
        if session.contract_version != CONTRACT_VERSION_EXPECTED:
            raise ContractVersionError(
                f"contract_version={session.contract_version}, "
                f"expected {CONTRACT_VERSION_EXPECTED}"
            )
        return SessionEvent(
            timestamp=ts,
            id=str(raw.get("id", "")),
            session=session,
        )
    log.debug("ignoring unknown record type: %r", rtype)
    return None


async def iter_lines(reader: asyncio.StreamReader) -> AsyncIterator[bytes]:
    while True:
        line = await reader.readline()
        if not line:
            return
        yield line


async def iter_events_from_reader(
    reader: asyncio.StreamReader,
) -> AsyncIterator[Event]:
    async for line in iter_lines(reader):
        line = line.strip()
        if not line:
            continue
        try:
            raw = json.loads(line)
        except json.JSONDecodeError as e:
            log.warning("skipping malformed JSON line: %s", e)
            continue
        ev = parse_record(raw)
        if ev is not None:
            yield ev


def _parse_endpoint(url: str) -> tuple[str, int]:
    """tcp://host:port → (host, port). Bare host:port is also accepted."""
    if "://" not in url:
        url = "tcp://" + url
    parsed = urlparse(url)
    if parsed.scheme != "tcp":
        raise ValueError(f"unsupported scheme {parsed.scheme!r} in {url!r}")
    if not parsed.hostname or not parsed.port:
        raise ValueError(f"endpoint must be tcp://host:port, got {url!r}")
    return parsed.hostname, parsed.port


class StatsClient:
    """Reconnecting asyncio client for the wfb-ng JSON stats API."""

    def __init__(
        self,
        endpoint: str,
        on_event: Callable[[Event], Any],
        *,
        reconnect_initial_s: float = 1.0,
        reconnect_max_s: float = 30.0,
    ) -> None:
        self._host, self._port = _parse_endpoint(endpoint)
        self._on_event = on_event
        self._reconnect_initial_s = reconnect_initial_s
        self._reconnect_max_s = reconnect_max_s
        self._stop = asyncio.Event()

    def stop(self) -> None:
        self._stop.set()

    async def run(self) -> None:
        backoff = self._reconnect_initial_s
        while not self._stop.is_set():
            try:
                reader, writer = await asyncio.open_connection(self._host, self._port)
            except OSError as e:
                log.warning(
                    "stats_client: connect %s:%d failed: %s (retry in %.1fs)",
                    self._host, self._port, e, backoff,
                )
                await self._sleep_or_stop(backoff)
                backoff = min(backoff * 2, self._reconnect_max_s)
                continue

            log.info("stats_client: connected to %s:%d", self._host, self._port)
            backoff = self._reconnect_initial_s
            try:
                async for ev in iter_events_from_reader(reader):
                    if self._stop.is_set():
                        break
                    res = self._on_event(ev)
                    if asyncio.iscoroutine(res):
                        await res
            except ContractVersionError:
                # Hard fail — silent divergence is the whole reason the contract exists.
                log.error("stats_client: contract_version mismatch; aborting")
                raise
            except Exception:
                log.exception("stats_client: error in event loop")
            finally:
                writer.close()
                try:
                    await writer.wait_closed()
                except Exception:
                    pass

            if self._stop.is_set():
                break
            log.info("stats_client: disconnected; reconnecting in %.1fs", backoff)
            await self._sleep_or_stop(backoff)
            backoff = min(backoff * 2, self._reconnect_max_s)

    async def _sleep_or_stop(self, seconds: float) -> None:
        try:
            await asyncio.wait_for(self._stop.wait(), timeout=seconds)
        except asyncio.TimeoutError:
            pass


class ReplayClient:
    """Read events from a captured JSONL file (same schema as the live feed).

    Used by `--replay` for offline validation. Yields events as fast as
    the consumer can process them (no pacing).
    """

    def __init__(
        self,
        path: str,
        on_event: Callable[[Event], Any],
    ) -> None:
        self._path = path
        self._on_event = on_event
        self._stop = asyncio.Event()

    def stop(self) -> None:
        self._stop.set()

    async def run(self) -> None:
        with open(self._path, "r") as fd:
            for line in fd:
                if self._stop.is_set():
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    raw = json.loads(line)
                except json.JSONDecodeError as e:
                    log.warning("replay: skipping malformed line: %s", e)
                    continue
                ev = parse_record(raw)
                if ev is None:
                    continue
                res = self._on_event(ev)
                if asyncio.iscoroutine(res):
                    await res
                await asyncio.sleep(0)
