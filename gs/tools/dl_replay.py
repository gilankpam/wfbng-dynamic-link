"""Replay a captured GS verbose.jsonl into a (bench) drone.

Reads the JSONL line by line, reconstructs each tick's Decision via
`gs.dynamic_link.wire.encode`, and `sendto`s it at the *recorded*
inter-tick cadence — so a flight you replay at the bench produces the
same applier behaviour you saw live (modulo encoder responses, which
is the very thing we want to compare).

Run as a module:

    python -m gs.tools.dl_replay \\
        --source /var/log/dynamic-link/gs.verbose.jsonl \\
        --target 10.5.0.2:5800

Or, after the package is installed, via the console script
`dl-replay` if your distro packaging exposes it.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import socket
import sys
import time
from pathlib import Path

from dynamic_link.decision import Decision
from dynamic_link.wire import Encoder

log = logging.getLogger("dl-replay")


def _parse_target(s: str) -> tuple[str, int]:
    if ":" not in s:
        raise argparse.ArgumentTypeError(f"--target {s!r}: want HOST:PORT")
    host, _, port = s.rpartition(":")
    if not host:
        raise argparse.ArgumentTypeError(f"--target {s!r}: empty host")
    try:
        portn = int(port)
    except ValueError as e:
        raise argparse.ArgumentTypeError(
            f"--target {s!r}: bad port: {e}",
        ) from None
    if not 1 <= portn <= 65535:
        raise argparse.ArgumentTypeError(
            f"--target {s!r}: port {portn} out of range",
        )
    return host, portn


def _decision_from_record(rec: dict) -> Decision:
    """Round-trip a verbose.jsonl record into a Decision dataclass.

    Tolerant of extra Phase-3 fields (`offset_us`, `offset_stddev_us`)
    that the LogSink may have stamped on top of the dataclass — we
    just ignore them.
    """
    return Decision(
        timestamp=float(rec["timestamp"]),
        mcs=int(rec["mcs"]),
        bandwidth=int(rec["bandwidth"]),
        tx_power_dBm=int(rec["tx_power_dBm"]),
        k=int(rec["k"]),
        n=int(rec["n"]),
        depth=int(rec["depth"]),
        bitrate_kbps=int(rec["bitrate_kbps"]),
        idr_request=bool(rec["idr_request"]),
        reason=str(rec.get("reason", "")),
        knobs_changed=list(rec.get("knobs_changed", [])),
        signals_snapshot=dict(rec.get("signals_snapshot", {})),
    )


async def _replay(args: argparse.Namespace) -> int:
    host, port = _parse_target(args.target)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    encoder = Encoder(seq=1)

    speed = float(args.speed)
    if speed <= 0:
        log.error("--speed must be > 0")
        return 2

    sent = 0
    skipped_before = 0
    skipped_after = 0
    parse_errors = 0
    prev_ts: float | None = None

    log.info("replay → %s:%d (speed %.2fx)", host, port, speed)

    with open(args.source, "r") as fd:
        for raw in fd:
            raw = raw.strip()
            if not raw:
                continue
            try:
                rec = json.loads(raw)
                dec = _decision_from_record(rec)
            except (KeyError, ValueError) as e:
                parse_errors += 1
                if parse_errors <= 5:
                    log.warning("skipping bad record: %s", e)
                continue

            if args.from_ts is not None and dec.timestamp < args.from_ts:
                skipped_before += 1
                continue
            if args.until_ts is not None and dec.timestamp > args.until_ts:
                skipped_after += 1
                continue

            # Sleep to match the recorded inter-tick gap. First tick
            # sends immediately.
            if prev_ts is not None:
                gap = (dec.timestamp - prev_ts) / speed
                if gap > 0:
                    await asyncio.sleep(gap)
            prev_ts = dec.timestamp

            packet = encoder.encode(dec)
            try:
                sock.sendto(packet, (host, port))
                sent += 1
            except OSError as e:
                log.warning("sendto: %s", e)

    sock.close()
    log.info(
        "done: sent=%d skipped_before=%d skipped_after=%d parse_errors=%d",
        sent, skipped_before, skipped_after, parse_errors,
    )
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="dl-replay")
    p.add_argument(
        "--source", required=True, type=Path,
        help="Path to GS verbose.jsonl",
    )
    p.add_argument(
        "--target", required=True, type=str,
        help="HOST:PORT of the (bench) drone applier",
    )
    p.add_argument(
        "--speed", default=1.0, type=float,
        help="Replay speed multiplier (default 1.0; 2.0 = 2x faster)",
    )
    p.add_argument(
        "--from-ts", type=float,
        help="Skip records with `timestamp` < this",
    )
    p.add_argument(
        "--until-ts", type=float,
        help="Stop after the last record with `timestamp` ≤ this",
    )
    p.add_argument(
        "--log-level", default="INFO",
        help="DEBUG, INFO, WARNING, ERROR (default INFO)",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        return asyncio.run(_replay(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
