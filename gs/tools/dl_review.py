"""View a flight bundle on a unified timeline.

Inputs:
  --bundle PATH         tarball produced by dl-bundle (or a directory
                        containing the same set of files)
  --drone-events PATH   dl-events.jsonl pulled from the drone SD card

Optional:
  --around TS           epoch seconds (or seconds-since-boot if the GS
                        log uses monotonic), focus the print window here
  --window SECONDS      ± half-window around --around (default 2.0)
  --reasons LIST        comma-separated list of drone reason filters

Output: chronological merge of GS event/verbose ticks, GS latency
samples, GS video frames, and drone failure events. Drone events are
re-stamped onto the GS clock by binary-searching the latency.jsonl
offset samples — the closer in time, the better the alignment.

The joining math (from Pillar A):

    gs_mono_us ≈ drone_mono_us + offset_us(at nearest sample)

Practical accuracy is bounded by the offset's stddev; in healthy
link conditions that's a few ms.
"""
from __future__ import annotations

import argparse
import bisect
import json
import logging
import sys
import tarfile
import tempfile
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger("dl-review")


@dataclass
class OffsetSample:
    """One latency.jsonl record, distilled to the bits dl-review needs."""
    drone_mono_us: int
    offset_us: int


def _load_offsets(path: Path) -> list[OffsetSample]:
    """Read latency.jsonl into a sorted list keyed by drone_mono_recv_us."""
    samples: list[OffsetSample] = []
    if not path.exists():
        return samples
    with open(path, "r") as fd:
        for raw in fd:
            raw = raw.strip()
            if not raw:
                continue
            try:
                rec = json.loads(raw)
            except ValueError:
                continue
            if rec.get("outlier"):
                continue
            try:
                samples.append(OffsetSample(
                    drone_mono_us=int(rec["drone_mono_recv_us"]),
                    offset_us=int(rec["offset_us"]),
                ))
            except (KeyError, TypeError, ValueError):
                continue
    samples.sort(key=lambda s: s.drone_mono_us)
    return samples


def _offset_at(samples: list[OffsetSample], drone_mono_us: int) -> int | None:
    """Find the offset estimate closest in drone-time to `drone_mono_us`.
    Returns None if there are no samples (i.e. ping_pong wasn't on)."""
    if not samples:
        return None
    keys = [s.drone_mono_us for s in samples]
    pos = bisect.bisect_left(keys, drone_mono_us)
    if pos == 0:
        return samples[0].offset_us
    if pos >= len(samples):
        return samples[-1].offset_us
    before = samples[pos - 1]
    after = samples[pos]
    if (drone_mono_us - before.drone_mono_us) <= (after.drone_mono_us - drone_mono_us):
        return before.offset_us
    return after.offset_us


# ---- Stream loaders --------------------------------------------------------

@dataclass
class Event:
    """One row in the unified-timeline stream.

    `ts_gs_mono_us` is the canonical sort key. For drone events
    without a matching offset sample (ping_pong was off, or the event
    fired before timesync converged) we fall back to the raw drone
    mono_us; ordering will be approximate but the user can still see
    the event.
    """
    ts_gs_mono_us: int
    source: str          # 'gs', 'gs.verbose', 'latency', 'video', 'drone'
    summary: str
    raw: dict


def _ts_event(rec: dict, source: str) -> Event:
    """Build an Event from a GS-side record. The verbose log uses the
    decision's float `timestamp` (seconds), already on the GS clock —
    no offset translation needed."""
    ts = float(rec.get("timestamp", 0.0))
    ts_us = int(ts * 1_000_000)
    bits = []
    if "mcs" in rec:
        bits.append(f"mcs={rec['mcs']} k={rec['k']} n={rec['n']} "
                    f"d={rec['depth']} br={rec['bitrate_kbps']}")
    if rec.get("knobs_changed"):
        bits.append(f"changed={','.join(rec['knobs_changed'])}")
    if rec.get("reason"):
        bits.append(f"reason={rec['reason']}")
    return Event(ts_gs_mono_us=ts_us, source=source,
                 summary=" ".join(bits), raw=rec)


def _ts_latency(rec: dict) -> Event:
    ts_us = int(rec["ts_gs_mono_us"])
    summary = (
        f"rtt={rec['rtt_us']/1000:.1f}ms offset={rec['offset_us']} "
        f"stddev={rec['offset_stddev_us']}"
        + (" OUTLIER" if rec.get("outlier") else "")
    )
    return Event(ts_gs_mono_us=ts_us, source="latency",
                 summary=summary, raw=rec)


def _ts_video(rec: dict) -> Event:
    ts_us = int(rec["ts_gs_mono_us"])
    drift_ms = rec["latency_drift_us"] / 1000.0
    summary = (
        f"frame seq={rec['rtp_seq_first']}-{rec['rtp_seq_last']} "
        f"pkts={rec['packets']}/{rec['expected']} "
        f"lost={rec['lost_in_frame']} drift={drift_ms:+.1f}ms"
    )
    return Event(ts_gs_mono_us=ts_us, source="video",
                 summary=summary, raw=rec)


def _ts_drone(rec: dict, samples: list[OffsetSample]) -> Event:
    drone_t = int(rec["t"])
    offset = _offset_at(samples, drone_t)
    ts_us = drone_t + offset if offset is not None else drone_t
    detail_pretty = json.dumps(rec.get("detail", {}), separators=(",", ":"))
    summary = f"[{rec['sev']}] {rec['reason']} {detail_pretty}"
    if offset is None:
        summary += " (no offset; raw drone-mono)"
    return Event(ts_gs_mono_us=ts_us, source="drone",
                 summary=summary, raw=rec)


def _load_jsonl(path: Path):
    if not path.exists():
        return
    with open(path, "r") as fd:
        for raw in fd:
            raw = raw.strip()
            if not raw:
                continue
            try:
                yield json.loads(raw)
            except ValueError:
                continue


def _gather(bundle_dir: Path, drone_events: Path | None) -> list[Event]:
    samples = _load_offsets(bundle_dir / "latency.jsonl")
    events: list[Event] = []
    for rec in _load_jsonl(bundle_dir / "gs.jsonl"):
        events.append(_ts_event(rec, "gs"))
    for rec in _load_jsonl(bundle_dir / "gs.verbose.jsonl"):
        events.append(_ts_event(rec, "gs.verbose"))
    for rec in _load_jsonl(bundle_dir / "latency.jsonl"):
        events.append(_ts_latency(rec))
    for rec in _load_jsonl(bundle_dir / "video_rtp.jsonl"):
        events.append(_ts_video(rec))
    if drone_events is not None:
        for rec in _load_jsonl(drone_events):
            events.append(_ts_drone(rec, samples))
    events.sort(key=lambda e: e.ts_gs_mono_us)
    return events


def _open_bundle(path: Path) -> tuple[Path, tempfile.TemporaryDirectory | None]:
    """Return a directory containing the bundle's files, plus an
    optional TemporaryDirectory whose lifetime the caller must keep
    alive. If the input is already a directory, returns (path, None)."""
    if path.is_dir():
        return path, None
    tmp = tempfile.TemporaryDirectory(prefix="dl-review-")
    with tarfile.open(path, "r:*") as tf:
        tf.extractall(tmp.name)
    extracted = Path(tmp.name)
    # Bundle files live at the root of the tar; if the tar happens to
    # include a top-level dir, descend into it.
    children = list(extracted.iterdir())
    if len(children) == 1 and children[0].is_dir():
        return children[0], tmp
    return extracted, tmp


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="dl-review")
    p.add_argument(
        "--bundle", required=True, type=Path,
        help="Path to a dl-bundle tarball or a directory of GS logs",
    )
    p.add_argument(
        "--drone-events", type=Path,
        help="Path to dl-events.jsonl pulled from the drone SD card",
    )
    p.add_argument(
        "--around", type=float,
        help="GS-mono seconds to focus on (or use --around-us for µs)",
    )
    p.add_argument("--around-us", type=int)
    p.add_argument(
        "--window", default=2.0, type=float,
        help="± half-window around --around in seconds (default 2.0)",
    )
    p.add_argument(
        "--reasons", default="",
        help="Comma-separated drone-reason filter (e.g. ENC_RESPONSE_BAD)",
    )
    p.add_argument(
        "--sources", default="",
        help="Comma-separated source filter "
             "(gs,gs.verbose,latency,video,drone)",
    )
    p.add_argument(
        "--limit", type=int, default=0,
        help="Max number of lines to print (0 = unlimited)",
    )
    p.add_argument(
        "--log-level", default="WARNING",
        help="DEBUG, INFO, WARNING, ERROR (default WARNING)",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.WARNING),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    bundle_dir, _tmp = _open_bundle(args.bundle)
    events = _gather(bundle_dir, args.drone_events)

    if args.around is not None:
        center_us = int(args.around * 1_000_000)
    elif args.around_us is not None:
        center_us = args.around_us
    else:
        center_us = None

    if center_us is not None:
        half_us = int(args.window * 1_000_000)
        events = [e for e in events
                  if (center_us - half_us) <= e.ts_gs_mono_us <= (center_us + half_us)]

    sources = {s.strip() for s in args.sources.split(",") if s.strip()}
    if sources:
        events = [e for e in events if e.source in sources]

    reasons = {r.strip() for r in args.reasons.split(",") if r.strip()}
    if reasons:
        events = [e for e in events
                  if e.source != "drone" or e.raw.get("reason") in reasons]

    if args.limit > 0:
        events = events[:args.limit]

    for e in events:
        ts_s = e.ts_gs_mono_us / 1_000_000
        print(f"{ts_s:14.6f} [{e.source:10s}] {e.summary}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
