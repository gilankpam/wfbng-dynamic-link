"""Diff two drone-side dl-events.jsonl files.

The intended use is comparing a flight SD log against a bench-replay
SD log. A clean replay produces matching streams; behavioural
divergence shows up as records present on one side but not the other.

Records are aligned by `seq` (per-boot monotonic counter). Since
each side's seq starts from zero on its own boot, a clean run
produces identical seq values for the same stream of failures.

We deliberately ignore `t` (run-relative microseconds) and don't
diff `detail` field-by-field — only `(reason, sev)` plus a
top-level marker `body_match` that flags whether `detail.body`
strings differ where present (the encoder reply is the very
thing we want to surface, not assert on).
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger("dl-events-diff")


@dataclass(frozen=True)
class Event:
    seq: int
    reason: str
    sev: str
    detail: dict


def _load(path: Path) -> list[Event]:
    out: list[Event] = []
    with open(path, "r") as fd:
        for lineno, raw in enumerate(fd, start=1):
            raw = raw.strip()
            if not raw:
                continue
            try:
                rec = json.loads(raw)
            except ValueError as e:
                log.warning("%s:%d: bad JSON: %s", path, lineno, e)
                continue
            try:
                out.append(Event(
                    seq=int(rec["seq"]),
                    reason=str(rec["reason"]),
                    sev=str(rec.get("sev", "")),
                    detail=dict(rec.get("detail") or {}),
                ))
            except (KeyError, TypeError) as e:
                log.warning("%s:%d: missing field: %s", path, lineno, e)
                continue
    return out


def _diff(a: list[Event], b: list[Event]) -> tuple[list[str], int, int]:
    """Walk both lists in seq order. Return (lines, n_diffs, n_matches)."""
    lines: list[str] = []
    diffs = 0
    matches = 0

    by_seq_a = {e.seq: e for e in a}
    by_seq_b = {e.seq: e for e in b}
    seqs = sorted(set(by_seq_a) | set(by_seq_b))

    for s in seqs:
        ea = by_seq_a.get(s)
        eb = by_seq_b.get(s)
        if ea and eb:
            if ea.reason != eb.reason or ea.sev != eb.sev:
                diffs += 1
                lines.append(
                    f"~ seq={s}: A={ea.reason}/{ea.sev} "
                    f"vs B={eb.reason}/{eb.sev}"
                )
            else:
                matches += 1
                # Detail comparison: only flag when both sides supply
                # a `body` and they differ — that's the high-signal
                # case (encoder responded differently).
                ba = ea.detail.get("body")
                bb = eb.detail.get("body")
                if ba is not None and bb is not None and ba != bb:
                    lines.append(
                        f"~ seq={s} {ea.reason}: body differs"
                        f"\n    A: {ba!r}\n    B: {bb!r}"
                    )
                    diffs += 1
        elif ea:
            diffs += 1
            lines.append(f"- seq={s} only in A: {ea.reason}/{ea.sev}")
        else:
            diffs += 1
            lines.append(f"+ seq={s} only in B: {eb.reason}/{eb.sev}")

    return lines, diffs, matches


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="dl-events-diff")
    p.add_argument("a", type=Path, help="First (e.g. flight) dl-events.jsonl")
    p.add_argument("b", type=Path, help="Second (e.g. bench) dl-events.jsonl")
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

    a = _load(args.a)
    b = _load(args.b)
    lines, diffs, matches = _diff(a, b)

    print(f"A: {len(a)} events ({args.a})")
    print(f"B: {len(b)} events ({args.b})")
    print(f"matches: {matches}")
    print(f"diffs:   {diffs}")
    print()
    for line in lines:
        print(line)

    return 1 if diffs else 0


if __name__ == "__main__":
    sys.exit(main())
