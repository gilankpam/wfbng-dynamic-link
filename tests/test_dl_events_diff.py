"""Tests for gs/tools/dl_events_diff.py."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gs"))

from tools import dl_events_diff  # noqa: E402


def _write(path: Path, records: list[dict]) -> None:
    with open(path, "w") as fd:
        for r in records:
            fd.write(json.dumps(r) + "\n")


def _ev(seq: int, reason: str, sev: str = "warn",
        **detail) -> dict:
    return {"t": seq * 1000, "seq": seq, "sev": sev,
            "reason": reason, "detail": detail}


def test_clean_replay_zero_diffs(tmp_path: Path):
    a = tmp_path / "a.jsonl"
    b = tmp_path / "b.jsonl"
    recs = [_ev(0, "WATCHDOG_TRIPPED"),
            _ev(1, "ENC_RESPONSE_BAD", http=500, body="oops")]
    _write(a, recs)
    _write(b, recs)

    rc = dl_events_diff.main([str(a), str(b)])
    assert rc == 0


def test_extra_event_in_b_flagged(tmp_path: Path, capsys):
    a = tmp_path / "a.jsonl"
    b = tmp_path / "b.jsonl"
    _write(a, [_ev(0, "WATCHDOG_TRIPPED")])
    _write(b, [_ev(0, "WATCHDOG_TRIPPED"),
               _ev(1, "ENC_RESPONSE_BAD", http=500)])

    rc = dl_events_diff.main([str(a), str(b)])
    assert rc != 0
    out = capsys.readouterr().out
    assert "+ seq=1 only in B" in out
    assert "ENC_RESPONSE_BAD" in out


def test_reason_mismatch_flagged(tmp_path: Path, capsys):
    a = tmp_path / "a.jsonl"
    b = tmp_path / "b.jsonl"
    _write(a, [_ev(0, "WATCHDOG_TRIPPED")])
    _write(b, [_ev(0, "ENC_RESPONSE_BAD")])

    rc = dl_events_diff.main([str(a), str(b)])
    assert rc != 0
    out = capsys.readouterr().out
    assert "~ seq=0" in out


def test_body_diff_surfaced(tmp_path: Path, capsys):
    a = tmp_path / "a.jsonl"
    b = tmp_path / "b.jsonl"
    _write(a, [_ev(0, "ENC_RESPONSE_BAD", http=500, body="rate limited")])
    _write(b, [_ev(0, "ENC_RESPONSE_BAD", http=500, body="bad request")])

    rc = dl_events_diff.main([str(a), str(b)])
    assert rc != 0
    out = capsys.readouterr().out
    assert "body differs" in out
    assert "rate limited" in out
    assert "bad request" in out


def test_t_field_ignored(tmp_path: Path):
    """Run-relative `t` differs between flight and bench; same `seq`
    + reason still counts as a match."""
    a = tmp_path / "a.jsonl"
    b = tmp_path / "b.jsonl"
    _write(a, [{"t": 100, "seq": 0, "sev": "warn",
                "reason": "WATCHDOG_TRIPPED", "detail": {}}])
    _write(b, [{"t": 999_999, "seq": 0, "sev": "warn",
                "reason": "WATCHDOG_TRIPPED", "detail": {}}])
    rc = dl_events_diff.main([str(a), str(b)])
    assert rc == 0
