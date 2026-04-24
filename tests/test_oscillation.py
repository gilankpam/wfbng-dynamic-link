"""Tests for the oscillation detector (failsafe 2)."""
from __future__ import annotations

from dynamic_link.decision import Decision
from dynamic_link.oscillation import OscillationConfig, OscillationDetector


def _dec(**overrides) -> Decision:
    base = Decision(
        timestamp=0.0,
        mcs=5, bandwidth=20, tx_power_dBm=18,
        k=8, n=12, depth=1, bitrate_kbps=10000,
        idr_request=False,
    )
    for k, v in overrides.items():
        setattr(base, k, v)
    return base


def _cfg(**overrides) -> OscillationConfig:
    defaults = {"window_ms": 1000, "lock_ms": 2000, "max_changes": 4}
    defaults.update(overrides)
    return OscillationConfig(**defaults)


def test_oscillation_below_threshold_is_noop():
    det = OscillationDetector(_cfg())
    # 4 FEC changes in 500 ms — equal to max_changes, not over.
    for i, n in enumerate([12, 14, 12, 14]):
        out = det.maybe_override(_dec(n=n), ts_ms=i * 100)
        assert out.n == n  # no override


def test_oscillation_locks_fec_after_fifth_change():
    det = OscillationDetector(_cfg(max_changes=4))
    # Five distinct changes in 500 ms — trips the detector.
    values = [(8, 12), (8, 14), (8, 12), (8, 14), (8, 16)]
    for i, (k, n) in enumerate(values):
        out = det.maybe_override(_dec(k=k, n=n), ts_ms=i * 100)
    # After the 5th change, the safer value — highest (n - k), then
    # highest n — gets locked. (8, 16) has overhead 8, (8, 14) has 6;
    # (8, 16) wins.
    assert (out.k, out.n) == (8, 16)
    # Subsequent decisions within the lock window get overridden.
    out = det.maybe_override(_dec(k=8, n=12), ts_ms=600)
    assert (out.k, out.n) == (8, 16)


def test_oscillation_lock_releases_after_lock_ms():
    det = OscillationDetector(_cfg(window_ms=1000, lock_ms=2000))
    values = [(8, 12), (8, 14), (8, 12), (8, 14), (8, 16)]
    for i, (k, n) in enumerate(values):
        det.maybe_override(_dec(k=k, n=n), ts_ms=i * 100)
    # Lock was established at ts=400 → expires at 400 + 2000 = 2400.
    out = det.maybe_override(_dec(k=8, n=12), ts_ms=2000)
    assert (out.k, out.n) == (8, 16)  # still locked
    out = det.maybe_override(_dec(k=8, n=12), ts_ms=2500)
    assert (out.k, out.n) == (8, 12)  # released


def test_oscillation_depth_picks_max():
    det = OscillationDetector(_cfg(max_changes=2))
    # 3 depth changes — trips; safer depth is MAX.
    values = [1, 2, 3]
    for i, d in enumerate(values):
        out = det.maybe_override(_dec(depth=d), ts_ms=i * 100)
    assert out.depth == 3


def test_oscillation_mcs_picks_min():
    det = OscillationDetector(_cfg(max_changes=2))
    values = [7, 5, 3]
    for i, m in enumerate(values):
        out = det.maybe_override(_dec(mcs=m), ts_ms=i * 100)
    assert out.mcs == 3


def test_oscillation_tx_power_picks_max():
    det = OscillationDetector(_cfg(max_changes=2))
    values = [10, 20, 15]
    for i, p in enumerate(values):
        out = det.maybe_override(_dec(tx_power_dBm=p), ts_ms=i * 100)
    assert out.tx_power_dBm == 20


def test_oscillation_bitrate_picks_min():
    det = OscillationDetector(_cfg(max_changes=2))
    values = [10000, 8000, 12000]
    for i, b in enumerate(values):
        out = det.maybe_override(_dec(bitrate_kbps=b), ts_ms=i * 100)
    assert out.bitrate_kbps == 8000


def test_oscillation_no_op_changes_do_not_count():
    det = OscillationDetector(_cfg(max_changes=2))
    # Same (k, n) repeated many times — should never trip.
    for i in range(10):
        out = det.maybe_override(_dec(k=8, n=12), ts_ms=i * 100)
    assert (out.k, out.n) == (8, 12)


def test_oscillation_window_sliding():
    det = OscillationDetector(_cfg(window_ms=500, max_changes=2))
    # 2 changes early, then silence, then 2 more — should NOT trip
    # because only 2 changes are within any 500 ms window.
    det.maybe_override(_dec(n=12), ts_ms=0)
    det.maybe_override(_dec(n=14), ts_ms=100)
    out = det.maybe_override(_dec(n=12), ts_ms=1000)
    out = det.maybe_override(_dec(n=14), ts_ms=1100)
    # Not locked.
    assert out.n == 14


def test_oscillation_reason_suffixed():
    det = OscillationDetector(_cfg(max_changes=2))
    for i, n in enumerate([12, 14, 16]):
        out = det.maybe_override(_dec(n=n), ts_ms=i * 100)
    assert "oscillation" in out.reason
    assert "oscillation" in out.knobs_changed
