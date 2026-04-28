"""Tests for the GS-side Cristian/EWMA offset estimator."""
from __future__ import annotations

import random

import pytest

from dynamic_link.timesync import TimeSync


def _simulate_pong(
    ts: TimeSync,
    *,
    seq: int,
    gs_t1: int,
    drone_offset_us: int,
    one_way_us: int,
    drone_processing_us: int = 50,
    extra_return_us: int = 0,
):
    """Build one ping/pong exchange and feed it to ``ts``.

    The drone monotonic clock is `gs_clock + drone_offset_us`. A real
    PONG carries (T1, T2, T3); we synthesise them from the chosen
    one-way latency and processing delay, then compute T4 as
    `T3 + return_one_way`. Returns the resulting LatencySample."""
    t1 = gs_t1
    drone_recv = (t1 + one_way_us) + drone_offset_us
    drone_send = drone_recv + drone_processing_us
    t4 = (drone_send - drone_offset_us) + one_way_us + extra_return_us
    return ts.observe(
        gs_seq=seq,
        gs_mono_us_t1=t1,
        drone_mono_recv_us_t2=drone_recv,
        drone_mono_send_us_t3=drone_send,
        gs_mono_us_t4=t4,
    )


def test_offset_estimate_converges_to_truth():
    truth = 10_000_000  # drone clock 10 s ahead of GS clock
    ts = TimeSync(ewma_alpha=0.3)
    for i in range(50):
        _simulate_pong(
            ts, seq=i, gs_t1=i * 200_000,  # 5 Hz
            drone_offset_us=truth,
            one_way_us=2000,                # symmetric 2 ms
            drone_processing_us=50,
        )
    assert ts.offset_us is not None
    # With symmetric latency Cristian's algorithm hits the ground truth
    # exactly; EWMA just smooths it. So convergence should be tight.
    assert abs(ts.offset_us - truth) < 100


def test_offset_unaffected_by_symmetric_jitter_on_average():
    truth = -500_000
    rng = random.Random(0)
    ts = TimeSync(ewma_alpha=0.2)
    for i in range(200):
        # Symmetric jitter on the one-way path: same draw both directions
        # would be wrong; instead we add iid noise both ways and rely on
        # the EWMA to average it out.
        outbound = 1500 + rng.randint(-200, 200)
        return_extra = rng.randint(-200, 200)
        _simulate_pong(
            ts, seq=i, gs_t1=i * 200_000,
            drone_offset_us=truth,
            one_way_us=outbound,
            drone_processing_us=80,
            extra_return_us=return_extra,
        )
    # With ~400 us peak-to-peak symmetric jitter the smoothed estimate
    # should still be well within a millisecond of truth.
    assert abs(ts.offset_us - truth) < 1000


def test_outliers_rejected_under_one_sided_congestion():
    truth = 0
    ts = TimeSync(ewma_alpha=0.3, outlier_rtt_factor=3.0)
    # First fill the median window with clean samples.
    for i in range(20):
        _simulate_pong(
            ts, seq=i, gs_t1=i * 200_000,
            drone_offset_us=truth,
            one_way_us=2000,
        )
    # Now inject one big return-path spike — the GS sees a huge RTT.
    pre_offset = ts.offset_us
    sample = _simulate_pong(
        ts, seq=999, gs_t1=999 * 200_000,
        drone_offset_us=truth,
        one_way_us=2000,
        extra_return_us=200_000,    # +200 ms only on return
    )
    assert sample.outlier is True
    # Outlier wasn't folded in, so the smoothed offset shouldn't move.
    assert ts.offset_us == pre_offset
    assert ts.stats["outliers"] == 1


def test_first_sample_seeds_estimate_directly():
    ts = TimeSync()
    sample = _simulate_pong(
        ts, seq=1, gs_t1=0,
        drone_offset_us=42_000_000,
        one_way_us=1000,
    )
    assert ts.offset_us is not None
    # Symmetric one-way → first sample = truth exactly.
    assert sample.offset_us == 42_000_000


def test_reset_drops_state():
    ts = TimeSync()
    _simulate_pong(ts, seq=1, gs_t1=0, drone_offset_us=1000, one_way_us=500)
    assert ts.offset_us is not None
    ts.reset()
    assert ts.offset_us is None
    assert ts.stats["samples"] == 0


def test_offset_stddev_grows_with_jitter():
    rng = random.Random(1)
    quiet = TimeSync(ewma_alpha=0.2)
    noisy = TimeSync(ewma_alpha=0.2)
    for i in range(60):
        _simulate_pong(quiet, seq=i, gs_t1=i * 200_000,
                       drone_offset_us=0, one_way_us=2000)
        _simulate_pong(noisy, seq=i, gs_t1=i * 200_000,
                       drone_offset_us=0,
                       one_way_us=2000 + rng.randint(-1000, 1000),
                       extra_return_us=rng.randint(-1000, 1000))
    assert noisy.offset_stddev_us > quiet.offset_stddev_us
