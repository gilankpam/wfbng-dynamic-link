"""Tests for the latency-budget predictor.

Reference numbers from wfb-ng/doc/design/fec-enhancements-v2.md §4.2
worked example (MCS7 HT40, per_packet_airtime ≈ 80 µs,
inter_packet_interval ≈ 1.4 ms):

    (8, 12) d=1 → 12 ms
    (8, 14) d=3 → 37 ms
    (8, 14) d=4 → 49 ms  (at the 50 ms cap)
"""
from __future__ import annotations

import pytest

from dynamic_link.predictor import (
    BudgetExhausted,
    PredictorConfig,
    Proposal,
    fit_or_degrade,
    max_n_for_latency,
    predict,
)


REFERENCE_CFG = PredictorConfig(
    per_packet_airtime_us=80.0,
    inter_packet_interval_ms=1.4,
    fec_decode_ms=1.0,
    block_duration_ms=12.0,
)


def test_reference_812_d1_approx_12ms():
    p = predict(Proposal(8, 12, 1), REFERENCE_CFG)
    # Components: fill 11.2 + air 0.96 + decode 1.0 + interleave 0 = ~13.16
    # §4.2 rounds to 12 ms; we're within 2 ms of the spec value.
    assert abs(p.latency_ms - 12.0) < 2.0


def test_reference_814_d3_approx_37ms():
    p = predict(Proposal(8, 14, 3), REFERENCE_CFG)
    # fill 11.2 + air 1.12 + decode 1.0 + interleave 24.0 = 37.32
    assert abs(p.latency_ms - 37.0) < 1.5


def test_reference_814_d4_at_cap():
    p = predict(Proposal(8, 14, 4), REFERENCE_CFG)
    # ~49 ms — inside the 50 ms cap
    assert p.latency_ms < 50.0
    assert p.latency_ms > 45.0


def test_latency_monotone_in_depth():
    prev = predict(Proposal(8, 12, 1), REFERENCE_CFG).latency_ms
    for d in (2, 3, 4):
        cur = predict(Proposal(8, 12, d), REFERENCE_CFG).latency_ms
        assert cur > prev
        prev = cur


def test_fit_or_degrade_accepts_when_under_cap():
    result = fit_or_degrade(Proposal(8, 12, 1), cap_ms=50.0, cfg=REFERENCE_CFG)
    assert result == Proposal(8, 12, 1)


def test_fit_or_degrade_drops_depth_first():
    # (8, 16) d=4 at reference cfg = 49.48 ms. A 45 ms cap forces one
    # depth drop; predictor should drop depth (not k) first.
    result = fit_or_degrade(Proposal(8, 16, 4), cap_ms=45.0, cfg=REFERENCE_CFG)
    assert result.k == 8
    assert result.n == 16
    assert result.depth < 4


def test_fit_or_degrade_refuses_when_depth1_overruns():
    # depth=1 already overruns; nothing left to drop → BudgetExhausted.
    # (8, 16) d=1 = 11.2 + 1.28 + 1.0 = ~13.5 ms; cap at 10 ms → no fit.
    with pytest.raises(BudgetExhausted):
        fit_or_degrade(Proposal(8, 16, 1), cap_ms=10.0, cfg=REFERENCE_CFG)


def test_fit_or_degrade_refuses_when_no_combination_fits():
    # Cap so tight nothing fits.
    with pytest.raises(BudgetExhausted):
        fit_or_degrade(Proposal(8, 12, 3), cap_ms=1.0, cfg=REFERENCE_CFG)


def test_max_n_for_latency_inverts_predict():
    """For a given (k, depth, cap), max_n_for_latency returns the
    largest integer n where predict() ≤ cap."""
    cap = 50.0
    for k in (2, 4, 6, 8):
        for depth in (1, 2, 3):
            n_max = max_n_for_latency(k, depth, cap, REFERENCE_CFG)
            if n_max <= 0:
                continue
            assert predict(Proposal(k, n_max, depth), REFERENCE_CFG).latency_ms <= cap
            # n_max + 1 should overrun (or hit some other limit).
            assert predict(Proposal(k, n_max + 1, depth), REFERENCE_CFG).latency_ms > cap


def test_max_n_for_latency_returns_zero_when_fixed_cost_exceeds_cap():
    """When block_fill + decode + interleave already exceed the cap,
    no n satisfies — return 0."""
    # k=8, depth=4 at reference: block_fill=11.2 + decode=1 + 36 interleave = 48.2 ms.
    # Cap at 5 ms → 0.
    assert max_n_for_latency(8, 4, cap_ms=5.0, cfg=REFERENCE_CFG) == 0
