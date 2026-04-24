"""Latency-budget predictor (§4 "Latency budget enforcement").

Formula from wfb-ng/doc/design/fec-enhancements-v2.md §4.2:

    block_fill_time = k × inter_packet_interval
    block_airtime   = n × per_packet_airtime
    fec_decode_time ≈ 1 ms at (k=8, n=12) on zfex SIMD
    latency_block   = block_fill_time + block_airtime + fec_decode_time
    latency_total   = latency_block + (depth − 1) × block_duration

Reference worked example (MCS7 HT40 @ 8 Mb/s H.264, per_packet_airtime
= 80 µs, inter_packet_interval = 1.4 ms):

    (k=8, n=12) d=1 = 12 ms     (block_duration baseline)
    (k=8, n=14) d=3 = 37 ms     (under the 50 ms hard cap)
    (k=8, n=14) d=4 = 49 ms     (at the cap)

All functions are pure / stateless.
"""
from __future__ import annotations

from dataclasses import dataclass

# Step 3 of §4.2 ladder: at n_max of a k-band with loss still non-zero,
# drop to the next lower band's floor.
LADDER_STEPS = {
    8: [(8, 12), (8, 14), (8, 16)],
    6: [(6, 10), (6, 12), (6, 14)],
    4: [(4, 8),  (4, 10), (4, 12)],
    2: [(2, 4),  (2, 6),  (2, 8)],
}
# Step 3 drop (k, n) destinations, mirroring §4.2 table.
LADDER_DROP = {8: (6, 12), 6: (4, 8), 4: (2, 6), 2: (1, 4)}


@dataclass(frozen=True)
class Proposal:
    """A (k, n, depth) proposal the controller is about to apply."""
    k: int
    n: int
    depth: int


@dataclass(frozen=True)
class Prediction:
    latency_ms: float
    block_fill_ms: float
    block_airtime_ms: float
    fec_decode_ms: float
    interleave_ms: float


@dataclass(frozen=True)
class PredictorConfig:
    # From gs.yaml `video.per_packet_airtime_us` (airtime per 1400B packet).
    per_packet_airtime_us: float = 80.0
    # Derived from encoder bitrate + MTU. For the reference 8 Mb/s / MCS7
    # HT40 point this is 1.4 ms.
    inter_packet_interval_ms: float = 1.4
    # Constant small decode latency.
    fec_decode_ms: float = 1.0
    # Depth overhead unit — one block cycle at the reference operating
    # point. Matches the "+interleave" column of the §4.2 worked example.
    block_duration_ms: float = 12.0


def predict(proposal: Proposal, cfg: PredictorConfig) -> Prediction:
    block_fill = proposal.k * cfg.inter_packet_interval_ms
    block_air = proposal.n * cfg.per_packet_airtime_us / 1000.0
    decode = cfg.fec_decode_ms
    interleave = (proposal.depth - 1) * cfg.block_duration_ms
    total = block_fill + block_air + decode + interleave
    return Prediction(
        latency_ms=total,
        block_fill_ms=block_fill,
        block_airtime_ms=block_air,
        fec_decode_ms=decode,
        interleave_ms=interleave,
    )


class BudgetExhausted(Exception):
    """No (k, n, depth) combination fits the cap."""


def fit_or_degrade(
    proposal: Proposal,
    cap_ms: float,
    cfg: PredictorConfig,
) -> Proposal:
    """Auto-adjust downward until we fit, or raise BudgetExhausted.

    Priority order per §4:
      1. drop depth by 1 (reclaims one block_duration).
      2. if depth already 1, drop k to next lower band
         (rebase n to that band's floor).
      3. if already (2, 4, 1) and still over cap → refuse.
    """
    current = proposal
    while True:
        p = predict(current, cfg)
        if p.latency_ms <= cap_ms:
            return current
        if current.depth > 1:
            current = Proposal(k=current.k, n=current.n, depth=current.depth - 1)
            continue
        # depth == 1; try to drop to next-lower band's floor.
        if current.k in LADDER_DROP:
            next_k, next_n = LADDER_DROP[current.k]
            # k=1 isn't a real operating band — refuse rather than land there.
            if next_k < 2:
                raise BudgetExhausted(
                    f"no fit for cap={cap_ms} ms starting from {proposal}"
                )
            current = Proposal(k=next_k, n=next_n, depth=1)
            continue
        raise BudgetExhausted(
            f"no fit for cap={cap_ms} ms starting from {proposal}"
        )
