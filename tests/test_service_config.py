"""Unit tests for service-level config loaders.

Focused on `_build_policy_config` config resolution behavior — fields
that have computed defaults, warnings on operator misconfig, etc.
"""
from __future__ import annotations

import logging

from dynamic_link.service import _build_policy_config


def _yaml(**fec_over):
    """Minimal raw config dict — same shape as PyYAML would emit."""
    fec = dict(
        depth_max=1,
        base_redundancy_ratio=0.4,
        max_redundancy_ratio=1.0,
        k_bounds=dict(min=2, max=50),
    )
    fec.update(fec_over)
    return dict(
        wfb_ng=dict(stats_api="tcp://127.0.0.1:8103"),
        leading_loop=dict(radio_profile="m8812eu2"),
        video=dict(framerate=60),
        fec=fec,
        policy=dict(bitrate=dict(utilization_factor=0.7)),
    )


def test_blocks_per_frame_default_is_one_plus_max_red():
    raw = _yaml(max_redundancy_ratio=1.0)   # no blocks_per_frame
    cfg = _build_policy_config(raw)
    assert cfg.dynamic_fec.blocks_per_frame == 2.0


def test_blocks_per_frame_default_tracks_max_red():
    raw = _yaml(max_redundancy_ratio=0.5)
    cfg = _build_policy_config(raw)
    assert cfg.dynamic_fec.blocks_per_frame == 1.5


def test_blocks_per_frame_explicit_override():
    raw = _yaml(max_redundancy_ratio=1.0, blocks_per_frame=3.0)
    cfg = _build_policy_config(raw)
    assert cfg.dynamic_fec.blocks_per_frame == 3.0


def test_blocks_per_frame_below_hard_bound_warns(caplog):
    raw = _yaml(max_redundancy_ratio=1.0, blocks_per_frame=1.0)
    with caplog.at_level(logging.WARNING, logger="dynamic_link"):
        cfg = _build_policy_config(raw)
    assert cfg.dynamic_fec.blocks_per_frame == 1.0
    assert any(
        "blocks_per_frame" in rec.message and "below" in rec.message.lower()
        for rec in caplog.records
    ), f"expected under-spec warning; got: {[r.message for r in caplog.records]}"


def test_blocks_per_frame_at_hard_bound_silent(caplog):
    raw = _yaml(max_redundancy_ratio=1.0, blocks_per_frame=2.0)
    with caplog.at_level(logging.WARNING, logger="dynamic_link"):
        _build_policy_config(raw)
    assert not any(
        "blocks_per_frame" in rec.message for rec in caplog.records
    )


def test_blocks_per_frame_above_hard_bound_silent(caplog):
    raw = _yaml(max_redundancy_ratio=1.0, blocks_per_frame=3.0)
    with caplog.at_level(logging.WARNING, logger="dynamic_link"):
        _build_policy_config(raw)
    assert not any(
        "blocks_per_frame" in rec.message for rec in caplog.records
    )
