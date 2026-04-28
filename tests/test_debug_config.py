"""Tests for the GS-side `debug:` block parser."""
from __future__ import annotations

import logging

import pytest

from dynamic_link.debug_config import DebugConfig


def test_no_debug_block_all_off():
    cfg = DebugConfig.from_yaml({})
    assert cfg.enabled is False
    assert cfg.ping_pong is False
    assert cfg.latency_log is False
    assert cfg.video_tap is False
    assert cfg.video_tap_port == 5600


def test_master_off_per_feature_off_by_default():
    cfg = DebugConfig.from_yaml({"debug": {"enabled": False}})
    assert cfg.enabled is False
    assert cfg.ping_pong is False
    assert cfg.latency_log is False
    assert cfg.video_tap is False


def test_master_on_all_features_on_by_default():
    cfg = DebugConfig.from_yaml({"debug": {"enabled": True}})
    assert cfg.enabled is True
    assert cfg.ping_pong is True
    assert cfg.latency_log is True
    assert cfg.video_tap is True


def test_per_feature_override_off_when_master_on():
    cfg = DebugConfig.from_yaml({"debug": {"enabled": True, "ping_pong": False,
                                            "latency_log": False}})
    assert cfg.ping_pong is False
    # latency_log requires ping_pong → coherence kicks in regardless
    assert cfg.latency_log is False
    assert cfg.video_tap is True


def test_per_feature_override_on_when_master_off():
    cfg = DebugConfig.from_yaml({"debug": {"enabled": False, "video_tap": True}})
    assert cfg.enabled is False
    assert cfg.ping_pong is False
    assert cfg.video_tap is True


def test_video_tap_port_override():
    cfg = DebugConfig.from_yaml({"debug": {"enabled": True, "video_tap_port": 6000}})
    assert cfg.video_tap_port == 6000


def test_latency_log_disabled_without_ping_pong(caplog):
    with caplog.at_level(logging.WARNING):
        cfg = DebugConfig.from_yaml({"debug": {"enabled": True,
                                                "ping_pong": False,
                                                "latency_log": True}})
    assert cfg.ping_pong is False
    assert cfg.latency_log is False
    assert any("latency_log requires ping_pong" in r.message for r in caplog.records)


def test_log_resolution_runs(caplog):
    with caplog.at_level(logging.INFO):
        cfg = DebugConfig.from_yaml({"debug": {"enabled": True}})
        cfg.log_resolution()
    assert any("debug flags" in r.message for r in caplog.records)
