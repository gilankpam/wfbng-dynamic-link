"""Phase 3 debug-suite feature flags.

Parsed from the `debug:` block of gs.yaml. Master switch + per-feature
overrides; per-feature defaults to follow the master unless explicitly
set in YAML. Production deploys leave the whole block off.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

log = logging.getLogger(__name__)


@dataclass
class DebugConfig:
    enabled: bool = False
    ping_pong: bool = False
    latency_log: bool = False
    video_tap: bool = False
    video_tap_port: int = 5600

    @classmethod
    def from_yaml(cls, raw: dict) -> "DebugConfig":
        block = raw.get("debug") or {}
        master = bool(block.get("enabled", False))

        def _resolve(key: str) -> bool:
            v = block.get(key)
            if v is None:
                return master
            return bool(v)

        cfg = cls(
            enabled=master,
            ping_pong=_resolve("ping_pong"),
            latency_log=_resolve("latency_log"),
            video_tap=_resolve("video_tap"),
            video_tap_port=int(block.get("video_tap_port", 5600)),
        )

        if cfg.latency_log and not cfg.ping_pong:
            log.warning(
                "debug: latency_log requires ping_pong; "
                "disabling latency_log",
            )
            cfg.latency_log = False

        return cfg

    def log_resolution(self) -> None:
        log.info(
            "debug flags: enabled=%s ping_pong=%s latency_log=%s "
            "video_tap=%s video_tap_port=%d",
            self.enabled, self.ping_pong, self.latency_log,
            self.video_tap, self.video_tap_port,
        )
