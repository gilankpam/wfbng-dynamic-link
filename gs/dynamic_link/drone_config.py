"""GS-side drone-config state machine.

Tracks the drone's authoritative `(mtu_bytes, fps, generation_id)`,
gates the policy's emit path on whether a HELLO has been received,
and detects drone reboot via `generation_id` change.

See `docs/superpowers/specs/2026-05-11-drone-config-handshake-and-dynamic-fec-design.md`.
"""
from __future__ import annotations

import enum
import logging
from dataclasses import dataclass

from .wire import Hello, HelloAck, HELLO_FLAG_VANILLA_WFB_NG

log = logging.getLogger(__name__)


class State(enum.Enum):
    AWAITING = "awaiting"
    SYNCED = "synced"


class DroneConfigEvent(enum.Enum):
    """Returned from `on_hello` to let callers decide what to log."""
    SYNCED = "synced"               # AWAITING → SYNCED
    REBOOT_DETECTED = "reboot"      # SYNCED → SYNCED with new gen_id
    ALREADY_SYNCED = "noop"         # same gen_id as current


@dataclass
class DroneConfigState:
    state: State = State.AWAITING
    generation_id: int | None = None
    mtu_bytes: int | None = None
    fps: int | None = None
    applier_build_sha: int | None = None
    # Derived from HELLO flags. True until a HELLO clears it; once a
    # vanilla-flagged HELLO arrives, stays False until a non-vanilla
    # HELLO replaces it (e.g. drone rebuilt with the custom branch and
    # rebooted → new generation_id, flags cleared).
    interleaving_supported: bool = True

    def is_synced(self) -> bool:
        return self.state is State.SYNCED

    def on_hello(self, h: Hello) -> DroneConfigEvent:
        """Apply a received HELLO. Returns an event describing the
        transition (for caller-side logging)."""
        if self.state is State.AWAITING:
            self._adopt(h)
            log.info(
                "drone_config sync gen=0x%08x mtu=%d fps=%d sha=0x%08x interleaver=%s",
                h.generation_id, h.mtu_bytes, h.fps, h.applier_build_sha,
                "enabled" if self.interleaving_supported else "vanilla",
            )
            return DroneConfigEvent.SYNCED
        # SYNCED
        if h.generation_id == self.generation_id:
            return DroneConfigEvent.ALREADY_SYNCED
        log.warning(
            "drone_reboot_detected old_gen=0x%08x new_gen=0x%08x "
            "new_mtu=%d new_fps=%d new_interleaver=%s",
            self.generation_id, h.generation_id, h.mtu_bytes, h.fps,
            "enabled" if (h.flags & HELLO_FLAG_VANILLA_WFB_NG) == 0 else "vanilla",
        )
        self._adopt(h)
        return DroneConfigEvent.REBOOT_DETECTED

    def build_ack(self) -> HelloAck | None:
        """Return a HelloAck for the current generation_id, or None if
        we haven't seen any HELLO yet."""
        if self.generation_id is None:
            return None
        return HelloAck(generation_id_echo=self.generation_id)

    def _adopt(self, h: Hello) -> None:
        self.state = State.SYNCED
        self.generation_id = h.generation_id
        self.mtu_bytes = h.mtu_bytes
        self.fps = h.fps
        self.applier_build_sha = h.applier_build_sha
        self.interleaving_supported = (
            (h.flags & HELLO_FLAG_VANILLA_WFB_NG) == 0
        )
