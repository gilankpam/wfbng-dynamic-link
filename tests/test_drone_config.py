"""Unit tests for the GS-side drone-config state machine."""
from __future__ import annotations

import pytest

from dynamic_link.drone_config import (
    DroneConfigEvent,
    DroneConfigState,
    State,
)
from dynamic_link.wire import Hello


def _hello(gen: int = 0xCAFEBABE, mtu: int = 3994, fps: int = 60) -> Hello:
    return Hello(generation_id=gen, mtu_bytes=mtu, fps=fps,
                 applier_build_sha=0xDEADBEEF)


def test_initial_state_is_awaiting():
    s = DroneConfigState()
    assert s.state is State.AWAITING
    assert s.mtu_bytes is None
    assert s.fps is None
    assert s.generation_id is None
    assert not s.is_synced()


def test_first_hello_transitions_to_synced():
    s = DroneConfigState()
    events = []
    ev = s.on_hello(_hello())
    assert s.state is State.SYNCED
    assert s.is_synced()
    assert s.mtu_bytes == 3994
    assert s.fps == 60
    assert s.generation_id == 0xCAFEBABE
    assert ev is DroneConfigEvent.SYNCED


def test_subsequent_hellos_same_genid_are_idempotent():
    s = DroneConfigState()
    s.on_hello(_hello())
    ev = s.on_hello(_hello())
    assert s.state is State.SYNCED
    assert ev is DroneConfigEvent.ALREADY_SYNCED


def test_hello_with_new_genid_triggers_reboot_and_resync():
    s = DroneConfigState()
    s.on_hello(_hello(gen=0x1111))
    ev = s.on_hello(_hello(gen=0x2222, mtu=1500, fps=90))
    assert s.state is State.SYNCED   # immediately re-syncs
    assert s.generation_id == 0x2222
    assert s.mtu_bytes == 1500
    assert s.fps == 90
    assert ev is DroneConfigEvent.REBOOT_DETECTED


def test_build_ack_returns_helloack_for_current_genid():
    s = DroneConfigState()
    s.on_hello(_hello(gen=0xABCD))
    ack = s.build_ack()
    assert ack is not None
    assert ack.generation_id_echo == 0xABCD


def test_build_ack_returns_none_when_awaiting():
    s = DroneConfigState()
    assert s.build_ack() is None
