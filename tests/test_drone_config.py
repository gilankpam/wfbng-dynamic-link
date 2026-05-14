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


@pytest.mark.asyncio
async def test_tunnel_listener_dispatches_hello_to_handler():
    """Bind a TunnelListener on an ephemeral port, send a wire-encoded
    HELLO from a UDP socket, verify the handler fires."""
    import asyncio
    from socket import AF_INET, SOCK_DGRAM, socket as Socket

    from dynamic_link.tunnel_listener import TunnelListener
    from dynamic_link.wire import encode_hello

    received: list[Hello] = []
    listener = TunnelListener(
        "127.0.0.1", 0,  # 0 = ephemeral
        on_pong=lambda *_: None,
        on_hello=lambda h: received.append(h),
    )
    await listener.start()
    try:
        transport = listener._transport
        assert transport is not None
        sockname = transport.get_extra_info("sockname")
        port = sockname[1]

        sender = Socket(AF_INET, SOCK_DGRAM)
        try:
            sender.sendto(
                encode_hello(Hello(generation_id=0x42, mtu_bytes=1400,
                                   fps=30, applier_build_sha=0)),
                ("127.0.0.1", port),
            )
            # Let the event loop process the datagram.
            await asyncio.sleep(0.05)
        finally:
            sender.close()
    finally:
        listener.stop()

    assert len(received) == 1
    assert received[0].generation_id == 0x42
    assert received[0].mtu_bytes == 1400
    assert received[0].fps == 30


def test_default_interleaving_supported_is_true():
    """Before any HELLO arrives, assume capable (today's default)."""
    s = DroneConfigState()
    assert s.interleaving_supported is True


def test_hello_without_vanilla_flag_keeps_interleaving_supported_true():
    from dynamic_link.wire import Hello
    s = DroneConfigState()
    s.on_hello(Hello(generation_id=0xCAFEBABE,
                     mtu_bytes=3994, fps=60,
                     applier_build_sha=0xDEADBEEF,
                     flags=0))
    assert s.interleaving_supported is True


def test_hello_with_vanilla_flag_sets_interleaving_supported_false():
    from dynamic_link.wire import Hello, HELLO_FLAG_VANILLA_WFB_NG
    s = DroneConfigState()
    s.on_hello(Hello(generation_id=0xCAFEBABE,
                     mtu_bytes=3994, fps=60,
                     applier_build_sha=0xDEADBEEF,
                     flags=HELLO_FLAG_VANILLA_WFB_NG))
    assert s.interleaving_supported is False


def test_hello_flips_back_to_true_on_reboot_into_capable_build():
    """A drone rebuilt with the custom branch (new generation_id, flags=0)
    must clear the vanilla flag."""
    from dynamic_link.wire import Hello, HELLO_FLAG_VANILLA_WFB_NG
    s = DroneConfigState()
    s.on_hello(Hello(generation_id=0x1111, mtu_bytes=3994, fps=60,
                     flags=HELLO_FLAG_VANILLA_WFB_NG))
    assert s.interleaving_supported is False
    s.on_hello(Hello(generation_id=0x2222, mtu_bytes=3994, fps=60,
                     flags=0))
    assert s.interleaving_supported is True
