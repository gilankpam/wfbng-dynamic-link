"""Dynamic-link GS service entry point (Phase 0 observer)."""
from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from pathlib import Path

import yaml

from .policy import (
    CooldownConfig,
    FECBounds,
    LeadingLoopConfig,
    Policy,
    PolicyConfig,
    SafeDefaults,
)
from .predictor import PredictorConfig
from .profile import load_profile
from .return_link import ReturnLink
from .signals import SignalAggregator
from .sinks import LogSink
from .stats_client import (
    ReplayClient,
    RxEvent,
    SessionEvent,
    SettingsEvent,
    StatsClient,
    TxEvent,
)
from .wire import Encoder as WireEncoder

log = logging.getLogger("dynamic_link")


DEFAULT_PACKAGED_RADIOS_DIR = Path(__file__).resolve().parent.parent.parent / "conf" / "radios"


def _load_yaml(path: Path) -> dict:
    with open(path, "r") as fd:
        return yaml.safe_load(fd) or {}


def _build_policy_config(raw: dict) -> PolicyConfig:
    leading_raw = raw.get("leading_loop", {})
    cooldown_raw = raw.get("cooldown", {})
    fec_raw = raw.get("fec", {})
    safe_raw = raw.get("safe_defaults", {})
    video_raw = raw.get("video", {})

    leading = LeadingLoopConfig(
        bandwidth=int(leading_raw.get("bandwidth", 20)),
        mcs_max=int(leading_raw.get("mcs_max", 7)),
        rssi_margin_db=float(leading_raw.get("rssi_margin_db", 8.0)),
        rssi_up_guard_db=float(leading_raw.get("rssi_up_guard_db", 3.0)),
        rssi_up_hold_ms=float(leading_raw.get("rssi_up_hold_ms", 2000.0)),
        rssi_down_hold_ms=float(leading_raw.get("rssi_down_hold_ms", 500.0)),
        rssi_target_dBm=float(leading_raw.get("rssi_target_dBm", -60.0)),
        rssi_deadband_db=float(leading_raw.get("rssi_deadband_db", 3.0)),
        tx_power_min_dBm=float(leading_raw.get("tx_power_min_dBm", 5.0)),
        tx_power_max_dBm=float(leading_raw.get("tx_power_max_dBm", 23.0)),
        tx_power_cooldown_ms=float(leading_raw.get("tx_power_cooldown_ms", 1000.0)),
        tx_power_freeze_after_mcs_ms=float(
            leading_raw.get("tx_power_freeze_after_mcs_ms", 2000.0)
        ),
        tx_power_step_max_db=float(leading_raw.get("tx_power_step_max_db", 3.0)),
        tx_power_gain_up_db=float(leading_raw.get("tx_power_gain_up_db", 1.0)),
        tx_power_gain_down_db=float(leading_raw.get("tx_power_gain_down_db", 1.0)),
    )
    cooldown = CooldownConfig(
        min_change_interval_ms_fec=float(
            cooldown_raw.get("min_change_interval_ms_fec", 200.0)
        ),
        min_change_interval_ms_depth=float(
            cooldown_raw.get("min_change_interval_ms_depth", 200.0)
        ),
        min_change_interval_ms_radio=float(
            cooldown_raw.get("min_change_interval_ms_radio", 500.0)
        ),
        min_change_interval_ms_cross=float(
            cooldown_raw.get("min_change_interval_ms_cross", 50.0)
        ),
    )
    fec = FECBounds(
        n_min=int(fec_raw.get("n_min", 4)),
        n_max=int(fec_raw.get("n_max", 16)),
        k_min=int(fec_raw.get("k_min", 2)),
        k_max=int(fec_raw.get("k_max", 8)),
        depth_max=int(fec_raw.get("depth_max", 3)),
    )
    safe_video = safe_raw.get("video", {})
    safe = SafeDefaults(
        k=int(safe_video.get("k", 8)),
        n=int(safe_video.get("n", 12)),
        depth=int(safe_raw.get("depth", 1)),
        mcs=int(safe_raw.get("mcs", 1)),
        bitrate_kbps=int(safe_raw.get("bitrate_kbps", 2000)),
    )
    predictor = PredictorConfig(
        per_packet_airtime_us=float(video_raw.get("per_packet_airtime_us", 80.0)),
    )
    return PolicyConfig(
        leading=leading,
        cooldown=cooldown,
        fec=fec,
        safe=safe,
        predictor=predictor,
        max_latency_ms=float(video_raw.get("max_latency_ms", 50.0)),
    )


def _build_aggregator(raw: dict) -> SignalAggregator:
    s = raw.get("smoothing", {})
    return SignalAggregator(
        ewma_alpha_rssi=float(s.get("ewma_alpha_rssi", 0.2)),
        ewma_alpha_fec=float(s.get("ewma_alpha_fec", 0.2)),
        ewma_alpha_burst=float(s.get("ewma_alpha_burst", 0.1)),
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="dynamic-link-gs",
        description="Dynamic-link GS controller (Phase 0 observer, log-only).",
    )
    p.add_argument(
        "--config",
        required=True,
        type=Path,
        help="Path to gs.yaml configuration.",
    )
    p.add_argument(
        "--log-file",
        type=Path,
        help="Path to write the decision log. Default: stdout.",
    )
    p.add_argument(
        "--replay",
        type=Path,
        help="Replay from a captured JSONL file instead of connecting over TCP.",
    )
    p.add_argument(
        "--verbose",
        action="store_true",
        help="Log every tick (not just ticks with a knob change).",
    )
    p.add_argument(
        "--log-level",
        default="INFO",
        help="Python log level (DEBUG, INFO, WARNING, ERROR).",
    )
    return p.parse_args(argv)


async def _run(args: argparse.Namespace) -> int:
    raw = _load_yaml(args.config)
    wfb = raw.get("wfb_ng", {})
    endpoint = wfb.get("stats_api", "tcp://127.0.0.1:8103")

    leading_raw = raw.get("leading_loop", {})
    profile_name = leading_raw.get("radio_profile", "m8812eu2")
    override_dir = leading_raw.get("radio_profiles_dir")
    search_dirs: list[Path] = []
    if override_dir:
        search_dirs.append(Path(override_dir))
    search_dirs.append(DEFAULT_PACKAGED_RADIOS_DIR)
    profile = load_profile(profile_name, search_dirs)
    log.info("loaded radio profile %s (%s)", profile.name, profile.chipset)

    policy_cfg = _build_policy_config(raw)
    policy = Policy(policy_cfg, profile)
    aggregator = _build_aggregator(raw)

    sink_stream = None
    if args.log_file is not None:
        sink_stream = open(args.log_file, "a", buffering=1)
    sink = LogSink(stream=sink_stream, verbose=args.verbose)

    enabled = bool(raw.get("enabled", False))

    # ---- Phase 2 wiring ----
    return_link: ReturnLink | None = None
    wire_encoder: WireEncoder | None = None
    if enabled:
        drone_addr = wfb.get("drone_addr", "10.5.0.2")
        drone_port = int(wfb.get("drone_port", 5800))
        return_link = ReturnLink(drone_addr, drone_port)
        wire_encoder = WireEncoder(seq=1)
        log.info("enabled=true; emitting decisions to %s:%d",
                 drone_addr, drone_port)
    else:
        log.info("enabled=false; observer mode (no wire emit)")

    mavlink_reader = None
    mav_local_addr = wfb.get("mavlink_local_addr", "127.0.0.1")
    mav_local_port = int(wfb.get("mavlink_local_port", 14550))
    try:
        from .mavlink_status import MAVLinkStatusReader
        mavlink_reader = MAVLinkStatusReader(
            mav_local_addr, mav_local_port,
            on_event=lambda ev: None,  # logged inside the reader
        )
        await mavlink_reader.start()
    except ImportError as e:
        log.warning("mavlink_status: unavailable (%s); "
                    "drone→GS status channel will not surface to the GS log", e)
    except OSError as e:
        log.warning("mavlink_status: bind %s:%d failed: %s",
                    mav_local_addr, mav_local_port, e)
        mavlink_reader = None

    def on_event(ev):
        if isinstance(ev, SettingsEvent):
            log.info(
                "settings: profile=%s wlans=%s", ev.profile, ev.wlans
            )
            return
        if isinstance(ev, SessionEvent):
            aggregator.update_session(ev.session)
            return
        if isinstance(ev, TxEvent):
            return
        if isinstance(ev, RxEvent):
            signals = aggregator.consume(ev)
            decision = policy.tick(signals)
            sink.write(decision)
            if enabled and return_link is not None and wire_encoder is not None:
                packet = wire_encoder.encode(decision)
                return_link.send(packet)

    if args.replay is not None:
        client = ReplayClient(str(args.replay), on_event)
    else:
        client = StatsClient(endpoint, on_event)

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _handle_signal():
        log.info("shutdown signal received")
        client.stop()
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _handle_signal)
        except NotImplementedError:
            pass

    try:
        await client.run()
    finally:
        sink.close()
        if return_link is not None:
            return_link.close()
        if mavlink_reader is not None:
            mavlink_reader.stop()

    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        return asyncio.run(_run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
