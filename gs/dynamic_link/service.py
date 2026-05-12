"""Dynamic-link GS service entry point (Phase 0 observer)."""
from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
import time
from pathlib import Path

import yaml

from . import wire
from .debug_config import DebugConfig
from .drone_config import DroneConfigEvent, DroneConfigState
from .flight_log import FlightDirRotator
from .latency_sink import LatencySink
from .observed import derive_observed
from .policy import (
    CooldownConfig,
    FECBounds,
    GateConfig,
    LeadingLoopConfig,
    Policy,
    PolicyConfig,
    ProfileSelectionConfig,
    SafeDefaults,
)
from .bitrate import BitrateConfig
from .dynamic_fec import DynamicFecConfig
from .idr_burst import IdrBurstConfig, IdrBurster
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
from .timesync import TimeSync
from .tunnel_listener import TunnelListener
from .video_tap import VideoRtpSink, VideoTap
from .wire import Encoder as WireEncoder, encode_hello_ack

log = logging.getLogger("dynamic_link")


DEFAULT_PACKAGED_RADIOS_DIR = Path(__file__).resolve().parent.parent.parent / "conf" / "radios"


def _load_yaml(path: Path) -> dict:
    with open(path, "r") as fd:
        return yaml.safe_load(fd) or {}


_DEPRECATED_LEADING_KEYS = {
    # Old hysteresis / inhibit knobs — superseded by gate / profile_selection.
    "snr_margin_db", "snr_up_guard_db", "snr_up_hold_ms", "snr_down_hold_ms",
    "loss_margin_weight", "fec_margin_weight", "forced_drop_inhibit_ms",
    "mcs_max",  # moved to gate.max_mcs
    # Old RSSI closed loop / closed-loop power knobs — fully retired.
    "rssi_margin_db", "rssi_up_guard_db", "rssi_up_hold_ms",
    "rssi_down_hold_ms", "rssi_target_dBm", "rssi_deadband_db",
    "tx_power_cooldown_ms", "tx_power_freeze_after_mcs_ms",
    "tx_power_step_max_db", "tx_power_gain_up_db", "tx_power_gain_down_db",
}


def _build_policy_config(raw: dict) -> PolicyConfig:
    leading_raw = raw.get("leading_loop", {})
    gate_raw = raw.get("gate", {})
    selection_raw = raw.get("profile_selection", {})
    cooldown_raw = raw.get("cooldown", {})
    fec_raw = raw.get("fec", {})
    safe_raw = raw.get("safe_defaults", {})
    video_raw = raw.get("video", {})

    deprecated_present = sorted(
        k for k in _DEPRECATED_LEADING_KEYS if k in leading_raw
    )
    if deprecated_present:
        log.warning(
            "leading_loop has deprecated keys (ignored): %s. "
            "Migrate to the `gate:` / `profile_selection:` sections.",
            ", ".join(deprecated_present),
        )

    leading = LeadingLoopConfig(
        bandwidth=int(leading_raw.get("bandwidth", 20)),
        tx_power_min_dBm=float(leading_raw.get("tx_power_min_dBm", 5.0)),
        tx_power_max_dBm=float(leading_raw.get("tx_power_max_dBm", 23.0)),
        # Deprecated keys: parse-and-store so they round-trip but the
        # selector ignores them. Defaults match the old values so any
        # in-tree consumer relying on them still gets sensible numbers.
        mcs_max=int(leading_raw.get("mcs_max", 7)),
        snr_margin_db=float(leading_raw.get("snr_margin_db", 3.0)),
        snr_up_guard_db=float(leading_raw.get("snr_up_guard_db", 2.0)),
        snr_up_hold_ms=float(leading_raw.get("snr_up_hold_ms", 2000.0)),
        snr_down_hold_ms=float(leading_raw.get("snr_down_hold_ms", 500.0)),
        loss_margin_weight=float(leading_raw.get("loss_margin_weight", 20.0)),
        fec_margin_weight=float(leading_raw.get("fec_margin_weight", 20.0)),
        forced_drop_inhibit_ms=float(
            leading_raw.get("forced_drop_inhibit_ms", 5000.0)
        ),
        rssi_up_guard_db=float(leading_raw.get("rssi_up_guard_db", 3.0)),
        rssi_up_hold_ms=float(leading_raw.get("rssi_up_hold_ms", 2000.0)),
        rssi_down_hold_ms=float(leading_raw.get("rssi_down_hold_ms", 500.0)),
        rssi_target_dBm=float(leading_raw.get("rssi_target_dBm", -60.0)),
        rssi_deadband_db=float(leading_raw.get("rssi_deadband_db", 3.0)),
        tx_power_cooldown_ms=float(
            leading_raw.get("tx_power_cooldown_ms", 1000.0)
        ),
        tx_power_freeze_after_mcs_ms=float(
            leading_raw.get("tx_power_freeze_after_mcs_ms", 2000.0)
        ),
        tx_power_step_max_db=float(
            leading_raw.get("tx_power_step_max_db", 3.0)
        ),
        tx_power_gain_up_db=float(leading_raw.get("tx_power_gain_up_db", 1.0)),
        tx_power_gain_down_db=float(
            leading_raw.get("tx_power_gain_down_db", 1.0)
        ),
    )

    gate = GateConfig(
        snr_ema_alpha=float(gate_raw.get("snr_ema_alpha", 0.3)),
        snr_slope_alpha=float(gate_raw.get("snr_slope_alpha", 0.3)),
        snr_predict_horizon_ticks=float(
            gate_raw.get("snr_predict_horizon_ticks", 3.0)
        ),
        snr_safety_margin=float(gate_raw.get("snr_safety_margin", 3.0)),
        loss_margin_weight=float(gate_raw.get("loss_margin_weight", 20.0)),
        fec_margin_weight=float(gate_raw.get("fec_margin_weight", 5.0)),
        hysteresis_up_db=float(gate_raw.get("hysteresis_up_db", 2.5)),
        hysteresis_down_db=float(gate_raw.get("hysteresis_down_db", 1.0)),
        emergency_loss_rate=float(gate_raw.get("emergency_loss_rate", 0.05)),
        emergency_fec_pressure=float(
            gate_raw.get("emergency_fec_pressure", 0.80)
        ),
        max_mcs=int(gate_raw.get("max_mcs", 7)),
        max_mcs_step_up=int(gate_raw.get("max_mcs_step_up", 1)),
    )

    selection = ProfileSelectionConfig(
        hold_fallback_mode_ms=int(
            selection_raw.get("hold_fallback_mode_ms", 1000)
        ),
        hold_modes_down_ms=int(selection_raw.get("hold_modes_down_ms", 2000)),
        min_between_changes_ms=int(
            selection_raw.get("min_between_changes_ms", 200)
        ),
        fast_downgrade=bool(selection_raw.get("fast_downgrade", True)),
        upward_confidence_loops=int(
            selection_raw.get("upward_confidence_loops", 4)
        ),
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
        depth_max=int(fec_raw.get("depth_max", 3)),
    )

    fec_kbounds_raw = fec_raw.get("k_bounds", {})
    dynamic_fec = DynamicFecConfig(
        k_min=int(fec_kbounds_raw.get("min", 4)),
        k_max=int(fec_kbounds_raw.get("max", 16)),
        base_redundancy_ratio=float(fec_raw.get("base_redundancy_ratio", 0.5)),
        max_redundancy_ratio=float(fec_raw.get("max_redundancy_ratio", 1.0)),
        n_loss_threshold=float(fec_raw.get("n_loss_threshold", 0.02)),
        n_loss_windows=int(fec_raw.get("n_loss_windows", 3)),
        n_loss_step=int(fec_raw.get("n_loss_step", 1)),
        n_recover_windows=int(fec_raw.get("n_recover_windows", 10)),
        n_recover_step=int(fec_raw.get("n_recover_step", 1)),
        max_n_escalation=int(fec_raw.get("max_n_escalation", 4)),
    )

    # Legacy fec.* keys: present in old gs.yaml configs but no longer
    # wired. Log a warning so the operator cleans them up.
    _legacy_fec_keys = (
        "mtu_bytes",                    # now drone-reported via DLHE (P4a)
        "fec_block_fill_ms_target",     # removed during the static-table era
        "n_min", "n_preempt_step",      # removed during the static-table era
    )
    _legacy_fec_reasons = {
        "mtu_bytes": "MTU is now reported by the drone at runtime",
        "fec_block_fill_ms_target": "block-fill is now bounded by k_bounds.max",
        "n_min": "absorbed into k_bounds.min",
        "n_preempt_step": "preemptive escalation removed",
    }
    for k in _legacy_fec_keys:
        if k in fec_raw:
            log.warning(
                "config: ignoring legacy fec.%s — %s", k,
                _legacy_fec_reasons.get(k, "deprecated"),
            )

    # Legacy encoder keys: encoder.fps moved drone-side.
    encoder_raw = raw.get("encoder", {})
    if "fps" in encoder_raw:
        log.warning(
            "config: ignoring legacy encoder.fps — FPS is now reported "
            "by the drone via DLHE (P4a)"
        )
    safe_video = safe_raw.get("video", {})
    safe = SafeDefaults(
        k=int(safe_video.get("k", 8)),
        n=int(safe_video.get("n", 12)),
        depth=int(safe_raw.get("depth", 1)),
        mcs=int(safe_raw.get("mcs", 1)),
    )
    predictor = PredictorConfig(
        per_packet_airtime_us=float(video_raw.get("per_packet_airtime_us", 80.0)),
    )
    policy_raw = raw.get("policy", {})
    bitrate_raw = policy_raw.get("bitrate", {})
    try:
        bitrate = BitrateConfig(
            utilization_factor=float(bitrate_raw.get("utilization_factor", 0.8)),
            base_redundancy_ratio=float(bitrate_raw.get(
                "base_redundancy_ratio",
                float(fec_raw.get("base_redundancy_ratio", 0.5)),
            )),
            min_bitrate_kbps=int(bitrate_raw.get("min_bitrate_kbps", 1000)),
            max_bitrate_kbps=int(bitrate_raw.get("max_bitrate_kbps", 24000)),
        )
    except ValueError as e:
        raise ValueError(f"policy.bitrate.{e}") from e
    return PolicyConfig(
        leading=leading,
        gate=gate,
        selection=selection,
        cooldown=cooldown,
        fec=fec,
        safe=safe,
        bitrate=bitrate,
        dynamic_fec=dynamic_fec,
        predictor=predictor,
        max_latency_ms=float(video_raw.get("max_latency_ms", 50.0)),
        starvation_windows=int(policy_raw.get("starvation_windows", 5)),
    )


def _build_idr_burst_config(raw: dict) -> IdrBurstConfig:
    policy_raw = raw.get("policy", {})
    burst_raw = policy_raw.get("idr_burst", {})
    cfg = IdrBurstConfig(
        enabled=bool(burst_raw.get("enabled", True)),
        count=int(burst_raw.get("count", 4)),
        interval_ms=int(burst_raw.get("interval_ms", 20)),
    )
    if cfg.count < 1:
        raise ValueError(
            f"policy.idr_burst.count must be >= 1; got {cfg.count}"
        )
    if cfg.interval_ms <= 0:
        raise ValueError(
            f"policy.idr_burst.interval_ms must be > 0; got {cfg.interval_ms}"
        )
    return cfg


def _build_aggregator(raw: dict) -> SignalAggregator:
    s = raw.get("smoothing", {})
    gate = raw.get("gate", {})
    starv = s.get("starvation_threshold_pps", 50.0)
    # snr_slope alpha lives under [gate] so operators can tune it
    # alongside the other gate knobs; aggregator just consumes it.
    return SignalAggregator(
        ewma_alpha_rssi=float(s.get("ewma_alpha_rssi", 0.2)),
        ewma_alpha_fec=float(s.get("ewma_alpha_fec", 0.2)),
        ewma_alpha_burst=float(s.get("ewma_alpha_burst", 0.1)),
        ewma_alpha_snr_slope=float(gate.get("snr_slope_alpha", 0.3)),
        starvation_threshold_pps=float(starv),
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
        "--log-dir",
        type=Path,
        help="Per-flight log root. The service opens a fresh "
             "flight-NNNN/ subdir on each drone reconnect and closes "
             "it after `debug.flight_rotation.gap_seconds` of empty "
             "rx_ant_stats. Each flight dir holds gs.jsonl, "
             "gs.verbose.jsonl, latency.jsonl, video_rtp.jsonl, and a "
             "flight.json manifest. If omitted, decisions go to "
             "stdout and per-flight streams are disabled.",
    )
    p.add_argument(
        "--video-tap-host",
        default="0.0.0.0",
        help="Bind address for the RTP video tap (default 0.0.0.0).",
    )
    p.add_argument(
        "--replay",
        type=Path,
        help="Replay from a captured JSONL file instead of connecting over TCP.",
    )
    p.add_argument(
        "--verbose",
        action="store_true",
        help="Log every tick on stdout. Useful when --log-dir is "
             "omitted; with --log-dir the verbose stream is already "
             "written into the flight dir.",
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
    drone_config = DroneConfigState()
    policy = Policy(policy_cfg, profile, drone_config=drone_config)
    aggregator = _build_aggregator(raw)

    enabled = bool(raw.get("enabled", False))

    debug_cfg = DebugConfig.from_yaml(raw)
    debug_cfg.log_resolution()

    # Per-flight rotator (Phase 3 forensics). Drives all four sinks
    # via stream getters; between flights the getters return None
    # and writes are dropped.
    rotator: FlightDirRotator | None = None
    if args.log_dir is not None:
        rotator = FlightDirRotator(
            args.log_dir, gap_seconds=debug_cfg.flight_gap_seconds,
        )
        log.info(
            "flight rotation: log_dir=%s gap_seconds=%.1f",
            args.log_dir, debug_cfg.flight_gap_seconds,
        )

    if rotator is not None:
        events_stream = rotator.events_stream
        verbose_stream = rotator.verbose_stream
    else:
        events_stream = sys.stdout
        verbose_stream = sys.stdout if args.verbose else None

    # Phase 3 timesync: ping/pong over the existing tunnel UDP path.
    timesync: TimeSync | None = None
    tunnel_listener: TunnelListener | None = None
    latency_sink: LatencySink | None = None
    if debug_cfg.ping_pong:
        timesync = TimeSync()
    if debug_cfg.latency_log and rotator is not None:
        latency_sink = LatencySink(rotator.latency_stream)
    elif debug_cfg.latency_log:
        log.warning("debug.latency_log=true but --log-dir not supplied; "
                    "latency samples will not be persisted")

    # Phase 3 video tap (B.2).
    video_tap: VideoTap | None = None
    video_rtp_sink: VideoRtpSink | None = None
    if debug_cfg.video_tap:
        if rotator is not None:
            video_rtp_sink = VideoRtpSink(rotator.video_rtp_stream)
        else:
            log.warning("debug.video_tap=true but --log-dir not supplied; "
                        "per-frame RTP records will not be persisted")

    # 1-slot holder for the latest RxEvent — populated in on_event,
    # read by _log_extras to derive the observed block. List used as
    # mutable cell so the closure can rebind without `nonlocal`.
    latest_rx: list[RxEvent | None] = [None]

    def _log_extras() -> dict:
        out: dict = {}
        if timesync is not None and timesync.offset_us is not None:
            out["offset_us"] = timesync.offset_us
            out["offset_stddev_us"] = timesync.offset_stddev_us
        observed = derive_observed(latest_rx[0])
        if observed:
            out["observed"] = observed
        return out

    sink = LogSink(
        events_stream=events_stream,
        verbose_stream=verbose_stream,
        extras_provider=_log_extras,
    )

    # ---- Phase 2 wiring ----
    # return_link is always constructed: P4a HELLO/HELLO-ACK uses it
    # even when decision emit is disabled (observer mode). The cost is
    # one UDP socket. wire_encoder stays gated on `enabled` since it's
    # only fed by the decision path.
    wire_encoder: WireEncoder | None = None
    drone_addr = wfb.get("drone_addr", "10.5.0.2")
    drone_port = int(wfb.get("drone_port", 5800))
    return_link: ReturnLink | None = ReturnLink(drone_addr, drone_port)
    if enabled:
        wire_encoder = WireEncoder(seq=1)
        log.info("enabled=true; emitting decisions to %s:%d",
                 drone_addr, drone_port)
    else:
        log.info("enabled=false; observer mode (no wire emit)")

    idr_burst_cfg = _build_idr_burst_config(raw)
    idr_burster: IdrBurster | None = None
    if enabled and return_link is not None and wire_encoder is not None:
        idr_burster = IdrBurster(idr_burst_cfg, return_link, wire_encoder)
        log.info(
            "idr_burst: enabled=%s count=%d interval_ms=%d",
            idr_burst_cfg.enabled, idr_burst_cfg.count,
            idr_burst_cfg.interval_ms,
        )

    # P4a HELLO/HELLO-ACK callback. Forwards to the state machine and
    # ACKs every received HELLO so ANNOUNCING retries always get a
    # fresh DLHA echo (build_ack is idempotent for the same gen_id).
    def _on_hello(h: wire.Hello) -> None:
        event = drone_config.on_hello(h)
        if event in (DroneConfigEvent.SYNCED, DroneConfigEvent.REBOOT_DETECTED):
            # State machine already logs the transition; this branch is
            # the explicit hook for follow-up actions (none yet).
            pass
        if return_link is None:
            return
        ack = drone_config.build_ack()
        if ack is not None:
            return_link.send_hello_ack(encode_hello_ack(ack))

    # ---- Tunnel listener (always on) ----
    # HELLO reception is core to P4a — bind the listener regardless of
    # `enabled` or `debug.ping_pong`. The pong handler is wired in only
    # when ping/pong is on; the listener treats it as optional.
    gs_listen_addr = wfb.get("gs_tunnel_listen_addr", "0.0.0.0")
    gs_listen_port = int(wfb.get("gs_tunnel_listen_port", 5801))

    _on_pong = None
    if debug_cfg.ping_pong:
        def _on_pong(pong: wire.Pong, t4_us: int) -> None:
            assert timesync is not None
            sample = timesync.observe(
                gs_seq=pong.gs_seq,
                gs_mono_us_t1=pong.gs_mono_us_echo,
                drone_mono_recv_us_t2=pong.drone_mono_recv_us,
                drone_mono_send_us_t3=pong.drone_mono_send_us,
                gs_mono_us_t4=t4_us,
            )
            if latency_sink is not None:
                latency_sink.write(sample)
            log.debug(
                "pong: seq=%d rtt_us=%d offset_us=%s outlier=%s",
                sample.gs_seq, sample.rtt_us,
                sample.offset_us, sample.outlier,
            )

    tunnel_listener = TunnelListener(
        gs_listen_addr, gs_listen_port,
        on_pong=_on_pong,
        on_hello=_on_hello,
    )
    try:
        await tunnel_listener.start()
    except OSError as e:
        log.warning("tunnel_listener: bind %s:%d failed: %s; "
                    "HELLO/HELLO-ACK and ping/pong disabled",
                    gs_listen_addr, gs_listen_port, e)
        tunnel_listener = None
        timesync = None

    if debug_cfg.video_tap:
        def _on_frame(rec) -> None:
            if video_rtp_sink is not None:
                video_rtp_sink.write(rec)

        video_tap = VideoTap(
            args.video_tap_host, debug_cfg.video_tap_port,
            on_frame=_on_frame,
        )
        try:
            await video_tap.start()
        except OSError as e:
            log.warning("video_tap: bind %s:%d failed: %s; "
                        "video tap disabled",
                        args.video_tap_host, debug_cfg.video_tap_port, e)
            video_tap = None

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
            if rotator is not None:
                rotator.on_rx_event(ev)
            latest_rx[0] = ev
            signals = aggregator.consume(ev)
            decision = policy.tick(signals)
            sink.write(decision)
            if enabled and return_link is not None and wire_encoder is not None:
                packet = wire_encoder.encode(decision)
                return_link.send(packet)
                if decision.idr_request and idr_burster is not None:
                    idr_burster.trigger(decision)

    if args.replay is not None:
        client = ReplayClient(str(args.replay), on_event)
    else:
        client = StatsClient(endpoint, on_event)

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    pinger_task: asyncio.Task | None = None
    if (debug_cfg.ping_pong and tunnel_listener is not None
            and return_link is not None):
        async def _pinger() -> None:
            seq = 0
            interval_s = 1.0 / 5.0  # 5 Hz
            while not stop_event.is_set():
                seq = (seq + 1) & 0xFFFFFFFF
                t1 = time.monotonic_ns() // 1000
                pkt = wire.encode_ping(
                    wire.Ping(gs_seq=seq, gs_mono_us=t1)
                )
                return_link.send_ping(pkt)
                try:
                    await asyncio.wait_for(stop_event.wait(), timeout=interval_s)
                except asyncio.TimeoutError:
                    pass
        pinger_task = asyncio.create_task(_pinger(), name="dl-pinger")

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
        stop_event.set()
        if pinger_task is not None:
            try:
                await asyncio.wait_for(pinger_task, timeout=1.0)
            except asyncio.TimeoutError:
                pinger_task.cancel()
        sink.close()
        if latency_sink is not None:
            latency_sink.close()
        if video_tap is not None:
            video_tap.stop()
        if video_rtp_sink is not None:
            video_rtp_sink.close()
        if tunnel_listener is not None:
            tunnel_listener.stop()
        if return_link is not None:
            return_link.close()
        if mavlink_reader is not None:
            mavlink_reader.stop()
        if rotator is not None:
            rotator.close()

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
