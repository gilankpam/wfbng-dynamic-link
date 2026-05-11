/* dl_config.h — drone.conf parser.
 *
 * Key=value plaintext, one per line, `#` or `;` starts a comment.
 * Matches the §6 sample in docs/dynamic-link-design.md with one new
 * key (`wlan_dev`) added for the iw backend.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DL_CONF_MAX_STR 128

typedef struct {
    /* Listen endpoint for decision packets. */
    char     listen_addr[DL_CONF_MAX_STR];
    uint16_t listen_port;

    /* wfb_tx control socket. */
    char     wfb_tx_ctrl_addr[DL_CONF_MAX_STR];
    uint16_t wfb_tx_ctrl_port;

    /* IDR throttle. */
    uint32_t min_idr_interval_ms;

    /* Inter-stage gap for direction-aware staggered apply. 0 disables
     * staggering (all backends fire in one shot, legacy behaviour). */
    uint32_t apply_stagger_ms;

    /* Pacing between sub-commands within a single apply phase: the
     * three CMD_SET_* sub-calls inside dl_backend_tx_apply (FEC,
     * DEPTH, RADIO) and the tx<->radio handoff inside the applier.
     * Both CMD_SET_FEC and CMD_SET_INTERLEAVE_DEPTH trigger
     * wfb-ng's refresh_session() which closes the open block,
     * flushes the interleaver and re-broadcasts SESSION packets;
     * back-to-back issues overlap those bursts and discard more
     * in-flight video than necessary. 0 disables (legacy). */
    uint32_t apply_sub_pace_ms;

    /* OSD sink (§4B). */
    bool     osd_enable;
    char     osd_msg_path[DL_CONF_MAX_STR];
    uint32_t osd_update_interval_ms;

    /* GS-link watchdog + safe_defaults. */
    uint32_t health_timeout_ms;
    uint8_t  safe_k;
    uint8_t  safe_n;
    uint8_t  safe_depth;
    uint8_t  safe_mcs;
    uint8_t  safe_bandwidth;
    int8_t   safe_tx_power_dBm;
    uint16_t safe_bitrate_kbps;

    /* Backends. */
    char     radio_backend[DL_CONF_MAX_STR];
    char     wlan_dev[DL_CONF_MAX_STR];
    char     encoder_kind[DL_CONF_MAX_STR];
    char     encoder_host[DL_CONF_MAX_STR];
    uint16_t encoder_port;

    /* MAVLink status channel (Phase 2). Drone → GS via wfb-ng's
     * mavlink stream (plain UDP loopback; wfb-ng's wfb_tx picks up
     * from 127.0.0.1:14560 on the drone). */
    bool     mavlink_enable;
    char     mavlink_addr[DL_CONF_MAX_STR];
    uint16_t mavlink_port;
    uint8_t  mavlink_sysid;
    uint8_t  mavlink_compid;

    /* ---- Debug suite (Phase 3) ---- */
    /* Master switch. False in production. */
    bool     debug_enable;
    /* Per-feature override. -1 = follow master, 0 = force off, 1 = force on. */
    int8_t   dbg_log_enable;

    /* GS-side endpoint for drone→GS tunnel traffic (PONG packets;
     * symmetric peer of `listen_addr`/`listen_port` on the GS). */
    char     gs_tunnel_addr[DL_CONF_MAX_STR];
    uint16_t gs_tunnel_port;

    /* SD-card failure log. */
    char     dbg_log_dir[DL_CONF_MAX_STR];
    uint32_t dbg_max_bytes;
    bool     dbg_fsync_each;
} dl_config_t;

/* Populate `cfg` with built-in defaults. */
void dl_config_defaults(dl_config_t *cfg);

/* Parse a conf file in place (updates fields of `cfg` that appear in
 * the file; unmentioned fields keep their incoming values).
 * Returns 0 on success, -1 on any I/O or parse error (details logged
 * via dl_log). */
int dl_config_load(const char *path, dl_config_t *cfg);

/* Resolve the dbg_log feature flag against the master switch.
 * `dbg_log_enable` is a tristate: -1 follows debug_enable, 0 forces
 * off, 1 forces on. */
bool dl_config_dbg_log_resolved(const dl_config_t *cfg);
