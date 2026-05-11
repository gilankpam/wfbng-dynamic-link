/* dl_config.c — parse /etc/dynamic-link/drone.conf. */
#include "dl_config.h"
#include "dl_log.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 256

void dl_config_defaults(dl_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    strncpy(cfg->listen_addr, "0.0.0.0", DL_CONF_MAX_STR - 1);
    cfg->listen_port = 5800;
    strncpy(cfg->wfb_tx_ctrl_addr, "127.0.0.1", DL_CONF_MAX_STR - 1);
    cfg->wfb_tx_ctrl_port = 8000;

    cfg->min_idr_interval_ms = 500;

    cfg->apply_stagger_ms = 50;
    cfg->apply_sub_pace_ms = 5;

    cfg->osd_enable = true;
    strncpy(cfg->osd_msg_path, "/tmp/MSPOSD.msg", DL_CONF_MAX_STR - 1);
    cfg->osd_update_interval_ms = 1000;

    cfg->health_timeout_ms = 10000;
    cfg->safe_k = 8;
    cfg->safe_n = 12;
    cfg->safe_depth = 1;
    cfg->safe_mcs = 1;
    cfg->safe_bandwidth = 20;
    cfg->safe_tx_power_dBm = 20;
    cfg->safe_bitrate_kbps = 2000;

    strncpy(cfg->radio_backend, "iw", DL_CONF_MAX_STR - 1);
    strncpy(cfg->wlan_dev, "wlan0", DL_CONF_MAX_STR - 1);
    strncpy(cfg->encoder_kind, "majestic", DL_CONF_MAX_STR - 1);
    strncpy(cfg->encoder_host, "127.0.0.1", DL_CONF_MAX_STR - 1);
    cfg->encoder_port = 80;

    cfg->mavlink_enable = true;
    strncpy(cfg->mavlink_addr, "127.0.0.1", DL_CONF_MAX_STR - 1);
    cfg->mavlink_port = 14560;
    cfg->mavlink_sysid = 250;   /* unassigned range; won't collide with FC sysid=1 */
    cfg->mavlink_compid = 191;  /* MAV_COMPONENT_ID_USER1 */

    cfg->debug_enable = false;
    cfg->dbg_log_enable = -1;   /* follow master */
    strncpy(cfg->gs_tunnel_addr, "10.5.0.1", DL_CONF_MAX_STR - 1);
    cfg->gs_tunnel_port = 5801;
    strncpy(cfg->dbg_log_dir, "/sdcard/dl-events", DL_CONF_MAX_STR - 1);
    cfg->dbg_max_bytes = 32 * 1024 * 1024;
    cfg->dbg_fsync_each = false;
}

static void trim(char *s) {
    char *end;
    while (*s && isspace((unsigned char)*s)) {
        memmove(s, s + 1, strlen(s));
    }
    end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) {
        *--end = '\0';
    }
}

static int parse_int(const char *val, long *out) {
    char *end;
    errno = 0;
    long v = strtol(val, &end, 10);
    if (errno != 0 || end == val || *end != '\0') return -1;
    *out = v;
    return 0;
}

static int parse_bool(const char *val, bool *out) {
    if (strcasecmp(val, "1") == 0 || strcasecmp(val, "true") == 0 ||
        strcasecmp(val, "yes") == 0 || strcasecmp(val, "on") == 0) {
        *out = true;
        return 0;
    }
    if (strcasecmp(val, "0") == 0 || strcasecmp(val, "false") == 0 ||
        strcasecmp(val, "no") == 0 || strcasecmp(val, "off") == 0) {
        *out = false;
        return 0;
    }
    return -1;
}

#define SET_STR(field) do { strncpy(cfg->field, val, DL_CONF_MAX_STR - 1); \
                            cfg->field[DL_CONF_MAX_STR - 1] = '\0'; } while(0)

#define SET_INT_RANGED(field, type, lo, hi) do {                      \
    long _v;                                                          \
    if (parse_int(val, &_v) != 0 || _v < (lo) || _v > (hi)) {         \
        dl_log_err("%s:%d: bad value for %s: %s", path, lineno, key, val); \
        rc = -1; continue;                                            \
    }                                                                 \
    cfg->field = (type)_v;                                            \
} while(0)

#define SET_BOOL(field) do {                                          \
    bool _v;                                                          \
    if (parse_bool(val, &_v) != 0) {                                  \
        dl_log_err("%s:%d: bad bool for %s: %s", path, lineno, key, val); \
        rc = -1; continue;                                            \
    }                                                                 \
    cfg->field = _v;                                                  \
} while(0)

int dl_config_load(const char *path, dl_config_t *cfg) {
    FILE *fd = fopen(path, "r");
    if (!fd) {
        dl_log_err("config: open %s: %s", path, strerror(errno));
        return -1;
    }

    char line[MAX_LINE];
    int lineno = 0;
    int rc = 0;
    while (fgets(line, sizeof(line), fd)) {
        lineno++;
        /* strip comments */
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';
        char *semi = strchr(line, ';');
        if (semi) *semi = '\0';

        trim(line);
        if (line[0] == '\0') continue;

        char *eq = strchr(line, '=');
        if (!eq) {
            dl_log_err("%s:%d: missing '=': %s", path, lineno, line);
            rc = -1;
            continue;
        }
        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        trim(key);
        trim(val);

        if      (strcmp(key, "listen_addr") == 0)        SET_STR(listen_addr);
        else if (strcmp(key, "listen_port") == 0)        SET_INT_RANGED(listen_port, uint16_t, 1, 65535);
        else if (strcmp(key, "wfb_tx_ctrl_addr") == 0)   SET_STR(wfb_tx_ctrl_addr);
        else if (strcmp(key, "wfb_tx_ctrl_port") == 0)   SET_INT_RANGED(wfb_tx_ctrl_port, uint16_t, 1, 65535);
        else if (strcmp(key, "video_k_min") == 0 ||
                 strcmp(key, "video_k_max") == 0 ||
                 strcmp(key, "video_n_max") == 0 ||
                 strcmp(key, "depth_max") == 0 ||
                 strcmp(key, "mcs_max") == 0 ||
                 strcmp(key, "tx_power_min_dBm") == 0 ||
                 strcmp(key, "tx_power_max_dBm") == 0) {
            dl_log_err("%s:%d: %s is no longer supported "
                       "(removed 2026-05-11); the drone applies whatever "
                       "the GS sends", path, lineno, key);
            rc = -1;
            continue;
        }
        else if (strcmp(key, "min_idr_interval_ms") == 0) SET_INT_RANGED(min_idr_interval_ms, uint32_t, 0, 60000);
        else if (strcmp(key, "apply_stagger_ms") == 0)   SET_INT_RANGED(apply_stagger_ms, uint32_t, 0, 500);
        else if (strcmp(key, "apply_sub_pace_ms") == 0)  SET_INT_RANGED(apply_sub_pace_ms, uint32_t, 0, 50);
        else if (strcmp(key, "osd_enable") == 0)         SET_BOOL(osd_enable);
        else if (strcmp(key, "osd_msg_path") == 0)       SET_STR(osd_msg_path);
        else if (strcmp(key, "osd_update_interval_ms") == 0) SET_INT_RANGED(osd_update_interval_ms, uint32_t, 100, 60000);
        else if (strcmp(key, "health_timeout_ms") == 0)  SET_INT_RANGED(health_timeout_ms, uint32_t, 500, 120000);
        else if (strcmp(key, "safe_k") == 0)             SET_INT_RANGED(safe_k, uint8_t, 1, 32);
        else if (strcmp(key, "safe_n") == 0)             SET_INT_RANGED(safe_n, uint8_t, 2, 255);
        else if (strcmp(key, "safe_depth") == 0)         SET_INT_RANGED(safe_depth, uint8_t, 1, 8);
        else if (strcmp(key, "safe_mcs") == 0)           SET_INT_RANGED(safe_mcs, uint8_t, 0, 7);
        else if (strcmp(key, "safe_bandwidth") == 0)     SET_INT_RANGED(safe_bandwidth, uint8_t, 20, 40);
        else if (strcmp(key, "safe_tx_power_dBm") == 0)  SET_INT_RANGED(safe_tx_power_dBm, int8_t, -10, 30);
        else if (strcmp(key, "safe_bitrate_kbps") == 0)  SET_INT_RANGED(safe_bitrate_kbps, uint16_t, 100, 65535);
        else if (strcmp(key, "radio_backend") == 0)      SET_STR(radio_backend);
        else if (strcmp(key, "wlan_dev") == 0)           SET_STR(wlan_dev);
        else if (strcmp(key, "encoder_kind") == 0)       SET_STR(encoder_kind);
        else if (strcmp(key, "encoder_host") == 0)       SET_STR(encoder_host);
        else if (strcmp(key, "encoder_port") == 0)       SET_INT_RANGED(encoder_port, uint16_t, 1, 65535);
        else if (strcmp(key, "mavlink_enable") == 0)     SET_BOOL(mavlink_enable);
        else if (strcmp(key, "mavlink_addr") == 0)       SET_STR(mavlink_addr);
        else if (strcmp(key, "mavlink_port") == 0)       SET_INT_RANGED(mavlink_port, uint16_t, 1, 65535);
        else if (strcmp(key, "mavlink_sysid") == 0)      SET_INT_RANGED(mavlink_sysid, uint8_t, 0, 255);
        else if (strcmp(key, "mavlink_compid") == 0)     SET_INT_RANGED(mavlink_compid, uint8_t, 0, 255);
        else if (strcmp(key, "debug_enable") == 0)       SET_BOOL(debug_enable);
        else if (strcmp(key, "dbg_log_enable") == 0)     SET_INT_RANGED(dbg_log_enable, int8_t, -1, 1);
        else if (strcmp(key, "gs_tunnel_addr") == 0)     SET_STR(gs_tunnel_addr);
        else if (strcmp(key, "gs_tunnel_port") == 0)     SET_INT_RANGED(gs_tunnel_port, uint16_t, 1, 65535);
        else if (strcmp(key, "dbg_log_dir") == 0)        SET_STR(dbg_log_dir);
        else if (strcmp(key, "dbg_max_bytes") == 0)      SET_INT_RANGED(dbg_max_bytes, uint32_t, 4096, 1 << 30);
        else if (strcmp(key, "dbg_fsync_each") == 0)     SET_BOOL(dbg_fsync_each);
        else {
            dl_log_warn("%s:%d: unknown key: %s", path, lineno, key);
        }
    }
    fclose(fd);
    return rc;
}

bool dl_config_dbg_log_resolved(const dl_config_t *cfg) {
    if (cfg->dbg_log_enable < 0) return cfg->debug_enable;
    return cfg->dbg_log_enable != 0;
}
