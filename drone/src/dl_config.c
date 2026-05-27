/* dl_config.c — parse /etc/dynamic-link/drone.conf. */
#include "dl_config.h"
#include "dl_log.h"

#include <ctype.h>
#include <errno.h>
#include <stddef.h>
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
    cfg->idr_listen_port = 11223;
    strncpy(cfg->idr_listen_addr, "0.0.0.0", DL_CONF_MAX_STR - 1);

    cfg->apply_stagger_ms = 50;
    cfg->apply_sub_pace_ms = 5;

    cfg->osd_enable = true;
    strncpy(cfg->osd_msg_path, "/tmp/MSPOSD.msg", DL_CONF_MAX_STR - 1);
    cfg->osd_update_interval_ms = 1000;
    cfg->osd_debug_latency = false;

    cfg->health_timeout_ms = 10000;
    cfg->safe_k = 8;
    cfg->safe_n = 12;
    cfg->safe_depth = 1;
    cfg->safe_mcs = 1;
    cfg->safe_bandwidth = 20;
    cfg->safe_tx_power_dBm = 20;
    cfg->safe_bitrate_kbps = 2000;
    cfg->interleaving_supported = true;

    strncpy(cfg->wlan_dev, "wlan0", DL_CONF_MAX_STR - 1);
    strncpy(cfg->encoder_kind, "majestic", DL_CONF_MAX_STR - 1);
    strncpy(cfg->encoder_host, "127.0.0.1", DL_CONF_MAX_STR - 1);
    cfg->encoder_port = 80;

    cfg->roi_qp_threshold_kbps  = 6000;
    cfg->roi_qp_low_anchor_kbps = 2000;
    cfg->roi_qp_floor           = -24;
    cfg->roi_qp_step            = 3;

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

    cfg->hello_announce_initial_ms = 500;
    cfg->hello_announce_steady_ms = 5000;
    cfg->hello_keepalive_ms = 10000;
    cfg->hello_announce_initial_count = 60;
    /* hello_mtu_bytes / hello_fps are required-by-default values
     * left at 0 by the memset above — operator must set
     * hello_mtu_bytes (and optionally hello_fps to bypass the encoder
     * file lookup). */
    strncpy(cfg->hello_majestic_yaml_path, "/etc/majestic.yaml", DL_CONF_MAX_STR - 1);
    strncpy(cfg->hello_waybeam_json_path, "/etc/waybeam.json", DL_CONF_MAX_STR - 1);
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

/* Parse a base-10 long out of `s`; reject empty, trailing garbage, or
 * out-of-range [lo..hi]. Returns 0 on success, -1 on failure. */
static int dl_parse_long_ranged(const char *s, long lo, long hi, long *out) {
    char *end;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0') return -1;
    if (v < lo || v > hi) return -1;
    *out = v;
    return 0;
}

static int dl_parse_bool(const char *val, bool *out) {
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

#define SET_INT_RANGED(field, type, lo, hi) do {                       \
    long _v;                                                           \
    if (dl_parse_long_ranged(val, (long)(lo), (long)(hi), &_v) != 0) { \
        dl_log_err("%s:%d: bad value for %s: %s",                      \
                   path, lineno, key, val);                            \
        rc = -1; continue;                                             \
    }                                                                  \
    cfg->field = (type)_v;                                             \
} while(0)

#define SET_BOOL(field) do {                                           \
    bool _v;                                                           \
    if (dl_parse_bool(val, &_v) != 0) {                                \
        dl_log_err("%s:%d: bad bool for %s: %s",                       \
                   path, lineno, key, val);                            \
        rc = -1; continue;                                             \
    }                                                                  \
    cfg->field = _v;                                                   \
} while(0)

#define F_INT(name_, type_, lo_, hi_) \
    { #name_, offsetof(dl_config_t, name_), type_, (long)(lo_), (long)(hi_) }
#define F_BOOL(name_) { #name_, offsetof(dl_config_t, name_) }
#define F_STR(name_)  { #name_, offsetof(dl_config_t, name_) }

static const dl_int_field_t DL_INT_FIELDS[] = {
    F_INT(listen_port,                   DL_F_U16, 1,      65535),
    F_INT(wfb_tx_ctrl_port,              DL_F_U16, 1,      65535),
    F_INT(min_idr_interval_ms,           DL_F_U32, 0,      60000),
    F_INT(idr_listen_port,               DL_F_U16, 0,      65535),
    F_INT(apply_stagger_ms,              DL_F_U32, 0,      500),
    F_INT(apply_sub_pace_ms,             DL_F_U32, 0,      50),
    F_INT(osd_update_interval_ms,        DL_F_U32, 100,    60000),
    F_INT(health_timeout_ms,             DL_F_U32, 500,    120000),
    F_INT(safe_k,                        DL_F_U8,  1,      32),
    F_INT(safe_n,                        DL_F_U8,  2,      255),
    F_INT(safe_depth,                    DL_F_U8,  1,      8),
    F_INT(safe_mcs,                      DL_F_U8,  0,      7),
    F_INT(safe_bandwidth,                DL_F_U8,  20,     40),
    F_INT(safe_tx_power_dBm,             DL_F_I8,  -10,    30),
    F_INT(safe_bitrate_kbps,             DL_F_U16, 100,    65535),
    F_INT(encoder_port,                  DL_F_U16, 1,      65535),
    F_INT(roi_qp_threshold_kbps,         DL_F_U16, 100,    65535),
    F_INT(roi_qp_low_anchor_kbps,        DL_F_U16, 100,    65535),
    F_INT(roi_qp_floor,                  DL_F_I8,  -30,    0),
    F_INT(roi_qp_step,                   DL_F_U8,  1,      10),
    F_INT(mavlink_port,                  DL_F_U16, 1,      65535),
    F_INT(mavlink_sysid,                 DL_F_U8,  0,      255),
    F_INT(mavlink_compid,                DL_F_U8,  0,      255),
    F_INT(gs_tunnel_port,                DL_F_U16, 1,      65535),
    F_INT(hello_announce_initial_ms,     DL_F_U32, 1,      60000),
    F_INT(hello_announce_steady_ms,      DL_F_U32, 1,      300000),
    F_INT(hello_keepalive_ms,            DL_F_U32, 1,      300000),
    F_INT(hello_announce_initial_count,  DL_F_U32, 0,      100000),
    F_INT(hello_mtu_bytes,               DL_F_U16, 0,      65535),
    F_INT(hello_fps,                     DL_F_U16, 0,      65535),
};

static const dl_bool_field_t DL_BOOL_FIELDS[] = {
    F_BOOL(osd_enable),
    F_BOOL(osd_debug_latency),
    F_BOOL(interleaving_supported),
    F_BOOL(mavlink_enable),
};

static const dl_str_field_t DL_STR_FIELDS[] = {
    F_STR(listen_addr),
    F_STR(wfb_tx_ctrl_addr),
    F_STR(idr_listen_addr),
    F_STR(osd_msg_path),
    F_STR(wlan_dev),
    F_STR(encoder_kind),
    F_STR(encoder_host),
    F_STR(mavlink_addr),
    F_STR(gs_tunnel_addr),
    F_STR(hello_majestic_yaml_path),
    F_STR(hello_waybeam_json_path),
};

/* Copy `src` into `dst` (size `dstlen`) replacing '-' with '_'.
 * Truncates safely. Used to convert CLI flag names back to conf
 * keys for table lookup. */
static void dl_kebab_to_snake(char *dst, size_t dstlen, const char *src) {
    size_t i = 0;
    for (; src[i] && i + 1 < dstlen; i++)
        dst[i] = (src[i] == '-') ? '_' : src[i];
    dst[i] = '\0';
}

const dl_int_field_t *dl_config_int_fields(size_t *n_out) {
    if (n_out) *n_out = sizeof(DL_INT_FIELDS) / sizeof(DL_INT_FIELDS[0]);
    return DL_INT_FIELDS;
}

const dl_bool_field_t *dl_config_bool_fields(size_t *n_out) {
    if (n_out) *n_out = sizeof(DL_BOOL_FIELDS) / sizeof(DL_BOOL_FIELDS[0]);
    return DL_BOOL_FIELDS;
}

const dl_str_field_t *dl_config_str_fields(size_t *n_out) {
    if (n_out) *n_out = sizeof(DL_STR_FIELDS) / sizeof(DL_STR_FIELDS[0]);
    return DL_STR_FIELDS;
}

static void write_int_field(dl_config_t *cfg, const dl_int_field_t *f, long v) {
    void *p = (char *)cfg + f->offset;
    switch (f->type) {
    case DL_F_U8:  *(uint8_t  *)p = (uint8_t )v; break;
    case DL_F_I8:  *(int8_t   *)p = (int8_t  )v; break;
    case DL_F_U16: *(uint16_t *)p = (uint16_t)v; break;
    case DL_F_U32: *(uint32_t *)p = (uint32_t)v; break;
    }
}

int dl_config_set_int_by_name(dl_config_t *cfg, const char *name, const char *val) {
    char norm[64];
    dl_kebab_to_snake(norm, sizeof(norm), name);
    size_t n = 0;
    const dl_int_field_t *t = dl_config_int_fields(&n);
    for (size_t i = 0; i < n; i++) {
        if (strcmp(t[i].name, norm) != 0) continue;
        long v;
        if (dl_parse_long_ranged(val, t[i].lo, t[i].hi, &v) != 0) return -1;
        write_int_field(cfg, &t[i], v);
        return 0;
    }
    return -1;
}

int dl_config_set_bool_by_name(dl_config_t *cfg, const char *name, bool val) {
    char norm[64];
    dl_kebab_to_snake(norm, sizeof(norm), name);
    size_t n = 0;
    const dl_bool_field_t *t = dl_config_bool_fields(&n);
    for (size_t i = 0; i < n; i++) {
        if (strcmp(t[i].name, norm) != 0) continue;
        *(bool *)((char *)cfg + t[i].offset) = val;
        return 0;
    }
    return -1;
}

int dl_config_set_str_by_name(dl_config_t *cfg, const char *name, const char *val) {
    char norm[64];
    dl_kebab_to_snake(norm, sizeof(norm), name);
    size_t n = 0;
    const dl_str_field_t *t = dl_config_str_fields(&n);
    for (size_t i = 0; i < n; i++) {
        if (strcmp(t[i].name, norm) != 0) continue;
        if (strlen(val) >= DL_CONF_MAX_STR) return -1;
        char *dst = (char *)cfg + t[i].offset;
        snprintf(dst, DL_CONF_MAX_STR, "%s", val);
        return 0;
    }
    return -1;
}

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

        /* Removed-key error: keys explicitly retired 2026-05-11 still
         * exist in old configs; surface them loudly. */
        if (strcmp(key, "video_k_min")       == 0 ||
            strcmp(key, "video_k_max")       == 0 ||
            strcmp(key, "video_n_max")       == 0 ||
            strcmp(key, "depth_max")         == 0 ||
            strcmp(key, "mcs_max")           == 0 ||
            strcmp(key, "tx_power_min_dBm")  == 0 ||
            strcmp(key, "tx_power_max_dBm")  == 0) {
            dl_log_err("%s:%d: %s is no longer supported "
                       "(removed 2026-05-11); the drone applies whatever "
                       "the GS sends", path, lineno, key);
            rc = -1;
            continue;
        }

        /* Table-driven int dispatch. */
        {
            size_t n = 0;
            const dl_int_field_t *t = dl_config_int_fields(&n);
            int matched = 0;
            for (size_t i = 0; i < n; i++) {
                if (strcmp(t[i].name, key) != 0) continue;
                long v;
                if (dl_parse_long_ranged(val, t[i].lo, t[i].hi, &v) != 0) {
                    dl_log_err("%s:%d: bad value for %s: %s",
                               path, lineno, key, val);
                    rc = -1;
                } else {
                    write_int_field(cfg, &t[i], v);
                }
                matched = 1; break;
            }
            if (matched) continue;
        }

        /* Table-driven bool dispatch. */
        {
            size_t n = 0;
            const dl_bool_field_t *t = dl_config_bool_fields(&n);
            int matched = 0;
            for (size_t i = 0; i < n; i++) {
                if (strcmp(t[i].name, key) != 0) continue;
                bool v;
                if (dl_parse_bool(val, &v) != 0) {
                    dl_log_err("%s:%d: bad bool for %s: %s",
                               path, lineno, key, val);
                    rc = -1;
                } else {
                    *(bool *)((char *)cfg + t[i].offset) = v;
                }
                matched = 1; break;
            }
            if (matched) continue;
        }

        /* Table-driven str dispatch. */
        {
            size_t n = 0;
            const dl_str_field_t *t = dl_config_str_fields(&n);
            int matched = 0;
            for (size_t i = 0; i < n; i++) {
                if (strcmp(t[i].name, key) != 0) continue;
                if (strlen(val) >= DL_CONF_MAX_STR) {
                    dl_log_err("%s:%d: value too long for %s",
                               path, lineno, key);
                    rc = -1;
                } else {
                    char *dst = (char *)cfg + t[i].offset;
                    snprintf(dst, DL_CONF_MAX_STR, "%s", val);
                }
                matched = 1; break;
            }
            if (matched) continue;
        }

        /* Phase-3 debug-suite keys (not in CLI scope; hand-written). */
        if      (strcmp(key, "debug_enable")   == 0) SET_BOOL(debug_enable);
        else if (strcmp(key, "dbg_log_enable") == 0) SET_INT_RANGED(dbg_log_enable, int8_t, -1, 1);
        else if (strcmp(key, "dbg_log_dir")    == 0) SET_STR(dbg_log_dir);
        else if (strcmp(key, "dbg_max_bytes")  == 0) SET_INT_RANGED(dbg_max_bytes, uint32_t, 4096, 1 << 30);
        else if (strcmp(key, "dbg_fsync_each") == 0) SET_BOOL(dbg_fsync_each);
        else {
            dl_log_warn("%s:%d: unknown key: %s", path, lineno, key);
        }
    }
    if (dl_config_validate(cfg) != 0) rc = -1;
    fclose(fd);
    return rc;
}

int dl_config_validate(const dl_config_t *cfg) {
    int rc = 0;
    if (cfg->roi_qp_threshold_kbps <= cfg->roi_qp_low_anchor_kbps) {
        dl_log_err("config: roi_qp_threshold_kbps (%u) must be > "
                   "roi_qp_low_anchor_kbps (%u)",
                   (unsigned)cfg->roi_qp_threshold_kbps,
                   (unsigned)cfg->roi_qp_low_anchor_kbps);
        rc = -1;
    }
    return rc;
}

bool dl_config_dbg_log_resolved(const dl_config_t *cfg) {
    if (cfg->dbg_log_enable < 0) return cfg->debug_enable;
    return cfg->dbg_log_enable != 0;
}
