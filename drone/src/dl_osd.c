/* dl_osd.c */
#include "dl_osd.h"
#include "dl_log.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

struct dl_osd {
    char path[DL_CONF_MAX_STR];
    bool enabled;
    /* Latest status + transient event lines. Written together so
     * msposd can render both. */
    char status_line[128];
    char event_line[128];
};

dl_osd_t *dl_osd_open(const dl_config_t *cfg) {
    dl_osd_t *o = calloc(1, sizeof(*o));
    if (!o) return NULL;
    snprintf(o->path, sizeof(o->path), "%s", cfg->osd_msg_path);
    o->enabled = cfg->osd_enable;
    return o;
}

void dl_osd_close(dl_osd_t *o) {
    free(o);
}

static void flush(dl_osd_t *o) {
    if (!o->enabled) return;

    /* Write atomically: path.tmp → rename(path). Avoids msposd reading
     * a half-written buffer. */
    char tmp[DL_CONF_MAX_STR + 8];
    snprintf(tmp, sizeof(tmp), "%s.tmp", o->path);
    FILE *fd = fopen(tmp, "w");
    if (!fd) {
        dl_log_debug("osd: fopen %s: %s", tmp, strerror(errno));
        return;
    }
    if (o->status_line[0]) fprintf(fd, "%s\n", o->status_line);
    if (o->event_line[0])  fprintf(fd, "%s\n", o->event_line);
    fflush(fd);
    fclose(fd);
    if (rename(tmp, o->path) < 0) {
        dl_log_debug("osd: rename %s -> %s: %s", tmp, o->path, strerror(errno));
        unlink(tmp);
    }
}

/* msposd directive prefix: line 50 (near bottom of frame), font
 * size 30. Without these directives msposd falls back to its boot-
 * default `&F38 &L43` style (huge font, middle of frame), which
 * causes long lines to marquee-scroll left-to-right. Same prefix
 * the reference alink_drone uses. */
#define DL_OSD_PREFIX "&L50&F30 "

void dl_osd_write_status(dl_osd_t *o, const dl_decision_t *d, int rssi_dBm) {
    if (!o) return;
    /* &T/&W/&B/&C are msposd placeholders (board temp, wifi-module temp,
     * video bitrate+fps, cpu%); msposd substitutes at render time. */
    snprintf(o->status_line, sizeof(o->status_line),
             DL_OSD_PREFIX
             "MCS%u %uM (%u,%u)d%u TX%d R%d | &B T&T W&W CPU&C",
             d->mcs,
             (unsigned)((d->bitrate_kbps + 500) / 1000),
             d->k, d->n, d->depth,
             (int)d->tx_power_dBm,
             rssi_dBm);
    /* Fresh status = the link recovered (or never tripped). Clear any
     * stale event line so a past WATCHDOG/REJECT toast doesn't sit on
     * the OSD forever — msposd will keep rendering the last bytes we
     * wrote, so we have to actively unset. */
    o->event_line[0] = '\0';
    flush(o);
}

void dl_osd_write_event(dl_osd_t *o, const char *text) {
    if (!o) return;
    snprintf(o->event_line, sizeof(o->event_line),
             DL_OSD_PREFIX "%s", text);
    flush(o);
}

void dl_osd_event_watchdog(dl_osd_t *o) {
    dl_osd_write_event(o, "WATCHDOG safe_defaults");
}

void dl_osd_event_reject(dl_osd_t *o, const char *reason) {
    char buf[64];
    snprintf(buf, sizeof(buf), "REJECT %s", reason);
    dl_osd_write_event(o, buf);
}
