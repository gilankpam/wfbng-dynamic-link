/* test_osd.c — status-line rendering, incl. IDR counter. */
#include "dl_osd.h"
#include "dl_config.h"
#include "dl_wire.h"
#include "test_main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void read_all(const char *path, char *out, size_t outlen) {
    FILE *f = fopen(path, "r");
    if (!f) { out[0] = '\0'; return; }
    size_t n = fread(out, 1, outlen - 1, f);
    out[n] = '\0';
    fclose(f);
}

DL_TEST(test_osd_status_includes_idr_counter) {
    char path[64];
    snprintf(path, sizeof(path), "/tmp/dl-osd-test-%d.msg", (int)getpid());

    dl_config_t cfg = {0};
    cfg.osd_enable = true;
    cfg.osd_debug_latency = false;
    snprintf(cfg.osd_msg_path, sizeof(cfg.osd_msg_path), "%s", path);

    dl_osd_t *o = dl_osd_open(&cfg);
    DL_ASSERT(o != NULL);

    dl_decision_t d = {
        .mcs = 5, .bitrate_kbps = 12000,
        .k = 8, .n = 14, .depth = 2,
        .tx_power_dBm = 18,
    };

    /* Zero counter is rendered as I0. */
    dl_osd_write_status(o, &d, -50);
    char buf[256];
    read_all(path, buf, sizeof(buf));
    DL_ASSERT(strstr(buf, " I0 |") != NULL);

    /* Three bumps -> I3. */
    dl_osd_bump_idr(o);
    dl_osd_bump_idr(o);
    dl_osd_bump_idr(o);
    dl_osd_write_status(o, &d, -50);
    read_all(path, buf, sizeof(buf));
    DL_ASSERT(strstr(buf, " I3 |") != NULL);

    dl_osd_close(o);
    unlink(path);
}
