/* dl_backend_radio.c — shells out to iw for TX-power changes.
 *
 * Implementation note: we use posix_spawn rather than fork+exec to
 * avoid touching the applier's fd table (no accidental FD_CLOEXEC
 * oversight, no forked heap state). `iw dev <wlan> set txpower fixed <mBm>`
 * returns 0 on success; we wait for it and surface non-zero as -1.
 */
#include "dl_backend_radio.h"
#include "dl_log.h"

#include <errno.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

struct dl_backend_radio {
    char     wlan[DL_CONF_MAX_STR];
    int8_t   current_dBm;
    bool     initialized;
};

static int run_iw(const char *wlan, int8_t dBm) {
    char mBm_str[16];
    snprintf(mBm_str, sizeof(mBm_str), "%d", (int)dBm * 100);

    char *const argv[] = {
        (char *)"iw",
        (char *)"dev",
        (char *)wlan,
        (char *)"set",
        (char *)"txpower",
        (char *)"fixed",
        mBm_str,
        NULL,
    };

    pid_t pid;
    int rc = posix_spawnp(&pid, "iw", NULL, NULL, argv, environ);
    if (rc != 0) {
        dl_log_warn("radio: posix_spawnp iw: %s", strerror(rc));
        return -1;
    }
    int status;
    if (waitpid(pid, &status, 0) < 0) {
        dl_log_warn("radio: waitpid: %s", strerror(errno));
        return -1;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        dl_log_warn("radio: iw exited with status %d (txpower=%d dBm)",
                    WIFEXITED(status) ? WEXITSTATUS(status) : -1,
                    (int)dBm);
        return -1;
    }
    dl_log_info("radio: txpower %d dBm (iw dev %s)", (int)dBm, wlan);
    return 0;
}

dl_backend_radio_t *dl_backend_radio_open(const dl_config_t *cfg) {
    dl_backend_radio_t *br = calloc(1, sizeof(*br));
    if (!br) return NULL;
    snprintf(br->wlan, sizeof(br->wlan), "%s", cfg->wlan_dev);
    return br;
}

void dl_backend_radio_close(dl_backend_radio_t *br) {
    free(br);
}

int dl_backend_radio_apply(dl_backend_radio_t *br,
                           const dl_decision_t *d,
                           dl_decision_t *prev) {
    if (!br) return -1;
    bool first = (prev == NULL) || prev->magic != DL_WIRE_MAGIC;
    if (!first && prev->tx_power_dBm == d->tx_power_dBm) {
        return 0;
    }
    int rc = run_iw(br->wlan, d->tx_power_dBm);
    if (rc == 0) br->current_dBm = d->tx_power_dBm;
    if (prev) *prev = *d;
    return rc;
}

int dl_backend_radio_apply_safe(dl_backend_radio_t *br,
                                const dl_config_t *cfg) {
    if (!br) return -1;
    int rc = run_iw(br->wlan, cfg->safe_tx_power_dBm);
    if (rc == 0) br->current_dBm = cfg->safe_tx_power_dBm;
    return rc;
}
