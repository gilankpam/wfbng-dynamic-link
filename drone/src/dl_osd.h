/* dl_osd.h — msposd status-line writer (§4B). */
#pragma once

#include "dl_config.h"
#include "dl_wire.h"
#include <stdint.h>

typedef struct dl_osd dl_osd_t;

dl_osd_t *dl_osd_open(const dl_config_t *cfg);
void dl_osd_close(dl_osd_t *o);

/* Write the steady-state line reflecting the last applied decision.
 * `rssi_dBm` is a hint; pass 0 if unknown. Safe to call on every tick
 * — file is rewritten atomically (tmpfile+rename). */
void dl_osd_write_status(dl_osd_t *o, const dl_decision_t *d, int rssi_dBm);

/* Write a transient event line on top of the status line. msposd
 * renders both if present. */
void dl_osd_write_event(dl_osd_t *o, const char *text);

/* Convenience events. */
void dl_osd_event_watchdog(dl_osd_t *o);
void dl_osd_event_reject(dl_osd_t *o, const char *reason);
