/* dl_ceiling.h — failsafe 4, per-airframe local bound enforcement.
 *
 * Design doc §5 failsafe 4. Runs on every accepted decision packet
 * BEFORE any backend dispatch. A rejection means NOTHING gets pushed.
 */
#pragma once

#include "dl_config.h"
#include "dl_wire.h"

typedef enum {
    DL_CEILING_OK = 0,
    DL_CEILING_K_OUT_OF_RANGE,
    DL_CEILING_N_TOO_LARGE,
    DL_CEILING_DEPTH_TOO_LARGE,
    DL_CEILING_MCS_TOO_HIGH,
    DL_CEILING_BANDWIDTH_BAD,
    DL_CEILING_TX_POWER_OUT_OF_RANGE,
    DL_CEILING_DEPTH_N_CONFLICT,  /* depth>1 && n>32 — wfb-ng TX reject */
} dl_ceiling_result_t;

const char *dl_ceiling_reason(dl_ceiling_result_t r);

/* Return DL_CEILING_OK or the specific failure code. Logs on failure. */
dl_ceiling_result_t dl_ceiling_check(const dl_decision_t *d,
                                     const dl_config_t *cfg);
