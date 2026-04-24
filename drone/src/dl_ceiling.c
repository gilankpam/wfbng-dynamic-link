/* dl_ceiling.c */
#include "dl_ceiling.h"
#include "dl_log.h"

const char *dl_ceiling_reason(dl_ceiling_result_t r) {
    switch (r) {
        case DL_CEILING_OK:                   return "ok";
        case DL_CEILING_K_OUT_OF_RANGE:       return "k_out_of_range";
        case DL_CEILING_N_TOO_LARGE:          return "n_too_large";
        case DL_CEILING_DEPTH_TOO_LARGE:      return "depth_too_large";
        case DL_CEILING_MCS_TOO_HIGH:         return "mcs_too_high";
        case DL_CEILING_BANDWIDTH_BAD:        return "bandwidth_bad";
        case DL_CEILING_TX_POWER_OUT_OF_RANGE: return "tx_power_out_of_range";
        case DL_CEILING_DEPTH_N_CONFLICT:     return "depth_n_conflict";
    }
    return "unknown";
}

dl_ceiling_result_t dl_ceiling_check(const dl_decision_t *d,
                                     const dl_config_t *cfg) {
    if (d->k < cfg->video_k_min || d->k > cfg->video_k_max) {
        dl_log_warn("ceiling: k=%u outside [%u,%u]",
                    d->k, cfg->video_k_min, cfg->video_k_max);
        return DL_CEILING_K_OUT_OF_RANGE;
    }
    if (d->n > cfg->video_n_max) {
        dl_log_warn("ceiling: n=%u > video_n_max=%u",
                    d->n, cfg->video_n_max);
        return DL_CEILING_N_TOO_LARGE;
    }
    if (d->depth > cfg->depth_max) {
        dl_log_warn("ceiling: depth=%u > depth_max=%u",
                    d->depth, cfg->depth_max);
        return DL_CEILING_DEPTH_TOO_LARGE;
    }
    if (d->mcs > cfg->mcs_max) {
        dl_log_warn("ceiling: mcs=%u > mcs_max=%u", d->mcs, cfg->mcs_max);
        return DL_CEILING_MCS_TOO_HIGH;
    }
    if (d->bandwidth != 20 && d->bandwidth != 40) {
        dl_log_warn("ceiling: bandwidth=%u not in {20, 40}", d->bandwidth);
        return DL_CEILING_BANDWIDTH_BAD;
    }
    if (d->tx_power_dBm < cfg->tx_power_min_dBm ||
        d->tx_power_dBm > cfg->tx_power_max_dBm) {
        dl_log_warn("ceiling: tx_power_dBm=%d outside [%d,%d]",
                    d->tx_power_dBm,
                    cfg->tx_power_min_dBm, cfg->tx_power_max_dBm);
        return DL_CEILING_TX_POWER_OUT_OF_RANGE;
    }
    /* wfb-ng rejects depth > 1 with n > 32. Enforce locally so we don't
     * feed a known-bad combo to the TX and have it refuse silently. */
    if (d->depth > 1 && d->n > 32) {
        dl_log_warn("ceiling: depth=%u n=%u (wfb-ng rejects depth>1 && n>32)",
                    d->depth, d->n);
        return DL_CEILING_DEPTH_N_CONFLICT;
    }
    return DL_CEILING_OK;
}
