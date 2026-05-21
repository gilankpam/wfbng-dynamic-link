/* dl_backend_enc.h — encoder HTTP backend (majestic / waybeam).
 *
 * API lives at `http://<encoder_host>:<encoder_port>`:
 *   GET /api/v1/set?video0.bitrate=<kbps>[&fpv.roiQp=<qp>][&video0.fps=<fps>]
 *   GET /request/idr
 *
 * See docs/encoder_api.md.
 */
#pragma once

#include "dl_config.h"
#include "dl_wire.h"
#include <stdint.h>

typedef struct dl_backend_enc dl_backend_enc_t;

dl_backend_enc_t *dl_backend_enc_open(const dl_config_t *cfg);
void dl_backend_enc_close(dl_backend_enc_t *be);

/* Apply bitrate / roi_qp / fps changes for `d`. roi_qp is computed
 * drone-side from d->bitrate_kbps via dl_compute_roi_qp; not part of
 * the wire as of v2. Returns 0 on success or no-op, -1 on HTTP failure. */
int dl_backend_enc_apply(dl_backend_enc_t *be, const dl_decision_t *d);

/* Send an IDR request, subject to min_idr_interval_ms throttle.
 * `now_ms` is a monotonic clock (CLOCK_MONOTONIC ms). Returns:
 *   0  = sent successfully
 *   1  = throttled (dropped, no request sent)
 *  -1  = HTTP failure
 */
int dl_backend_enc_request_idr(dl_backend_enc_t *be, uint64_t now_ms);

/* Push safe_defaults on watchdog trip (bitrate only — ROI/fps stay
 * where they are). */
int dl_backend_enc_apply_safe(dl_backend_enc_t *be, const dl_config_t *cfg);

/* Pure function. Maps bitrate → roi_qp using cfg->roi_qp_* knobs.
 * Exposed for unit tests; production callers go through
 * dl_backend_enc_apply.
 *
 * Formula: linear ramp from 0 at >= threshold down to floor at <= anchor,
 * quantized to multiples of step. Returns a signed delta in [floor, 0].
 */
int dl_compute_roi_qp(uint16_t bitrate_kbps, const dl_config_t *cfg);

/* Same as above but takes the four knobs directly, avoiding a
 * 1.7KB dl_config_t copy on hot paths. */
int dl_compute_roi_qp_raw(uint16_t bitrate_kbps,
                          uint16_t threshold_kbps,
                          uint16_t low_anchor_kbps,
                          int8_t   floor,
                          uint8_t  step);
