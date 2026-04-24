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

/* Apply bitrate / roi_qp / fps changes from `d` vs. `prev`. `prev`
 * may be NULL on first call. Updates `prev` on return. Returns 0 on
 * success (or no-op), -1 on HTTP failure. */
int dl_backend_enc_apply(dl_backend_enc_t *be,
                         const dl_decision_t *d,
                         dl_decision_t *prev);

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
