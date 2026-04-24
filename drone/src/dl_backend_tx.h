/* dl_backend_tx.h — dispatch wfb-ng CMD_SET_* commands. */
#pragma once

#include "dl_config.h"
#include "dl_wire.h"

typedef struct dl_backend_tx dl_backend_tx_t;

/* Opens a UDP socket connected to cfg->wfb_tx_ctrl_addr:port. Returns
 * NULL on failure. */
dl_backend_tx_t *dl_backend_tx_open(const dl_config_t *cfg);

void dl_backend_tx_close(dl_backend_tx_t *bt);

/* Diff-based apply: only emits commands for knobs that differ from
 * `prev`. `prev` may be NULL on first call (emits everything). The
 * updated state is written back to `prev` on success.
 *
 * `short_gi` is always pinned to false (§1 design policy).
 *
 * Returns 0 if every emitted command succeeded, -1 if any failed (but
 * the rest are still attempted — partial application).
 */
int dl_backend_tx_apply(dl_backend_tx_t *bt,
                        const dl_decision_t *d,
                        dl_decision_t *prev);

/* Push safe_defaults — the watchdog fallback. Always emits all three
 * commands unconditionally. Returns 0 on all-success, -1 otherwise. */
int dl_backend_tx_apply_safe(dl_backend_tx_t *bt,
                             const dl_config_t *cfg);
