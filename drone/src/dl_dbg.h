/* dl_dbg.h — Phase 3 SD-card failure log.
 *
 * Append-only JSONL writer. Drone-side equivalent of the GS event/
 * verbose logs, but scoped to *failures* — successful applies don't
 * produce records, only their broken siblings.
 *
 * One record per call, line-buffered (no per-event fsync — too slow
 * on consumer SD). Size-capped; rotates to .1 / .2 once the active
 * file exceeds `max_bytes`. If the configured directory isn't
 * mounted/writable at startup, dl_dbg_init logs a single warning and
 * the rest of the API turns into no-ops — debug logging must never
 * stall the applier.
 *
 * Singleton, mirroring `dl_log`. Threading model: same as `dl_log` —
 * single-threaded applier loop, no locking.
 */
#pragma once

#include "dl_config.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DL_DBG_SEV_DEBUG = 0,
    DL_DBG_SEV_INFO  = 1,
    DL_DBG_SEV_WARN  = 2,
    DL_DBG_SEV_ERROR = 3,
} dl_dbg_sev_t;

/* Open the active dl-events.jsonl in append mode. The "enabled" flag
 * resolves through dl_config_dbg_log_resolved(); when off, dl_dbg
 * remains a no-op for the lifetime of the process. Safe to call
 * exactly once at startup. */
void dl_dbg_init(const dl_config_t *cfg);

/* Drain any buffered output and close the file. Safe to call on a
 * non-initialized dl_dbg. */
void dl_dbg_close(void);

/* True if dl_dbg is open and writing — for tests and the
 * dl_applier startup banner. */
bool dl_dbg_enabled(void);

/* Emit one JSONL line: `{"t":<mono_us>,"seq":N,"sev":"warn",
 * "reason":"ENC_RESPONSE_BAD","detail":<detail>}`. `detail_json`
 * must be a valid JSON value (object, string, number, etc.) — we
 * splice it in raw, no escaping. Pass "{}" if there's no detail.
 * Truncates `detail_json` defensively at 1024 chars to bound the
 * line size. */
void dl_dbg_emit(const char *reason,
                 dl_dbg_sev_t sev,
                 const char *detail_json);

/* Convenience: format a one-key object detail like {"errno":42}. */
void dl_dbg_emit_errno(const char *reason, int err);

/* Convenience: format {"http":<status>,"body":"<base64>"} for
 * encoder-response capture. `body` is escaped for inclusion as a
 * JSON string; `body_len` capped internally. */
void dl_dbg_emit_http(const char *reason,
                      dl_dbg_sev_t sev,
                      int http_status,
                      const char *body,
                      size_t body_len);
