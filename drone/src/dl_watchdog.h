/* dl_watchdog.h — failsafe 1, GS-link health watchdog.
 *
 * Pure state machine. The applier's main loop calls:
 *   - dl_watchdog_notify_decision() on every accepted decision packet.
 *   - dl_watchdog_tick(now_ms) on its periodic timer. Returns true iff
 *     the caller should push safe_defaults THIS tick (one-shot —
 *     subsequent ticks while silent return false).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint64_t last_decision_ms;
    uint32_t timeout_ms;
    bool     ever_seen;
    bool     tripped;     /* latched until next decision */
} dl_watchdog_t;

void dl_watchdog_init(dl_watchdog_t *w, uint32_t timeout_ms);

void dl_watchdog_notify_decision(dl_watchdog_t *w, uint64_t now_ms);

/* Returns true exactly once per "silent window" — when the link first
 * goes stale. Subsequent calls while still silent return false. */
bool dl_watchdog_tick(dl_watchdog_t *w, uint64_t now_ms);

bool dl_watchdog_is_tripped(const dl_watchdog_t *w);
