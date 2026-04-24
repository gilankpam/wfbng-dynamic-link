/* dl_watchdog.c */
#include "dl_watchdog.h"

void dl_watchdog_init(dl_watchdog_t *w, uint32_t timeout_ms) {
    w->last_decision_ms = 0;
    w->timeout_ms = timeout_ms;
    w->ever_seen = false;
    w->tripped = false;
}

void dl_watchdog_notify_decision(dl_watchdog_t *w, uint64_t now_ms) {
    w->last_decision_ms = now_ms;
    w->ever_seen = true;
    w->tripped = false;
}

bool dl_watchdog_tick(dl_watchdog_t *w, uint64_t now_ms) {
    if (!w->ever_seen) return false;  /* no reference point yet */
    if (w->tripped) return false;
    if (now_ms >= w->last_decision_ms &&
        (now_ms - w->last_decision_ms) >= w->timeout_ms) {
        w->tripped = true;
        return true;
    }
    return false;
}

bool dl_watchdog_is_tripped(const dl_watchdog_t *w) {
    return w->tripped;
}
