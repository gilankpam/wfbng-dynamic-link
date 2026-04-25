/* dl_dedup.h — sequence-number dedup with explicit reset.
 *
 * Tracks the last-seen sequence and rejects packets whose seq is
 * older or equal (signed-32-bit delta ≤ 0). Reset clears the
 * "ever seen" flag so the next packet seeds a new baseline — used
 * after the watchdog trips, so a GS restart whose seq numbers fall
 * below the prior `last_seq` recovers without operator action. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t last_seq;
    bool     ever;
} dl_dedup_t;

void dl_dedup_init(dl_dedup_t *d);

/* True → drop (duplicate or stale). False → fresh, accept. */
bool dl_dedup_check(dl_dedup_t *d, uint32_t seq);

/* Forget the last-seen seq. Next dl_dedup_check accepts unconditionally
 * and seeds a new baseline. */
void dl_dedup_reset(dl_dedup_t *d);
