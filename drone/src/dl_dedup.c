#include "dl_dedup.h"

void dl_dedup_init(dl_dedup_t *d) {
    d->last_seq = 0;
    d->ever     = false;
}

void dl_dedup_reset(dl_dedup_t *d) {
    d->ever = false;
}

bool dl_dedup_check(dl_dedup_t *d, uint32_t seq) {
    if (!d->ever) {
        d->last_seq = seq;
        d->ever     = true;
        return false;
    }
    int32_t delta = (int32_t)(seq - d->last_seq);
    if (delta <= 0) return true;
    d->last_seq = seq;
    return false;
}
