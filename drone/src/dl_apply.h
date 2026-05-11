/* dl_apply.h — direction helper for staggered profile apply.
 *
 * Header-only on purpose: the function is pure and trivial enough
 * that a separate translation unit would just be ceremony, and
 * keeping it inline lets the test binary link without pulling in
 * dl_applier.c (which has main()).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DL_APPLY_DIR_EQUAL = 0,   /* single-shot, no stagger */
    DL_APPLY_DIR_UP    = 1,   /* tx+radio first, encoder after gap */
    DL_APPLY_DIR_DOWN  = -1,  /* encoder first, tx+radio after gap */
} dl_apply_dir_t;

static inline dl_apply_dir_t
dl_apply_direction(uint16_t prev_bitrate_kbps,
                   uint16_t new_bitrate_kbps,
                   bool first_decision) {
    if (first_decision) return DL_APPLY_DIR_EQUAL;
    if (new_bitrate_kbps > prev_bitrate_kbps) return DL_APPLY_DIR_UP;
    if (new_bitrate_kbps < prev_bitrate_kbps) return DL_APPLY_DIR_DOWN;
    return DL_APPLY_DIR_EQUAL;
}
