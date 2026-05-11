/* dl_hello.h — drone-side config-announce state machine.
 *
 * Responsibilities:
 *   - At boot: read MTU from /etc/wfb.yaml (wireless.mlink) and FPS
 *     from /etc/majestic.yaml (video0.fps). If either fails, refuse
 *     to enter ANNOUNCING — GS will stay in safe_defaults forever.
 *   - Generate a random generation_id (per-boot identifier).
 *   - ANNOUNCING: send DLHE on a 500 ms cadence (configurable) for
 *     the first 60 retries, then 5 s indefinitely. Transition to
 *     KEEPALIVE on first matching DLHA.
 *   - KEEPALIVE: send DLHE every 10 s. Transition back to ANNOUNCING
 *     if no DLHA arrives for 3 keepalive intervals.
 *
 * Pure logic: caller owns the UDP socket and the timerfd.
 *
 * Note: the state-machine type is `dl_hello_sm_t` (not `dl_hello_t`)
 * because `dl_hello_t` already names the wire packet struct in
 * dl_wire.h.
 */
#pragma once

#include "dl_config.h"
#include "dl_wire.h"

#include <stdint.h>

typedef enum {
    DL_HELLO_STATE_INIT       = 0,
    DL_HELLO_STATE_ANNOUNCING = 1,
    DL_HELLO_STATE_KEEPALIVE  = 2,
    DL_HELLO_STATE_DISABLED   = 3,
} dl_hello_state_t;

typedef struct {
    dl_hello_state_t state;
    uint32_t generation_id;
    uint16_t mtu_bytes;
    uint16_t fps;
    uint32_t announce_count;
    uint32_t announces_without_ack;
    uint32_t keepalives_without_ack;
    const dl_config_t *cfg;
} dl_hello_sm_t;

int dl_hello_init(dl_hello_sm_t *h, const dl_config_t *cfg);

size_t dl_hello_build_announce(dl_hello_sm_t *h, uint8_t *buf, size_t buflen);

uint32_t dl_hello_next_delay_ms(const dl_hello_sm_t *h);

int dl_hello_on_ack(dl_hello_sm_t *h, const dl_hello_ack_t *ack);

int dl_hello_on_keepalive_tick(dl_hello_sm_t *h);
