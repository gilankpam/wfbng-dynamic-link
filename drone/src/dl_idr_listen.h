/* dl_idr_listen.h — UDP listener for PixelPilot IDR-token bursts.
 *
 * PixelPilot_rk sends short UDP datagrams (typically 6 bytes, a
 * random 3-byte ASCII token + newline) to the drone on port 11223
 * when it detects an RTP sequence gap or a decode stall. Anything
 * arriving on this socket is treated as an IDR request; the caller
 * is responsible for throttling the encoder API.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct dl_idr_listen dl_idr_listen_t;

/* Open a non-blocking AF_INET SOCK_DGRAM socket bound to
 * `bind_addr:port`. Returns NULL on error (logged) or if port == 0
 * (disabled). `bind_addr` of NULL or "" binds to 0.0.0.0. */
dl_idr_listen_t *dl_idr_listen_open(const char *bind_addr, uint16_t port);

/* Bound fd for poll(2). Returns -1 if l == NULL. */
int dl_idr_listen_fd(const dl_idr_listen_t *l);

/* Drain all queued datagrams (recvfrom until EAGAIN). Returns the
 * count consumed. Used for logging only — the caller decides whether
 * to actuate IDR based on count > 0. NULL-safe (returns 0). */
size_t dl_idr_listen_drain(dl_idr_listen_t *l);

/* Close and free. NULL-safe. */
void dl_idr_listen_close(dl_idr_listen_t *l);
