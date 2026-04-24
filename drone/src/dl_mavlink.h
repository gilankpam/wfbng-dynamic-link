/* dl_mavlink.h — minimal MAVLink v1 STATUSTEXT emitter.
 *
 * Phase 2 drone → GS status channel. Piggybacks on wfb-ng's existing
 * `mavlink` stream (plain UDP loopback pair): the drone writes
 * MAVLink bytes to cfg.mavlink_addr:mavlink_port (default
 * 127.0.0.1:14560), wfb-ng's wfb_tx picks them up and sends them over
 * the radio; the GS's wfb_rx delivers them to 127.0.0.1:14550 where
 * gs/dynamic_link/mavlink_status.py reads them.
 *
 * MAVLink v1 STATUSTEXT (msgid 253) layout:
 *
 *   [0xFE][LEN=51][SEQ][SYSID][COMPID][MSGID=253][severity(1)][text(50)][CRC(2)]
 *
 * CRC is X.25 (MAVLink's crc_accumulate) over bytes 1..56 plus the
 * message-specific "CRC extra" byte — 83 for STATUSTEXT in MAVLink v1
 * common.xml. CRC stored little-endian.
 *
 * Messages are rate-limited per-reason to avoid MAVLink stream flood
 * during sustained failures.
 */
#pragma once

#include "dl_config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DL_MAV_SEV_EMERGENCY  0
#define DL_MAV_SEV_ALERT      1
#define DL_MAV_SEV_CRITICAL   2
#define DL_MAV_SEV_ERROR      3
#define DL_MAV_SEV_WARNING    4
#define DL_MAV_SEV_NOTICE     5
#define DL_MAV_SEV_INFO       6
#define DL_MAV_SEV_DEBUG      7

typedef struct dl_mavlink dl_mavlink_t;

/* Open the UDP socket and return an emitter. Returns NULL if
 * mavlink_enable is false or the socket open fails. */
dl_mavlink_t *dl_mavlink_open(const dl_config_t *cfg);

void dl_mavlink_close(dl_mavlink_t *m);

/* Emit one STATUSTEXT. `reason` is a short tag for the rate limiter
 * (e.g. "reject", "watchdog"); same reason within min_interval_ms is
 * dropped. `severity` is DL_MAV_SEV_*. `text` is truncated to 50
 * bytes. Returns 0 on sent, 1 on throttled, -1 on send error. */
int dl_mavlink_emit(dl_mavlink_t *m, const char *reason,
                    uint8_t severity, const char *text);

/* Low-level: encode a STATUSTEXT frame into `buf`. Returns frame size
 * on success, 0 on failure. Exported for tests. */
size_t dl_mavlink_encode_statustext(uint8_t *buf, size_t buflen,
                                    uint8_t seq, uint8_t sysid,
                                    uint8_t compid, uint8_t severity,
                                    const char *text);

/* X.25 CRC used by MAVLink v1/v2. Init `*crc` to 0xFFFF. Exported
 * for tests. */
void dl_mavlink_crc_accumulate(uint8_t data, uint16_t *crc);
