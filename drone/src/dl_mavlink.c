/* dl_mavlink.c */
#include "dl_mavlink.h"
#include "dl_log.h"

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define MAVLINK_V1_STX            0xFE
#define MAVLINK_MSG_ID_STATUSTEXT 253
#define STATUSTEXT_PAYLOAD_LEN    51    /* severity(1) + text(50) */
#define STATUSTEXT_CRC_EXTRA      83    /* from MAVLink v1 common.xml */
#define STATUSTEXT_FRAME_LEN      (6 + STATUSTEXT_PAYLOAD_LEN + 2)  /* = 59 */

/* Rate-limiter: keep N most-recent (reason, last_sent_ms) entries. */
#define REASON_SLOTS 8
#define MIN_INTERVAL_MS 500  /* ≤2 msgs/sec per reason */

typedef struct {
    char     reason[16];
    uint64_t last_sent_ms;
} reason_slot_t;

struct dl_mavlink {
    int                fd;
    struct sockaddr_in dst;
    uint8_t            seq;
    uint8_t            sysid;
    uint8_t            compid;
    reason_slot_t      slots[REASON_SLOTS];
};

void dl_mavlink_crc_accumulate(uint8_t data, uint16_t *crc) {
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xFFu);
    tmp = (uint8_t)(tmp ^ (tmp << 4));
    *crc = (uint16_t)((*crc >> 8)
                    ^ ((uint16_t)tmp << 8)
                    ^ ((uint16_t)tmp << 3)
                    ^ ((uint16_t)tmp >> 4));
}

static uint16_t crc_calc(const uint8_t *buf, size_t len, uint8_t extra) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) dl_mavlink_crc_accumulate(buf[i], &crc);
    dl_mavlink_crc_accumulate(extra, &crc);
    return crc;
}

size_t dl_mavlink_encode_statustext(uint8_t *buf, size_t buflen,
                                    uint8_t seq, uint8_t sysid,
                                    uint8_t compid, uint8_t severity,
                                    const char *text) {
    if (buflen < STATUSTEXT_FRAME_LEN) return 0;

    buf[0] = MAVLINK_V1_STX;
    buf[1] = STATUSTEXT_PAYLOAD_LEN;
    buf[2] = seq;
    buf[3] = sysid;
    buf[4] = compid;
    buf[5] = MAVLINK_MSG_ID_STATUSTEXT;
    buf[6] = severity;

    /* text: up to 50 bytes, zero-padded (no NUL required by MAVLink). */
    size_t tlen = text ? strnlen(text, 50) : 0;
    if (tlen > 50) tlen = 50;
    memset(&buf[7], 0, 50);
    if (tlen > 0) memcpy(&buf[7], text, tlen);

    /* CRC over bytes 1..56 = LEN + (seq, sysid, compid, msgid) + 51-byte payload. */
    uint16_t crc = crc_calc(&buf[1], 1 + 4 + STATUSTEXT_PAYLOAD_LEN,
                            STATUSTEXT_CRC_EXTRA);
    buf[57] = (uint8_t)(crc & 0xFFu);
    buf[58] = (uint8_t)(crc >> 8);
    return STATUSTEXT_FRAME_LEN;
}

static uint64_t now_monotonic_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

static bool rate_limit(dl_mavlink_t *m, const char *reason, uint64_t now) {
    /* Find existing slot. */
    for (int i = 0; i < REASON_SLOTS; ++i) {
        if (m->slots[i].reason[0] != '\0' &&
            strncmp(m->slots[i].reason, reason, sizeof(m->slots[i].reason)) == 0) {
            if ((now - m->slots[i].last_sent_ms) < MIN_INTERVAL_MS) {
                return true;  /* drop */
            }
            m->slots[i].last_sent_ms = now;
            return false;
        }
    }
    /* Find free slot. */
    int oldest_idx = 0;
    uint64_t oldest_ts = UINT64_MAX;
    for (int i = 0; i < REASON_SLOTS; ++i) {
        if (m->slots[i].reason[0] == '\0') {
            snprintf(m->slots[i].reason, sizeof(m->slots[i].reason),
                     "%s", reason);
            m->slots[i].last_sent_ms = now;
            return false;
        }
        if (m->slots[i].last_sent_ms < oldest_ts) {
            oldest_ts = m->slots[i].last_sent_ms;
            oldest_idx = i;
        }
    }
    /* No free slot — evict LRU. */
    snprintf(m->slots[oldest_idx].reason, sizeof(m->slots[oldest_idx].reason),
             "%s", reason);
    m->slots[oldest_idx].last_sent_ms = now;
    return false;
}

dl_mavlink_t *dl_mavlink_open(const dl_config_t *cfg) {
    if (!cfg->mavlink_enable) {
        dl_log_info("mavlink: disabled in config");
        return NULL;
    }
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        dl_log_warn("mavlink: socket: %s", strerror(errno));
        return NULL;
    }
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(cfg->mavlink_port);
    if (inet_pton(AF_INET, cfg->mavlink_addr, &dst.sin_addr) != 1) {
        dl_log_warn("mavlink: bad mavlink_addr %s", cfg->mavlink_addr);
        close(fd);
        return NULL;
    }
    /* Use connect so we can send() instead of sendto() — also means
     * we learn about ECONNREFUSED if nobody's listening. Not fatal. */
    if (connect(fd, (struct sockaddr *)&dst, sizeof(dst)) < 0) {
        dl_log_warn("mavlink: connect %s:%u: %s (continuing)",
                    cfg->mavlink_addr, cfg->mavlink_port, strerror(errno));
    }

    dl_mavlink_t *m = calloc(1, sizeof(*m));
    if (!m) { close(fd); return NULL; }
    m->fd = fd;
    m->dst = dst;
    m->sysid = cfg->mavlink_sysid;
    m->compid = cfg->mavlink_compid;
    dl_log_info("mavlink: emitting to %s:%u (sysid=%u compid=%u)",
                cfg->mavlink_addr, cfg->mavlink_port,
                m->sysid, m->compid);
    return m;
}

void dl_mavlink_close(dl_mavlink_t *m) {
    if (!m) return;
    if (m->fd >= 0) close(m->fd);
    free(m);
}

int dl_mavlink_emit(dl_mavlink_t *m, const char *reason,
                    uint8_t severity, const char *text) {
    if (!m) return 0;  /* disabled — silently succeed */
    uint64_t now = now_monotonic_ms();
    if (rate_limit(m, reason, now)) return 1;

    uint8_t buf[STATUSTEXT_FRAME_LEN];
    size_t n = dl_mavlink_encode_statustext(buf, sizeof(buf),
                                            m->seq++, m->sysid,
                                            m->compid, severity, text);
    if (n == 0) return -1;
    ssize_t sent = send(m->fd, buf, n, MSG_NOSIGNAL);
    if (sent < 0) {
        dl_log_debug("mavlink: send: %s", strerror(errno));
        return -1;
    }
    return 0;
}
