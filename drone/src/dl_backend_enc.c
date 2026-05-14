/* dl_backend_enc.c — raw-sockets HTTP GET for majestic / waybeam.
 *
 * We don't parse the response; majestic/waybeam return short JSON that
 * we have no use for. We just confirm the connect+send+some-reply
 * succeeded and move on.
 */
#include "dl_backend_enc.h"
#include "dl_dbg.h"
#include "dl_latency.h"
#include "dl_log.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define CONNECT_TIMEOUT_MS 1000
#define RECV_TIMEOUT_MS    500

struct dl_backend_enc {
    char     host[DL_CONF_MAX_STR];
    uint16_t port;
    uint32_t min_idr_interval_ms;
    uint64_t last_idr_ms;
    bool     idr_ever_sent;
};

/* Parse "HTTP/1.x NNN " out of the first line of a response buffer.
 * Returns the status code, or 0 if the buffer doesn't look like an
 * HTTP reply (silent encoder, garbage, etc.). */
static int parse_http_status(const char *buf, size_t len) {
    if (len < 12) return 0;
    if (memcmp(buf, "HTTP/", 5) != 0) return 0;
    /* Skip past version: HTTP/1.0 or HTTP/1.1, both 8 chars. */
    size_t i = 5;
    while (i < len && buf[i] != ' ') i++;
    if (i + 4 > len) return 0;
    /* buf[i] == ' '; status follows. */
    int s = 0;
    for (size_t j = i + 1; j < i + 4 && j < len; ++j) {
        if (buf[j] < '0' || buf[j] > '9') return 0;
        s = s * 10 + (buf[j] - '0');
    }
    return s;
}

static int http_get(const char *host, uint16_t port, const char *path) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        dl_log_warn("enc: socket: %s", strerror(errno));
        dl_dbg_emit_errno("ENC_APPLY_FAIL", errno);
        return -1;
    }

    struct timeval tv;
    tv.tv_sec = CONNECT_TIMEOUT_MS / 1000;
    tv.tv_usec = (CONNECT_TIMEOUT_MS % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    tv.tv_sec = RECV_TIMEOUT_MS / 1000;
    tv.tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &dst.sin_addr) != 1) {
        /* Fall back to resolver for hostnames. */
        struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
        struct addrinfo *res = NULL;
        char portstr[8];
        snprintf(portstr, sizeof(portstr), "%u", (unsigned)port);
        if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) {
            dl_log_warn("enc: resolve %s: %s", host, strerror(errno));
            dl_dbg_emit_errno("ENC_APPLY_FAIL", errno);
            close(fd);
            return -1;
        }
        memcpy(&dst, res->ai_addr, sizeof(dst));
        freeaddrinfo(res);
    }

    if (connect(fd, (struct sockaddr *)&dst, sizeof(dst)) < 0) {
        int saved = errno;
        dl_log_warn("enc: connect %s:%u: %s", host, port, strerror(saved));
        char detail[160];
        snprintf(detail, sizeof(detail),
                 "{\"errno\":%d,\"host\":\"%s\",\"port\":%u}",
                 saved, host, (unsigned)port);
        dl_dbg_emit("ENC_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
        close(fd);
        return -1;
    }

    char req[512];
    int n = snprintf(req, sizeof(req),
                     "GET %s HTTP/1.0\r\n"
                     "Host: %s\r\n"
                     "Connection: close\r\n"
                     "\r\n",
                     path, host);
    if (n < 0 || (size_t)n >= sizeof(req)) {
        dl_log_warn("enc: request too long");
        dl_dbg_emit("ENC_APPLY_FAIL", DL_DBG_SEV_WARN,
                    "{\"reason\":\"request too long\"}");
        close(fd);
        return -1;
    }

    ssize_t nsent = send(fd, req, (size_t)n, 0);
    if (nsent != n) {
        int saved = errno;
        dl_log_warn("enc: send: %s", strerror(saved));
        dl_dbg_emit_errno("ENC_APPLY_FAIL", saved);
        close(fd);
        return -1;
    }

    /* Capture the first ~512B of the reply so we can parse the status
     * line and surface a non-2xx body to the SD log. Anything beyond
     * that is drained and discarded. */
    char reply[512];
    size_t reply_len = 0;
    while (reply_len < sizeof(reply)) {
        ssize_t got = recv(fd, reply + reply_len,
                           sizeof(reply) - reply_len, 0);
        if (got <= 0) break;
        reply_len += (size_t)got;
    }
    /* Drain the rest. */
    char discard[256];
    while (recv(fd, discard, sizeof(discard), 0) > 0) {}
    close(fd);

    if (reply_len == 0) {
        /* Silent encoder. Log at debug — could mean the GET took effect
         * or could be a real failure. We don't escalate to dl_dbg here
         * because there's nothing actionable to capture; if the user
         * has a recurring silent-encoder problem, the log line above
         * shows the path that's getting nothing. */
        dl_log_debug("enc: no HTTP reply from %s:%u for %s",
                     host, port, path);
        return 0;
    }

    int status = parse_http_status(reply, reply_len);
    if (status >= 200 && status < 300) {
        return 0;
    }
    /* Non-2xx (or unparseable): capture the body for the SD log and
     * propagate as an error. This was previously returned 0 to avoid
     * the MAVLink apply_fail path; the debug-OSD work made surfacing
     * application-layer rejects worth the extra STATUSTEXT noise. See
     * docs/superpowers/specs/2026-05-13-debug-osd-apply-latency-design.md
     * §"Encoder error-contract change". */
    dl_log_warn("enc: http %d from %s:%u for %s",
                status, host, port, path);
    const char *body = reply;
    size_t body_len = reply_len;
    for (size_t i = 0; i + 3 < reply_len; ++i) {
        if (reply[i] == '\r' && reply[i+1] == '\n' &&
            reply[i+2] == '\r' && reply[i+3] == '\n') {
            body = reply + i + 4;
            body_len = reply_len - (i + 4);
            break;
        }
    }
    dl_dbg_emit_http("ENC_RESPONSE_BAD", DL_DBG_SEV_WARN,
                     status, body, body_len);
    return -1;
}

dl_backend_enc_t *dl_backend_enc_open(const dl_config_t *cfg) {
    dl_backend_enc_t *be = calloc(1, sizeof(*be));
    if (!be) return NULL;
    snprintf(be->host, sizeof(be->host), "%s", cfg->encoder_host);
    be->port = cfg->encoder_port;
    be->min_idr_interval_ms = cfg->min_idr_interval_ms;
    dl_log_info("enc: %s at %s:%u", cfg->encoder_kind, be->host, be->port);
    return be;
}

void dl_backend_enc_close(dl_backend_enc_t *be) {
    free(be);
}

static int apply_set(dl_backend_enc_t *be,
                     uint16_t bitrate_kbps, uint8_t roi_qp, uint8_t fps) {
    char path[256];
    char *p = path;
    size_t left = sizeof(path);
    int n = snprintf(p, left, "/api/v1/set?video0.bitrate=%u", bitrate_kbps);
    if (n < 0 || (size_t)n >= left) return -1;
    p += n; left -= (size_t)n;
    if (roi_qp != 0) {
        n = snprintf(p, left, "&fpv.roiQp=%u", roi_qp);
        if (n < 0 || (size_t)n >= left) return -1;
        p += n; left -= (size_t)n;
    }
    if (fps != 0) {
        n = snprintf(p, left, "&video0.fps=%u", fps);
        if (n < 0 || (size_t)n >= left) return -1;
        p += n; left -= (size_t)n;
    }
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_ENC);
    int rc = http_get(be->host, be->port, path);
    dl_latency_end(h, rc);
    return rc;
}

int dl_backend_enc_apply(dl_backend_enc_t *be,
                         const dl_decision_t *d,
                         dl_decision_t *prev) {
    if (!be) return -1;
    bool first = (prev == NULL) || prev->magic != DL_WIRE_MAGIC;
    bool changed = first
                || prev->bitrate_kbps != d->bitrate_kbps
                || prev->roi_qp       != d->roi_qp
                || prev->fps          != d->fps;
    int rc = 0;
    if (changed && d->bitrate_kbps > 0) {
        rc = apply_set(be, d->bitrate_kbps, d->roi_qp, d->fps);
    }
    if (prev) *prev = *d;
    return rc;
}

int dl_backend_enc_request_idr(dl_backend_enc_t *be, uint64_t now_ms) {
    if (!be) return -1;
    if (be->idr_ever_sent &&
        (now_ms - be->last_idr_ms) < be->min_idr_interval_ms) {
        dl_log_debug("enc: IDR throttled (%u ms since last, min %u)",
                     (unsigned)(now_ms - be->last_idr_ms),
                     (unsigned)be->min_idr_interval_ms);
        return 1;
    }
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_IDR);
    int rc = http_get(be->host, be->port, "/request/idr");
    dl_latency_end(h, rc);
    /* Arm the throttle on ANY attempt — whether the encoder
     * 2xx'd, 4xx'd, 5xx'd, or the connect failed. The throttle's
     * purpose is spam prevention; a broken encoder shouldn't get
     * hammered just because rc != 0. (This restores pre-Task-9
     * behavior the encoder-contract flip accidentally regressed.) */
    be->last_idr_ms = now_ms;
    be->idr_ever_sent = true;
    return rc;
}

int dl_backend_enc_apply_safe(dl_backend_enc_t *be, const dl_config_t *cfg) {
    if (!be) return -1;
    return apply_set(be, cfg->safe_bitrate_kbps, 0, 0);
}
