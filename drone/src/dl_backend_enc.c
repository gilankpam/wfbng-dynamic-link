/* dl_backend_enc.c — raw-sockets HTTP GET for majestic / waybeam.
 *
 * We don't parse the response; majestic/waybeam return short JSON that
 * we have no use for. We just confirm the connect+send+some-reply
 * succeeded and move on.
 */
#include "dl_backend_enc.h"
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

static int http_get(const char *host, uint16_t port, const char *path) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        dl_log_warn("enc: socket: %s", strerror(errno));
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
            close(fd);
            return -1;
        }
        memcpy(&dst, res->ai_addr, sizeof(dst));
        freeaddrinfo(res);
    }

    if (connect(fd, (struct sockaddr *)&dst, sizeof(dst)) < 0) {
        dl_log_warn("enc: connect %s:%u: %s", host, port, strerror(errno));
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
        close(fd);
        return -1;
    }

    ssize_t nsent = send(fd, req, (size_t)n, 0);
    if (nsent != n) {
        dl_log_warn("enc: send: %s", strerror(errno));
        close(fd);
        return -1;
    }

    /* Drain (and discard) whatever reply we get. Short timeout — if the
     * encoder dropped the connection that's fine, we moved on. */
    char discard[256];
    ssize_t total = 0;
    while (total < 4096) {
        ssize_t got = recv(fd, discard, sizeof(discard), 0);
        if (got <= 0) break;
        total += got;
    }
    close(fd);

    if (total <= 0) {
        /* A silent encoder is suspicious but not fatal; GET might
         * still have taken effect. Log once per call. */
        dl_log_debug("enc: no HTTP reply from %s:%u for %s", host, port, path);
    }
    return 0;
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
    /* Compose query string with only the fields we want to change. */
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
    return http_get(be->host, be->port, path);
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
    int rc = http_get(be->host, be->port, "/request/idr");
    if (rc == 0) {
        be->last_idr_ms = now_ms;
        be->idr_ever_sent = true;
    }
    return rc;
}

int dl_backend_enc_apply_safe(dl_backend_enc_t *be, const dl_config_t *cfg) {
    if (!be) return -1;
    return apply_set(be, cfg->safe_bitrate_kbps, 0, 0);
}
