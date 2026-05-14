/* dl_backend_tx.c — speak the wfb-ng tx_cmd.h protocol on UDP. */
#include "dl_backend_tx.h"
#include "dl_dbg.h"
#include "dl_latency.h"
#include "dl_log.h"
#include "vendored/tx_cmd.h"

#include <arpa/inet.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

struct dl_backend_tx {
    int fd;
    struct sockaddr_in dst;
    /* Microsecond pacing between sub-commands (FEC, DEPTH, RADIO).
     * See cfg->apply_sub_pace_ms — wfb-ng's refresh_session burst
     * needs space to land before the next SET_* hits. 0 = no pacing. */
    useconds_t pace_us;
    bool interleaving_supported;
};

static void pace(const dl_backend_tx_t *bt) {
    if (bt->pace_us > 0) usleep(bt->pace_us);
}

static uint32_t next_req_id(void) {
    /* Simple monotonic; wfb-ng only echoes it so uniqueness within a
     * short window is enough. */
    static uint32_t id = 1;
    return id++;
}

/* Discard any queued replies left over from previous timed-out requests.
 * Without this, the next recv() would read a stale reply whose req_id
 * doesn't match, desyncing the socket for the rest of the process's
 * lifetime. */
static int drain_pending(int fd) {
    cmd_resp_t junk;
    int drained = 0;
    while (recv(fd, &junk, sizeof(junk), MSG_DONTWAIT) >= 0) drained++;
    return drained;
}

static int send_and_recv(int fd, const cmd_req_t *req, size_t req_len,
                         cmd_resp_t *resp, const char *label) {
    int n_drained = drain_pending(fd);
    if (n_drained > 0) {
        dl_log_debug("tx_cmd %s: drained %d stale reply%s before send",
                     label, n_drained, n_drained == 1 ? "" : "ies");
    }

    ssize_t nsent = send(fd, req, req_len, 0);
    if (nsent < 0) {
        int saved = errno;
        dl_log_warn("tx_cmd %s: send: %s", label, strerror(saved));
        char detail[128];
        snprintf(detail, sizeof(detail),
                 "{\"cmd\":\"%s\",\"errno\":%d}", label, saved);
        dl_dbg_emit("TX_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
        return -1;
    }
    if ((size_t)nsent != req_len) {
        dl_log_warn("tx_cmd %s: short send %zd/%zu", label, nsent, req_len);
        char detail[128];
        snprintf(detail, sizeof(detail),
                 "{\"cmd\":\"%s\",\"sent\":%zd,\"want\":%zu}",
                 label, nsent, req_len);
        dl_dbg_emit("TX_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
        return -1;
    }

    /* Loop on req_id mismatch: a stale reply may still be in transit
     * from a prior timed-out request. Discard and recv again within
     * the SO_RCVTIMEO budget; bail when recv times out. */
    for (;;) {
        ssize_t nrecv = recv(fd, resp, sizeof(*resp), 0);
        if (nrecv < 0) {
            int saved = errno;
            dl_log_warn("tx_cmd %s: recv: %s", label, strerror(saved));
            char detail[128];
            snprintf(detail, sizeof(detail),
                     "{\"cmd\":\"%s\",\"errno\":%d}", label, saved);
            dl_dbg_emit("TX_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
            return -1;
        }
        if ((size_t)nrecv < offsetof(cmd_resp_t, u)) {
            dl_log_warn("tx_cmd %s: short reply %zd bytes", label, nrecv);
            char detail[128];
            snprintf(detail, sizeof(detail),
                     "{\"cmd\":\"%s\",\"reply_len\":%zd}", label, nrecv);
            dl_dbg_emit("TX_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
            return -1;
        }
        if (ntohl(resp->req_id) == ntohl(req->req_id)) break;
        dl_log_debug("tx_cmd %s: discard late reply (sent %u got %u)",
                     label,
                     (unsigned)ntohl(req->req_id),
                     (unsigned)ntohl(resp->req_id));
    }

    uint32_t rc = ntohl(resp->rc);
    if (rc != 0) {
        dl_log_warn("tx_cmd %s: rc=%u (errno=%s)",
                    label, rc, strerror((int)rc));
        char detail[128];
        snprintf(detail, sizeof(detail),
                 "{\"cmd\":\"%s\",\"rc\":%u}", label, rc);
        dl_dbg_emit("TX_APPLY_FAIL", DL_DBG_SEV_WARN, detail);
        return -1;
    }
    return 0;
}

dl_backend_tx_t *dl_backend_tx_open(const dl_config_t *cfg) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        dl_log_err("tx_cmd: socket: %s", strerror(errno));
        return NULL;
    }

    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(cfg->wfb_tx_ctrl_port);
    if (inet_pton(AF_INET, cfg->wfb_tx_ctrl_addr, &dst.sin_addr) != 1) {
        dl_log_err("tx_cmd: bad wfb_tx_ctrl_addr %s", cfg->wfb_tx_ctrl_addr);
        close(fd);
        return NULL;
    }
    if (connect(fd, (struct sockaddr *)&dst, sizeof(dst)) < 0) {
        dl_log_err("tx_cmd: connect %s:%u: %s",
                   cfg->wfb_tx_ctrl_addr, cfg->wfb_tx_ctrl_port,
                   strerror(errno));
        close(fd);
        return NULL;
    }

    /* 500 ms — comfortably above observed wfb_tx round-trip on a busy
     * SoC (200 ms was too tight on SSC338Q under load), but well below
     * the 10 Hz decision interval so a single hung command can't stall
     * a whole apply cycle. */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        dl_log_warn("tx_cmd: SO_RCVTIMEO: %s", strerror(errno));
    }

    dl_backend_tx_t *bt = calloc(1, sizeof(*bt));
    if (!bt) {
        close(fd);
        return NULL;
    }
    bt->fd = fd;
    bt->dst = dst;
    bt->pace_us = (useconds_t)cfg->apply_sub_pace_ms * 1000u;
    bt->interleaving_supported = cfg->interleaving_supported;
    dl_log_info("tx_cmd: connected to %s:%u (sub_pace=%u ms, interleaver=%s)",
                cfg->wfb_tx_ctrl_addr, cfg->wfb_tx_ctrl_port,
                (unsigned)cfg->apply_sub_pace_ms,
                cfg->interleaving_supported ? "enabled" : "vanilla");
    return bt;
}

void dl_backend_tx_close(dl_backend_tx_t *bt) {
    if (!bt) return;
    if (bt->fd >= 0) close(bt->fd);
    free(bt);
}

static int send_fec(dl_backend_tx_t *bt, uint8_t k, uint8_t n) {
    cmd_req_t req = { .req_id = htonl(next_req_id()), .cmd_id = CMD_SET_FEC };
    cmd_resp_t resp;
    req.u.cmd_set_fec.k = k;
    req.u.cmd_set_fec.n = n;
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_FEC);
    int rc = send_and_recv(bt->fd, &req,
                           offsetof(cmd_req_t, u) + sizeof(req.u.cmd_set_fec),
                           &resp, "set_fec");
    dl_latency_end(h, rc);
    return rc;
}

static int send_depth(dl_backend_tx_t *bt, uint8_t depth) {
    cmd_req_t req = {
        .req_id = htonl(next_req_id()),
        .cmd_id = CMD_SET_INTERLEAVE_DEPTH,
    };
    cmd_resp_t resp;
    req.u.cmd_set_interleave_depth.depth = depth;
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_DPTH);
    int rc = send_and_recv(bt->fd, &req,
                           offsetof(cmd_req_t, u) +
                               sizeof(req.u.cmd_set_interleave_depth),
                           &resp, "set_interleave_depth");
    dl_latency_end(h, rc);
    return rc;
}

static int send_radio(dl_backend_tx_t *bt, uint8_t mcs, uint8_t bandwidth) {
    cmd_req_t req = { .req_id = htonl(next_req_id()), .cmd_id = CMD_SET_RADIO };
    cmd_resp_t resp;
    req.u.cmd_set_radio.stbc       = 0;
    req.u.cmd_set_radio.ldpc       = false;
    req.u.cmd_set_radio.short_gi   = false;  /* pinned (§1) */
    req.u.cmd_set_radio.bandwidth  = bandwidth;
    req.u.cmd_set_radio.mcs_index  = mcs;
    req.u.cmd_set_radio.vht_mode   = false;
    req.u.cmd_set_radio.vht_nss    = 1;
    dl_lat_handle_t h = dl_latency_begin(DL_LAT_RADIO);
    int rc = send_and_recv(bt->fd, &req,
                           offsetof(cmd_req_t, u) + sizeof(req.u.cmd_set_radio),
                           &resp, "set_radio");
    dl_latency_end(h, rc);
    return rc;
}

int dl_backend_tx_apply(dl_backend_tx_t *bt,
                        const dl_decision_t *d,
                        dl_decision_t *prev) {
    int rc = 0;
    bool first = (prev == NULL) || prev->magic != DL_WIRE_MAGIC;
    bool emitted = false;

    if (first || prev->k != d->k || prev->n != d->n) {
        if (send_fec(bt, d->k, d->n) < 0) rc = -1;
        emitted = true;
    }
    if (bt->interleaving_supported &&
        (first || prev->depth != d->depth)) {
        if (emitted) pace(bt);
        if (send_depth(bt, d->depth) < 0) rc = -1;
        emitted = true;
    }
    if (first || prev->mcs != d->mcs || prev->bandwidth != d->bandwidth) {
        if (emitted) pace(bt);
        if (send_radio(bt, d->mcs, d->bandwidth) < 0) rc = -1;
    }

    if (prev) *prev = *d;
    return rc;
}

int dl_backend_tx_apply_safe(dl_backend_tx_t *bt, const dl_config_t *cfg) {
    int rc = 0;
    if (send_fec(bt, cfg->safe_k, cfg->safe_n) < 0) rc = -1;
    if (bt->interleaving_supported) {
        pace(bt);
        if (send_depth(bt, cfg->safe_depth) < 0) rc = -1;
    }
    pace(bt);
    if (send_radio(bt, cfg->safe_mcs, cfg->safe_bandwidth) < 0) rc = -1;
    return rc;
}
