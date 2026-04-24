/* dl_backend_tx.c — speak the wfb-ng tx_cmd.h protocol on UDP. */
#include "dl_backend_tx.h"
#include "dl_log.h"
#include "vendored/tx_cmd.h"

#include <arpa/inet.h>
#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

struct dl_backend_tx {
    int fd;
    struct sockaddr_in dst;
};

static uint32_t next_req_id(void) {
    /* Simple monotonic; wfb-ng only echoes it so uniqueness within a
     * short window is enough. */
    static uint32_t id = 1;
    return id++;
}

static int send_and_recv(int fd, const cmd_req_t *req, size_t req_len,
                         cmd_resp_t *resp, const char *label) {
    ssize_t nsent = send(fd, req, req_len, 0);
    if (nsent < 0) {
        dl_log_warn("tx_cmd %s: send: %s", label, strerror(errno));
        return -1;
    }
    if ((size_t)nsent != req_len) {
        dl_log_warn("tx_cmd %s: short send %zd/%zu", label, nsent, req_len);
        return -1;
    }
    /* 200 ms receive timeout via SO_RCVTIMEO, set once at open. */
    ssize_t nrecv = recv(fd, resp, sizeof(*resp), 0);
    if (nrecv < 0) {
        dl_log_warn("tx_cmd %s: recv: %s", label, strerror(errno));
        return -1;
    }
    if ((size_t)nrecv < offsetof(cmd_resp_t, u)) {
        dl_log_warn("tx_cmd %s: short reply %zd bytes", label, nrecv);
        return -1;
    }
    if (ntohl(resp->req_id) != ntohl(req->req_id)) {
        dl_log_warn("tx_cmd %s: req_id mismatch (sent %u got %u)",
                    label,
                    (unsigned)ntohl(req->req_id),
                    (unsigned)ntohl(resp->req_id));
        return -1;
    }
    uint32_t rc = ntohl(resp->rc);
    if (rc != 0) {
        dl_log_warn("tx_cmd %s: rc=%u (errno=%s)",
                    label, rc, strerror((int)rc));
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

    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
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
    dl_log_info("tx_cmd: connected to %s:%u",
                cfg->wfb_tx_ctrl_addr, cfg->wfb_tx_ctrl_port);
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
    return send_and_recv(bt->fd, &req,
                         offsetof(cmd_req_t, u) + sizeof(req.u.cmd_set_fec),
                         &resp, "set_fec");
}

static int send_depth(dl_backend_tx_t *bt, uint8_t depth) {
    cmd_req_t req = {
        .req_id = htonl(next_req_id()),
        .cmd_id = CMD_SET_INTERLEAVE_DEPTH,
    };
    cmd_resp_t resp;
    req.u.cmd_set_interleave_depth.depth = depth;
    return send_and_recv(bt->fd, &req,
                         offsetof(cmd_req_t, u) +
                             sizeof(req.u.cmd_set_interleave_depth),
                         &resp, "set_interleave_depth");
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
    return send_and_recv(bt->fd, &req,
                         offsetof(cmd_req_t, u) + sizeof(req.u.cmd_set_radio),
                         &resp, "set_radio");
}

int dl_backend_tx_apply(dl_backend_tx_t *bt,
                        const dl_decision_t *d,
                        dl_decision_t *prev) {
    int rc = 0;
    bool first = (prev == NULL) || prev->magic != DL_WIRE_MAGIC;

    if (first || prev->k != d->k || prev->n != d->n) {
        if (send_fec(bt, d->k, d->n) < 0) rc = -1;
    }
    if (first || prev->depth != d->depth) {
        if (send_depth(bt, d->depth) < 0) rc = -1;
    }
    if (first || prev->mcs != d->mcs || prev->bandwidth != d->bandwidth) {
        if (send_radio(bt, d->mcs, d->bandwidth) < 0) rc = -1;
    }

    if (prev) *prev = *d;
    return rc;
}

int dl_backend_tx_apply_safe(dl_backend_tx_t *bt, const dl_config_t *cfg) {
    int rc = 0;
    if (send_fec(bt, cfg->safe_k, cfg->safe_n) < 0) rc = -1;
    if (send_depth(bt, cfg->safe_depth) < 0) rc = -1;
    if (send_radio(bt, cfg->safe_mcs, cfg->safe_bandwidth) < 0) rc = -1;
    return rc;
}
