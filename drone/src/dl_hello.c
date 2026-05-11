/* dl_hello.c — see dl_hello.h. */
#include "dl_hello.h"
#include "dl_log.h"
#include "dl_yaml_get.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

static uint32_t random_u32(void) {
    /* Fallback path is fine: we only need per-boot uniqueness, not
     * cryptographic randomness. */
    uint32_t v = 0;
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd >= 0) {
        ssize_t r = read(fd, &v, sizeof(v));
        close(fd);
        if (r == (ssize_t)sizeof(v) && v != 0) return v;
    }
    struct timespec ts = {0};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    v = (uint32_t)(ts.tv_sec ^ ts.tv_nsec ^ (uint32_t)getpid());
    if (v == 0) v = 1;
    return v;
}

int dl_hello_init(dl_hello_sm_t *h, const dl_config_t *cfg) {
    memset(h, 0, sizeof(*h));
    h->cfg = cfg;
    h->state = DL_HELLO_STATE_DISABLED;

    int mtu = 0;
    int rc = dl_yaml_get_int(cfg->hello_wfb_yaml_path,
                             "wireless", "mlink", &mtu);
    if (rc != 0) {
        dl_log_err("dl_hello: failed to read mtu (%s wireless.mlink): %d",
                   cfg->hello_wfb_yaml_path, rc);
        return -1;
    }
    if (mtu <= 0 || mtu > 0xFFFF) {
        dl_log_err("dl_hello: mtu out of range: %d", mtu);
        return -1;
    }

    int fps = 0;
    rc = dl_yaml_get_int(cfg->hello_majestic_yaml_path,
                         "video0", "fps", &fps);
    if (rc != 0) {
        dl_log_err("dl_hello: failed to read fps (%s video0.fps): %d",
                   cfg->hello_majestic_yaml_path, rc);
        return -1;
    }
    if (fps <= 0 || fps > 0xFFFF) {
        dl_log_err("dl_hello: fps out of range: %d", fps);
        return -1;
    }

    h->mtu_bytes = (uint16_t)mtu;
    h->fps = (uint16_t)fps;
    h->generation_id = random_u32();
    h->state = DL_HELLO_STATE_ANNOUNCING;
    h->announce_count = 0;
    h->announces_without_ack = 0;
    h->keepalives_without_ack = 0;
    dl_log_info("dl_hello: ANNOUNCING gen=0x%08x mtu=%u fps=%u",
                h->generation_id, h->mtu_bytes, h->fps);
    return 0;
}

size_t dl_hello_build_announce(dl_hello_sm_t *h, uint8_t *buf, size_t buflen) {
    if (h->state == DL_HELLO_STATE_DISABLED) return 0;
    dl_hello_t pkt = {
        .version = DL_WIRE_VERSION,
        .flags = 0,
        .generation_id = h->generation_id,
        .mtu_bytes = h->mtu_bytes,
        .fps = h->fps,
        .applier_build_sha = 0,
    };
    size_t n = dl_wire_encode_hello(&pkt, buf, buflen);
    if (n == 0) return 0;
    h->announce_count++;
    return n;
}

uint32_t dl_hello_next_delay_ms(const dl_hello_sm_t *h) {
    if (h->state == DL_HELLO_STATE_DISABLED) return 0;
    if (h->state == DL_HELLO_STATE_KEEPALIVE) {
        return h->cfg->hello_keepalive_ms;
    }
    if (h->announce_count == 0) return 0;
    if (h->announce_count < h->cfg->hello_announce_initial_count) {
        return h->cfg->hello_announce_initial_ms;
    }
    return h->cfg->hello_announce_steady_ms;
}

int dl_hello_on_ack(dl_hello_sm_t *h, const dl_hello_ack_t *ack) {
    if (h->state == DL_HELLO_STATE_DISABLED) return 0;
    if (ack->generation_id_echo != h->generation_id) return 0;
    if (h->state != DL_HELLO_STATE_KEEPALIVE) {
        dl_log_info("dl_hello: KEEPALIVE gen=0x%08x", h->generation_id);
    }
    h->state = DL_HELLO_STATE_KEEPALIVE;
    h->keepalives_without_ack = 0;
    h->announce_count = 0;
    return 1;
}

int dl_hello_on_keepalive_tick(dl_hello_sm_t *h) {
    if (h->state != DL_HELLO_STATE_KEEPALIVE) return 0;
    h->keepalives_without_ack++;
    if (h->keepalives_without_ack >= 3) {
        dl_log_warn("dl_hello: 3 keepalives unacked, returning to ANNOUNCING");
        h->state = DL_HELLO_STATE_ANNOUNCING;
        h->announce_count = 0;
        h->keepalives_without_ack = 0;
        return 1;
    }
    return 0;
}
