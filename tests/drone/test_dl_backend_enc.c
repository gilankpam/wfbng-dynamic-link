/* test_dl_backend_enc.c — backend HTTP path: always-emit roiQp,
 * signed format, dedup against computed roi_qp. */
#include "test_main.h"
#include "dl_backend_enc.h"
#include "dl_config.h"
#include "dl_wire.h"

#include <arpa/inet.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct {
    int  port;
    int  fd;
    char received[2048];
    size_t received_len;
    int  request_count;
    pthread_t tid;
    int  stop;
} mock_http_t;

static void *mock_http_thread(void *arg) {
    mock_http_t *m = arg;
    while (!m->stop) {
        struct sockaddr_in c;
        socklen_t cl = sizeof(c);
        int cfd = accept(m->fd, (struct sockaddr *)&c, &cl);
        if (cfd < 0) return NULL;
        char buf[1024];
        ssize_t n = recv(cfd, buf, sizeof(buf) - 1, 0);
        if (n > 0) {
            size_t add = (size_t)n;
            if (m->received_len + add + 1 < sizeof(m->received)) {
                memcpy(m->received + m->received_len, buf, add);
                m->received_len += add;
                m->received[m->received_len++] = '\n';
                m->received[m->received_len] = '\0';
            }
            m->request_count++;
            const char *resp = "HTTP/1.0 200 OK\r\nContent-Length: 0\r\n\r\n";
            send(cfd, resp, strlen(resp), 0);
        }
        close(cfd);
    }
    return NULL;
}

static void mock_http_start(mock_http_t *m) {
    memset(m, 0, sizeof(*m));
    m->fd = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(m->fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in a = { .sin_family = AF_INET,
                             .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    bind(m->fd, (struct sockaddr *)&a, sizeof(a));
    socklen_t al = sizeof(a);
    getsockname(m->fd, (struct sockaddr *)&a, &al);
    m->port = ntohs(a.sin_port);
    listen(m->fd, 4);
    pthread_create(&m->tid, NULL, mock_http_thread, m);
}

static void mock_http_stop(mock_http_t *m) {
    m->stop = 1;
    shutdown(m->fd, SHUT_RDWR);
    close(m->fd);
    pthread_join(m->tid, NULL);
}

static void cfg_init(dl_config_t *cfg, int port) {
    dl_config_defaults(cfg);
    snprintf(cfg->encoder_host, sizeof(cfg->encoder_host), "%s", "127.0.0.1");
    cfg->encoder_port = (uint16_t)port;
}

DL_TEST(test_enc_emits_signed_roi_qp_when_starved) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    dl_decision_t d = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 4000, .fps = 60 };
    int rc = dl_backend_enc_apply(be, &d);
    DL_ASSERT_EQ(rc, 0);

    /* At 4000 kbps with defaults, roi_qp = -12. Path must include a
     * SIGNED %d-style value, not a wrap-around large uint. */
    DL_ASSERT(strstr(m.received, "fpv.roiQp=-12") != NULL);
    DL_ASSERT(strstr(m.received, "video0.bitrate=4000") != NULL);

    dl_backend_enc_close(be);
    mock_http_stop(&m);
}

DL_TEST(test_enc_emits_roi_qp_zero_above_threshold) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    dl_decision_t d = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 8000, .fps = 60 };
    int rc = dl_backend_enc_apply(be, &d);
    DL_ASSERT_EQ(rc, 0);
    /* The crucial bug-fix assertion: at 8000 kbps roi_qp = 0, and we
     * still send fpv.roiQp=0 (so waybeam clears any prior ROI). */
    DL_ASSERT(strstr(m.received, "fpv.roiQp=0") != NULL);

    dl_backend_enc_close(be);
    mock_http_stop(&m);
}

DL_TEST(test_enc_dedupes_repeat_apply) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    dl_decision_t d = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 4000, .fps = 60 };
    dl_backend_enc_apply(be, &d);
    int n1 = m.request_count;
    dl_backend_enc_apply(be, &d);  /* identical → no HTTP */
    DL_ASSERT_EQ(m.request_count, n1);
    dl_backend_enc_close(be);
    mock_http_stop(&m);
}

DL_TEST(test_enc_dedupes_within_quantization_band) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    /* Same bitrate+fps repeats are dedup'd. (Bitrate variation within
     * a quantization step is NOT dedup'd because the dedup key
     * includes raw bitrate, not just computed roi_qp.) */
    dl_decision_t d1 = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 4000, .fps = 60 };
    dl_decision_t d2 = { .magic = DL_WIRE_MAGIC, .bitrate_kbps = 4000, .fps = 60 };
    dl_backend_enc_apply(be, &d1);
    int n1 = m.request_count;
    dl_backend_enc_apply(be, &d2);  /* identical bitrate+fps → no HTTP */
    DL_ASSERT_EQ(m.request_count, n1);
    dl_backend_enc_close(be);
    mock_http_stop(&m);
}

DL_TEST(test_enc_safe_uses_compute_formula) {
    mock_http_t m;
    mock_http_start(&m);
    dl_config_t cfg;
    cfg_init(&cfg, m.port);
    cfg.safe_bitrate_kbps = 2000;   /* hits the floor */

    dl_backend_enc_t *be = dl_backend_enc_open(&cfg);
    int rc = dl_backend_enc_apply_safe(be, &cfg);
    DL_ASSERT_EQ(rc, 0);

    DL_ASSERT(strstr(m.received, "video0.bitrate=2000") != NULL);
    DL_ASSERT(strstr(m.received, "fpv.roiQp=-24") != NULL);

    dl_backend_enc_close(be);
    mock_http_stop(&m);
}
