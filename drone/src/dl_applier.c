/* dl_applier.c — GS decision-packet dispatcher.
 *
 * Single-threaded poll(2) loop over:
 *   - UDP listen socket (decisions from GS over wfb-ng tunnel).
 *   - timerfd       (watchdog + OSD tick).
 *
 * On every accepted decision:
 *   parse_wire -> ceiling_check -> dispatch backends -> reset watchdog
 *   -> OSD update. Partial backend failure is logged but the loop
 *   continues.
 *
 * On watchdog trip:
 *   one-shot safe_defaults push; latch until the next fresh decision.
 */
#include "dl_backend_enc.h"
#include "dl_backend_radio.h"
#include "dl_backend_tx.h"
#include "dl_ceiling.h"
#include "dl_config.h"
#include "dl_log.h"
#include "dl_mavlink.h"
#include "dl_osd.h"
#include "dl_watchdog.h"
#include "dl_wire.h"

#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define RX_BUF_SIZE 256

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

static uint64_t now_monotonic_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

static int open_listen_socket(const dl_config_t *cfg) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        dl_log_err("listen: socket: %s", strerror(errno));
        return -1;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg->listen_port);
    if (inet_pton(AF_INET, cfg->listen_addr, &addr.sin_addr) != 1) {
        dl_log_err("listen: bad addr %s", cfg->listen_addr);
        close(fd);
        return -1;
    }
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        dl_log_err("listen: bind %s:%u: %s",
                   cfg->listen_addr, cfg->listen_port, strerror(errno));
        close(fd);
        return -1;
    }
    dl_log_info("listen: bound %s:%u", cfg->listen_addr, cfg->listen_port);
    return fd;
}

static int open_tick_timer(uint32_t interval_ms) {
    int fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (fd < 0) {
        dl_log_err("tick: timerfd_create: %s", strerror(errno));
        return -1;
    }
    struct itimerspec ts = {0};
    ts.it_value.tv_sec = interval_ms / 1000;
    ts.it_value.tv_nsec = (long)(interval_ms % 1000) * 1000000L;
    ts.it_interval = ts.it_value;
    if (timerfd_settime(fd, 0, &ts, NULL) < 0) {
        dl_log_err("tick: timerfd_settime: %s", strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

static bool dedup(uint32_t seq, uint32_t *last_seq, bool *ever) {
    /* Drop exact duplicates and out-of-order older-than-last sequences.
     * Wrap tolerance: treat (seq - last) as signed 32-bit — negatives are
     * old, positives are new. */
    if (!*ever) {
        *last_seq = seq;
        *ever = true;
        return false;  /* not a dup */
    }
    int32_t delta = (int32_t)(seq - *last_seq);
    if (delta <= 0) return true;
    *last_seq = seq;
    return false;
}

typedef struct {
    const char         *config_path;
    bool                log_debug;
} cli_args_t;

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s --config <drone.conf> [--debug]\n"
        "\n"
        "  --config PATH   path to drone.conf\n"
        "  --debug         enable DEBUG-level logging\n",
        prog);
}

static int parse_args(int argc, char **argv, cli_args_t *out) {
    static struct option opts[] = {
        { "config", required_argument, 0, 'c' },
        { "debug",  no_argument,       0, 'd' },
        { "help",   no_argument,       0, 'h' },
        { 0 }
    };
    out->config_path = NULL;
    out->log_debug = false;
    int c;
    while ((c = getopt_long(argc, argv, "c:dh", opts, NULL)) != -1) {
        switch (c) {
            case 'c': out->config_path = optarg; break;
            case 'd': out->log_debug = true; break;
            case 'h': usage(argv[0]); return 1;
            default:  usage(argv[0]); return -1;
        }
    }
    if (!out->config_path) {
        fprintf(stderr, "--config is required\n");
        usage(argv[0]);
        return -1;
    }
    return 0;
}

int main(int argc, char **argv) {
    cli_args_t args;
    int pa = parse_args(argc, argv, &args);
    if (pa != 0) return pa < 0 ? 2 : 0;

    dl_log_init("dl-applier", args.log_debug ? DL_LOG_DEBUG : DL_LOG_INFO);
    dl_log_info("dynamic-link applier starting");

    dl_config_t cfg;
    dl_config_defaults(&cfg);
    if (dl_config_load(args.config_path, &cfg) < 0) {
        dl_log_fatal("config load failed");
        return 3;
    }
    if (dl_config_validate(&cfg) < 0) {
        dl_log_fatal("config validation failed");
        return 3;
    }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    int listen_fd = open_listen_socket(&cfg);
    if (listen_fd < 0) return 4;

    uint32_t tick_ms = cfg.osd_update_interval_ms;
    if (cfg.health_timeout_ms / 2 < tick_ms) tick_ms = cfg.health_timeout_ms / 2;
    if (tick_ms < 100) tick_ms = 100;
    int tick_fd = open_tick_timer(tick_ms);
    if (tick_fd < 0) { close(listen_fd); return 4; }

    dl_backend_tx_t    *bt = dl_backend_tx_open(&cfg);
    dl_backend_radio_t *br = dl_backend_radio_open(&cfg);
    dl_backend_enc_t   *be = dl_backend_enc_open(&cfg);
    dl_osd_t           *osd = dl_osd_open(&cfg);
    dl_mavlink_t       *mav = dl_mavlink_open(&cfg);

    if (!bt) dl_log_warn("tx backend not available; FEC/DEPTH/RADIO drops");

    dl_watchdog_t wd;
    dl_watchdog_init(&wd, cfg.health_timeout_ms);

    /* One prev-state per backend — each tracks only the knobs it
     * actually applies, and mutating one must not fool the others. */
    dl_decision_t last_tx = {0};
    dl_decision_t last_radio = {0};
    dl_decision_t last_enc = {0};
    dl_decision_t last_applied = {0};  /* for OSD display only */
    uint32_t last_seq = 0;
    bool     ever_seen_seq = false;
    int      last_rssi_dBm = 0;  /* Phase 2 will populate from MAVLink */

    struct pollfd pfds[2];
    pfds[0].fd = listen_fd; pfds[0].events = POLLIN;
    pfds[1].fd = tick_fd;   pfds[1].events = POLLIN;

    while (!g_stop) {
        int n = poll(pfds, 2, -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            dl_log_err("poll: %s", strerror(errno));
            break;
        }

        if (pfds[0].revents & POLLIN) {
            uint8_t buf[RX_BUF_SIZE];
            struct sockaddr_in src;
            socklen_t slen = sizeof(src);
            ssize_t got = recvfrom(listen_fd, buf, sizeof(buf), 0,
                                   (struct sockaddr *)&src, &slen);
            if (got < 0) {
                if (errno != EAGAIN && errno != EINTR) {
                    dl_log_warn("recvfrom: %s", strerror(errno));
                }
            } else {
                dl_decision_t d;
                dl_decode_result_t dr = dl_wire_decode(buf, (size_t)got, &d);
                if (dr != DL_DECODE_OK) {
                    dl_log_warn("decode: result=%d from %s:%u (%zd bytes)",
                                (int)dr,
                                inet_ntoa(src.sin_addr), ntohs(src.sin_port),
                                got);
                } else if (dedup(d.sequence, &last_seq, &ever_seen_seq)) {
                    dl_log_debug("decode: duplicate seq=%u", d.sequence);
                } else {
                    dl_ceiling_result_t cr = dl_ceiling_check(&d, &cfg);
                    if (cr != DL_CEILING_OK) {
                        const char *rsn = dl_ceiling_reason(cr);
                        dl_osd_event_reject(osd, rsn);
                        char text[64];
                        snprintf(text, sizeof(text), "DL REJECT %s", rsn);
                        dl_mavlink_emit(mav, "reject",
                                        DL_MAV_SEV_WARNING, text);
                    } else {
                        uint64_t now = now_monotonic_ms();
                        int drc = 0;
                        if (bt && dl_backend_tx_apply(bt, &d, &last_tx) < 0) drc = -1;
                        if (br && dl_backend_radio_apply(br, &d, &last_radio) < 0) drc = -1;
                        if (be && dl_backend_enc_apply(be, &d, &last_enc) < 0) drc = -1;
                        if (d.flags & DL_FLAG_IDR_REQUEST) {
                            dl_backend_enc_request_idr(be, now);
                        }
                        if (drc < 0) {
                            dl_log_warn("apply: at least one backend failed seq=%u",
                                        d.sequence);
                            dl_mavlink_emit(mav, "apply_fail",
                                            DL_MAV_SEV_WARNING,
                                            "DL APPLY_FAIL backend");
                        } else {
                            dl_log_debug("apply: seq=%u mcs=%u k=%u n=%u d=%u tx=%d br=%u",
                                         d.sequence, d.mcs, d.k, d.n, d.depth,
                                         d.tx_power_dBm, d.bitrate_kbps);
                        }
                        last_applied = d;
                        dl_osd_write_status(osd, &last_applied, last_rssi_dBm);
                        dl_watchdog_notify_decision(&wd, now);
                    }
                }
            }
        }

        if (pfds[1].revents & POLLIN) {
            uint64_t expirations;
            ssize_t r = read(tick_fd, &expirations, sizeof(expirations));
            (void)r;  /* drained; don't care about the count */
            uint64_t now = now_monotonic_ms();
            if (dl_watchdog_tick(&wd, now)) {
                dl_log_warn("watchdog tripped; pushing safe_defaults");
                if (bt) dl_backend_tx_apply_safe(bt, &cfg);
                if (br) dl_backend_radio_apply_safe(br, &cfg);
                if (be) dl_backend_enc_apply_safe(be, &cfg);
                dl_osd_event_watchdog(osd);
                dl_mavlink_emit(mav, "watchdog", DL_MAV_SEV_ERROR,
                                "DL WATCHDOG safe_defaults");
                /* Invalidate last-states so the next fresh decision emits
                 * everything. */
                memset(&last_tx, 0, sizeof(last_tx));
                memset(&last_radio, 0, sizeof(last_radio));
                memset(&last_enc, 0, sizeof(last_enc));
            }
        }
    }

    dl_log_info("dl-applier shutting down");
    close(listen_fd);
    close(tick_fd);
    dl_mavlink_close(mav);
    dl_osd_close(osd);
    dl_backend_enc_close(be);
    dl_backend_radio_close(br);
    dl_backend_tx_close(bt);
    return 0;
}
