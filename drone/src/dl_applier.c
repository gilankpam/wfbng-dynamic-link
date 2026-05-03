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
#include "dl_apply.h"
#include "dl_backend_enc.h"
#include "dl_backend_radio.h"
#include "dl_backend_tx.h"
#include "dl_ceiling.h"
#include "dl_config.h"
#include "dl_dbg.h"
#include "dl_dedup.h"
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

static uint64_t now_monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

/* Open an unconnected UDP socket for drone→GS tunnel traffic (PONG
 * packets) and resolve the GS endpoint into `out_dst`. Returns -1 on
 * failure (logged); missing socket is non-fatal and disables PONG.
 *
 * Why no connect(): connect() on UDP triggers an immediate route
 * lookup, and this applier may start before wfb-ng's TUN interface is
 * up — the ENETUNREACH at that moment used to latch gs_tunnel_fd=-1
 * for the rest of the process. With sendto() per packet, the route
 * lookup happens fresh each call, so the path self-heals once the
 * tunnel comes up. */
static int open_gs_tunnel_socket(const dl_config_t *cfg,
                                 struct sockaddr_in *out_dst) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        dl_log_warn("gs_tunnel: socket: %s", strerror(errno));
        return -1;
    }
    memset(out_dst, 0, sizeof(*out_dst));
    out_dst->sin_family = AF_INET;
    out_dst->sin_port = htons(cfg->gs_tunnel_port);
    if (inet_pton(AF_INET, cfg->gs_tunnel_addr, &out_dst->sin_addr) != 1) {
        dl_log_warn("gs_tunnel: bad addr %s", cfg->gs_tunnel_addr);
        close(fd);
        return -1;
    }
    dl_log_info("gs_tunnel: ready %s:%u (sendto per PONG)",
                cfg->gs_tunnel_addr, cfg->gs_tunnel_port);
    return fd;
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

typedef enum {
    APPLY_IDLE = 0,
    APPLY_UP_GAP,    /* phase 1 = tx+radio applied; encoder pending */
    APPLY_DOWN_GAP,  /* phase 1 = encoder applied; tx+radio pending */
} apply_state_t;

static int arm_gap(int gap_fd, uint32_t ms) {
    struct itimerspec ts = {0};
    ts.it_value.tv_sec  = ms / 1000;
    ts.it_value.tv_nsec = (long)(ms % 1000) * 1000000L;
    /* it_interval stays zero -> single-shot */
    if (timerfd_settime(gap_fd, 0, &ts, NULL) < 0) {
        dl_log_warn("gap: arm: %s", strerror(errno));
        return -1;
    }
    return 0;
}

static int disarm_gap(int gap_fd) {
    struct itimerspec ts = {0};
    if (timerfd_settime(gap_fd, 0, &ts, NULL) < 0) {
        dl_log_warn("gap: disarm: %s", strerror(errno));
        return -1;
    }
    return 0;
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

    dl_dbg_init(&cfg);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    int listen_fd = open_listen_socket(&cfg);
    if (listen_fd < 0) return 4;

    /* GS-side tunnel endpoint for PONG (and future debug back-channel
     * packets). Opened only when the debug suite's master switch is on;
     * a closed socket means PINGs are silently dropped, which is fine
     * — production never sees a PING in the first place. */
    int gs_tunnel_fd = -1;
    struct sockaddr_in gs_tunnel_dst = {0};
    if (cfg.debug_enable) {
        gs_tunnel_fd = open_gs_tunnel_socket(&cfg, &gs_tunnel_dst);
    }

    uint32_t tick_ms = cfg.osd_update_interval_ms;
    if (cfg.health_timeout_ms / 2 < tick_ms) tick_ms = cfg.health_timeout_ms / 2;
    if (tick_ms < 100) tick_ms = 100;
    int tick_fd = open_tick_timer(tick_ms);
    if (tick_fd < 0) { close(listen_fd); return 4; }

    int gap_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (gap_fd < 0) {
        dl_log_err("gap: timerfd_create: %s", strerror(errno));
        close(listen_fd); close(tick_fd);
        return 4;
    }

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
    dl_dedup_t dedup;
    dl_dedup_init(&dedup);
    int      last_rssi_dBm = 0;  /* Phase 2 will populate from MAVLink */

    apply_state_t apply_state   = APPLY_IDLE;
    dl_decision_t apply_pending = {0};

    struct pollfd pfds[3];
    pfds[0].fd = listen_fd; pfds[0].events = POLLIN;
    pfds[1].fd = tick_fd;   pfds[1].events = POLLIN;
    pfds[2].fd = gap_fd;    pfds[2].events = POLLIN;

    while (!g_stop) {
        int n = poll(pfds, 3, -1);
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
            /* T2 — captured as early as possible after the kernel handed
             * us the datagram. Used only for PING; cheap to take always. */
            uint64_t recv_us = now_monotonic_us();
            if (got < 0) {
                if (errno != EAGAIN && errno != EINTR) {
                    dl_log_warn("recvfrom: %s", strerror(errno));
                }
            } else if (dl_wire_peek_kind(buf, (size_t)got) == DL_PKT_PING) {
                dl_ping_t ping;
                if (dl_wire_decode_ping(buf, (size_t)got, &ping) != DL_DECODE_OK) {
                    dl_log_debug("ping: decode failed (%zd bytes from %s:%u)",
                                 got, inet_ntoa(src.sin_addr),
                                 ntohs(src.sin_port));
                } else if (gs_tunnel_fd >= 0) {
                    dl_pong_t pong = {
                        .flags = 0,
                        .gs_seq = ping.gs_seq,
                        .gs_mono_us_echo = ping.gs_mono_us,
                        .drone_mono_recv_us = recv_us,
                        .drone_mono_send_us = now_monotonic_us(),
                    };
                    uint8_t out[DL_PONG_ON_WIRE_SIZE];
                    size_t n_out = dl_wire_encode_pong(&pong, out, sizeof(out));
                    if (n_out > 0) {
                        ssize_t s = sendto(gs_tunnel_fd, out, n_out, 0,
                                           (struct sockaddr *)&gs_tunnel_dst,
                                           sizeof(gs_tunnel_dst));
                        if (s < 0) {
                            dl_log_debug("pong: sendto: %s", strerror(errno));
                        }
                    }
                }
            } else {
                dl_decision_t d;
                dl_decode_result_t dr = dl_wire_decode(buf, (size_t)got, &d);
                if (dr != DL_DECODE_OK) {
                    dl_log_warn("decode: result=%d from %s:%u (%zd bytes)",
                                (int)dr,
                                inet_ntoa(src.sin_addr), ntohs(src.sin_port),
                                got);
                    char detail[64];
                    snprintf(detail, sizeof(detail),
                             "{\"code\":%d,\"len\":%zd}", (int)dr, got);
                    dl_dbg_emit("DECODE_BAD", DL_DBG_SEV_WARN, detail);
                } else if (dl_dedup_check(&dedup, d.sequence)) {
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
                        char detail[96];
                        snprintf(detail, sizeof(detail),
                                 "{\"reason\":\"%s\",\"seq\":%u}",
                                 rsn, d.sequence);
                        dl_dbg_emit("CEILING_REJECT", DL_DBG_SEV_WARN, detail);
                    } else {
                        /* New decision supersedes any in-flight phase 2;
                         * the per-backend diff in phase-1 below will
                         * reapply anything that still differs. */
                        if (apply_state != APPLY_IDLE) {
                            disarm_gap(gap_fd);
                            apply_state = APPLY_IDLE;
                        }

                        uint64_t now = now_monotonic_ms();
                        bool first = (last_enc.magic != DL_WIRE_MAGIC);
                        dl_apply_dir_t dir = dl_apply_direction(
                            last_enc.bitrate_kbps, d.bitrate_kbps, first);

                        int drc = 0;
                        if (cfg.apply_stagger_ms == 0 ||
                            dir == DL_APPLY_DIR_EQUAL) {
                            if (bt && dl_backend_tx_apply(bt, &d, &last_tx) < 0) drc = -1;
                            if (br && dl_backend_radio_apply(br, &d, &last_radio) < 0) drc = -1;
                            if (be && dl_backend_enc_apply(be, &d, &last_enc) < 0) drc = -1;
                            if (d.flags & DL_FLAG_IDR_REQUEST) {
                                dl_backend_enc_request_idr(be, now);
                            }
                        } else if (dir == DL_APPLY_DIR_UP) {
                            /* Widen capacity first, then producer expands
                             * after the gap. */
                            if (bt && dl_backend_tx_apply(bt, &d, &last_tx) < 0) drc = -1;
                            if (br && dl_backend_radio_apply(br, &d, &last_radio) < 0) drc = -1;
                            apply_pending = d;
                            apply_state   = APPLY_UP_GAP;
                            arm_gap(gap_fd, cfg.apply_stagger_ms);
                        } else {  /* DL_APPLY_DIR_DOWN */
                            /* Throttle producer first, then narrow
                             * capacity after the gap. IDR rides with
                             * the encoder phase. */
                            if (be && dl_backend_enc_apply(be, &d, &last_enc) < 0) drc = -1;
                            if (d.flags & DL_FLAG_IDR_REQUEST) {
                                dl_backend_enc_request_idr(be, now);
                            }
                            apply_pending = d;
                            apply_state   = APPLY_DOWN_GAP;
                            arm_gap(gap_fd, cfg.apply_stagger_ms);
                        }

                        if (drc < 0) {
                            dl_log_warn("apply: at least one backend failed seq=%u",
                                        d.sequence);
                            dl_mavlink_emit(mav, "apply_fail",
                                            DL_MAV_SEV_WARNING,
                                            "DL APPLY_FAIL backend");
                            char detail[64];
                            snprintf(detail, sizeof(detail),
                                     "{\"seq\":%u}", d.sequence);
                            dl_dbg_emit("APPLY_FAIL", DL_DBG_SEV_WARN, detail);
                        } else {
                            dl_log_debug("apply: seq=%u mcs=%u k=%u n=%u d=%u tx=%d br=%u dir=%d",
                                         d.sequence, d.mcs, d.k, d.n, d.depth,
                                         d.tx_power_dBm, d.bitrate_kbps,
                                         (int)dir);
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
                /* Drop any queued phase 2 — safe values supersede. */
                if (apply_state != APPLY_IDLE) {
                    disarm_gap(gap_fd);
                    apply_state = APPLY_IDLE;
                }
                if (bt) dl_backend_tx_apply_safe(bt, &cfg);
                if (br) dl_backend_radio_apply_safe(br, &cfg);
                if (be) dl_backend_enc_apply_safe(be, &cfg);
                dl_osd_event_watchdog(osd);
                dl_mavlink_emit(mav, "watchdog", DL_MAV_SEV_ERROR,
                                "DL WATCHDOG safe_defaults");
                char detail[64];
                snprintf(detail, sizeof(detail),
                         "{\"timeout_ms\":%u}",
                         (unsigned)cfg.health_timeout_ms);
                dl_dbg_emit("WATCHDOG_TRIPPED", DL_DBG_SEV_ERROR, detail);
                /* Invalidate last-states so the next fresh decision emits
                 * everything. Reset dedup too so a GS that restarted with
                 * a lower seq baseline (or a wedged seq from a stray
                 * dl-inject) recovers without operator action. */
                memset(&last_tx, 0, sizeof(last_tx));
                memset(&last_radio, 0, sizeof(last_radio));
                memset(&last_enc, 0, sizeof(last_enc));
                dl_dedup_reset(&dedup);
            }
        }

        if (pfds[2].revents & POLLIN) {
            uint64_t expirations;
            ssize_t r = read(gap_fd, &expirations, sizeof(expirations));
            (void)r;  /* always drain to clear POLLIN */
            int drc = 0;
            if (apply_state == APPLY_UP_GAP) {
                if (be && dl_backend_enc_apply(be, &apply_pending, &last_enc) < 0)
                    drc = -1;
                if (apply_pending.flags & DL_FLAG_IDR_REQUEST) {
                    dl_backend_enc_request_idr(be, now_monotonic_ms());
                }
            } else if (apply_state == APPLY_DOWN_GAP) {
                if (bt && dl_backend_tx_apply(bt, &apply_pending, &last_tx) < 0)
                    drc = -1;
                if (br && dl_backend_radio_apply(br, &apply_pending, &last_radio) < 0)
                    drc = -1;
            }
            /* APPLY_IDLE here means a stale expiration the kernel had
             * already queued before disarm landed — drained, ignore. */
            if (apply_state != APPLY_IDLE && drc < 0) {
                dl_log_warn("apply: phase2 backend failed seq=%u",
                            apply_pending.sequence);
                dl_mavlink_emit(mav, "apply_fail",
                                DL_MAV_SEV_WARNING,
                                "DL APPLY_FAIL backend");
                char detail[64];
                snprintf(detail, sizeof(detail),
                         "{\"seq\":%u,\"phase\":2}", apply_pending.sequence);
                dl_dbg_emit("APPLY_FAIL", DL_DBG_SEV_WARN, detail);
            } else if (apply_state != APPLY_IDLE) {
                dl_log_debug("apply: phase2 seq=%u state=%d",
                             apply_pending.sequence, (int)apply_state);
            }
            apply_state = APPLY_IDLE;
        }
    }

    dl_log_info("dl-applier shutting down");
    close(listen_fd);
    close(tick_fd);
    close(gap_fd);
    if (gs_tunnel_fd >= 0) close(gs_tunnel_fd);
    dl_mavlink_close(mav);
    dl_osd_close(osd);
    dl_backend_enc_close(be);
    dl_backend_radio_close(br);
    dl_backend_tx_close(bt);
    dl_dbg_close();
    return 0;
}
