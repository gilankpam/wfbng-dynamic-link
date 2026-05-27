/* dl_applier.c — GS decision-packet dispatcher.
 *
 * Single-threaded poll(2) loop over:
 *   - UDP listen socket (decisions from GS over wfb-ng tunnel).
 *   - timerfd       (watchdog + OSD tick).
 *
 * On every accepted decision:
 *   parse_wire -> dispatch backends -> reset watchdog -> OSD update.
 *   Partial backend failure is logged but the loop continues.
 *
 * On watchdog trip:
 *   one-shot safe_defaults push; latch until the next fresh decision.
 */
#include "dl_apply.h"
#include "dl_backend_enc.h"
#include "dl_backend_radio.h"
#include "dl_backend_tx.h"
#include "dl_config.h"
#include "dl_dbg.h"
#include "dl_dedup.h"
#include "dl_hello.h"
#include "dl_idr_listen.h"
#include "dl_latency.h"
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

/* ---- CLI parsing -------------------------------------------------
 * Layered config: defaults → conf file (if --config) → CLI overrides.
 * Per-field flags are generated from the dl_config field tables; bit
 * arrays remember which fields the user passed so unset CLI flags
 * never clobber conf-file values.
 */
typedef struct {
    const char  *config_path;     /* may be NULL */
    bool         log_debug;
    dl_config_t  overrides;       /* only fields with set_*[i]=true are valid */
    /* one bit per index into DL_*_FIELDS tables */
    uint8_t      set_int [(64 + 7) / 8];   /* sized generously; assert below */
    uint8_t      set_bool[(16 + 7) / 8];
    uint8_t      set_str [(16 + 7) / 8];
} cli_args_t;

static inline void bit_set (uint8_t *a, size_t i) { a[i >> 3] |= (uint8_t)(1u << (i & 7)); }
static inline bool bit_test(const uint8_t *a, size_t i) { return (a[i >> 3] >> (i & 7)) & 1u; }

/* Long-option `val` encoding: high byte = category, low byte = index.
 * 0x10xx = int field, 0x20xx = bool field, 0x30xx = str field. */
enum {
    OPT_INT_BASE  = 0x1000,
    OPT_BOOL_BASE = 0x2000,
    OPT_STR_BASE  = 0x3000,
};

/* Convert a field name (snake_case) to a heap-allocated kebab-case
 * copy for use as a long-option name. Caller owns the storage; never
 * freed (lives until process exit). */
static char *xstrdup_kebab(const char *snake) {
    size_t n = strlen(snake);
    char *out = (char *)malloc(n + 1);
    if (!out) { perror("malloc"); exit(2); }
    for (size_t i = 0; i < n; i++) out[i] = (snake[i] == '_') ? '-' : snake[i];
    out[n] = '\0';
    return out;
}

static const char *type_label(dl_field_type_t t) {
    switch (t) {
    case DL_F_U8:  return "u8";
    case DL_F_I8:  return "i8";
    case DL_F_U16: return "u16";
    case DL_F_U32: return "u32";
    }
    return "?";
}

/* Print "foo-bar" form of `snake` to stderr (used by usage()). */
static void print_kebab(const char *snake) {
    for (const char *p = snake; *p; p++) {
        fputc(*p == '_' ? '-' : *p, stderr);
    }
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [--config <drone.conf>] [--debug] [field-overrides...]\n"
        "\n"
        "  --config PATH      path to drone.conf (optional; defaults used if omitted)\n"
        "  --debug            enable DEBUG-level logging\n"
        "  --help             print this help and exit\n"
        "\n"
        "Field overrides (CLI > conf file > defaults):\n",
        prog);

    size_t n;
    const dl_int_field_t *ti = dl_config_int_fields(&n);
    fprintf(stderr, "\n  Integer fields (--name VALUE):\n");
    for (size_t i = 0; i < n; i++) {
        fprintf(stderr, "    --");
        print_kebab(ti[i].name);
        fprintf(stderr, "  (%s, range [%ld..%ld])\n",
                type_label(ti[i].type), ti[i].lo, ti[i].hi);
    }

    const dl_bool_field_t *tb = dl_config_bool_fields(&n);
    fprintf(stderr, "\n  Boolean switches (set to true when passed):\n");
    for (size_t i = 0; i < n; i++) {
        fprintf(stderr, "    --");
        print_kebab(tb[i].name);
        fputc('\n', stderr);
    }

    const dl_str_field_t *ts = dl_config_str_fields(&n);
    fprintf(stderr, "\n  String fields (--name VALUE):\n");
    for (size_t i = 0; i < n; i++) {
        fprintf(stderr, "    --");
        print_kebab(ts[i].name);
        fputc('\n', stderr);
    }
    fprintf(stderr, "\n");
}

static int parse_args(int argc, char **argv, cli_args_t *out) {
    memset(out, 0, sizeof(*out));

    size_t n_int = 0, n_bool = 0, n_str = 0;
    const dl_int_field_t  *ti = dl_config_int_fields (&n_int);
    const dl_bool_field_t *tb = dl_config_bool_fields(&n_bool);
    const dl_str_field_t  *ts = dl_config_str_fields (&n_str);

    /* Bounds-check against the set_* bit arrays. */
    if (n_int  > sizeof(out->set_int)  * 8 ||
        n_bool > sizeof(out->set_bool) * 8 ||
        n_str  > sizeof(out->set_str)  * 8) {
        fprintf(stderr, "internal error: field table outgrew set_* bit arrays\n");
        return -1;
    }

    /* Build the option table: 3 fixed entries + per-field + terminator. */
    size_t total = 3 + n_int + n_bool + n_str + 1;
    struct option *opts = (struct option *)calloc(total, sizeof(struct option));
    if (!opts) { perror("calloc"); return -1; }

    size_t k = 0;
    opts[k++] = (struct option){ "config", required_argument, 0, 'c' };
    opts[k++] = (struct option){ "debug",  no_argument,       0, 'd' };
    opts[k++] = (struct option){ "help",   no_argument,       0, 'h' };
    for (size_t i = 0; i < n_int; i++)
        opts[k++] = (struct option){ xstrdup_kebab(ti[i].name), required_argument, 0, (int)(OPT_INT_BASE  + i) };
    for (size_t i = 0; i < n_bool; i++)
        opts[k++] = (struct option){ xstrdup_kebab(tb[i].name), no_argument,       0, (int)(OPT_BOOL_BASE + i) };
    for (size_t i = 0; i < n_str; i++)
        opts[k++] = (struct option){ xstrdup_kebab(ts[i].name), required_argument, 0, (int)(OPT_STR_BASE  + i) };
    opts[k] = (struct option){ 0 };

    /* Seed overrides with defaults so write_*_by_name has a valid
     * target type to copy into; only fields with set_*[i]=true are
     * later copied out. */
    dl_config_defaults(&out->overrides);

    int c;
    while ((c = getopt_long(argc, argv, "c:dh", opts, NULL)) != -1) {
        if (c == 'c')      out->config_path = optarg;
        else if (c == 'd') out->log_debug = true;
        else if (c == 'h') { usage(argv[0]); free(opts); return 1; }
        else if (c >= OPT_INT_BASE && c < OPT_INT_BASE + (int)n_int) {
            size_t i = (size_t)(c - OPT_INT_BASE);
            if (dl_config_set_int_by_name(&out->overrides, ti[i].name, optarg) != 0) {
                fprintf(stderr, "--");
                print_kebab(ti[i].name);
                fprintf(stderr, ": bad value %s (expected %s in [%ld..%ld])\n",
                        optarg, type_label(ti[i].type), ti[i].lo, ti[i].hi);
                free(opts); return -1;
            }
            bit_set(out->set_int, i);
        }
        else if (c >= OPT_BOOL_BASE && c < OPT_BOOL_BASE + (int)n_bool) {
            size_t i = (size_t)(c - OPT_BOOL_BASE);
            if (dl_config_set_bool_by_name(&out->overrides, tb[i].name, true) != 0) {
                fprintf(stderr, "--");
                print_kebab(tb[i].name);
                fprintf(stderr, ": internal error setting bool\n");
                free(opts); return -1;
            }
            bit_set(out->set_bool, i);
        }
        else if (c >= OPT_STR_BASE && c < OPT_STR_BASE + (int)n_str) {
            size_t i = (size_t)(c - OPT_STR_BASE);
            if (dl_config_set_str_by_name(&out->overrides, ts[i].name, optarg) != 0) {
                fprintf(stderr, "--");
                print_kebab(ts[i].name);
                fprintf(stderr, ": value rejected (too long? max %d)\n",
                        DL_CONF_MAX_STR - 1);
                free(opts); return -1;
            }
            bit_set(out->set_str, i);
        }
        else {
            usage(argv[0]);
            free(opts);
            return -1;
        }
    }
    free(opts);
    /* `out->config_path` may legitimately be NULL → caller skips load. */
    return 0;
}

/* Copy fields the user passed from `args->overrides` into `cfg`. */
static void apply_cli_overrides(dl_config_t *cfg, const cli_args_t *args) {
    size_t n;
    const dl_int_field_t  *ti = dl_config_int_fields (&n);
    for (size_t i = 0; i < n; i++) {
        if (!bit_test(args->set_int, i)) continue;
        void       *dst = (char *)cfg + ti[i].offset;
        const void *src = (const char *)&args->overrides + ti[i].offset;
        switch (ti[i].type) {
        case DL_F_U8:  *(uint8_t  *)dst = *(const uint8_t  *)src; break;
        case DL_F_I8:  *(int8_t   *)dst = *(const int8_t   *)src; break;
        case DL_F_U16: *(uint16_t *)dst = *(const uint16_t *)src; break;
        case DL_F_U32: *(uint32_t *)dst = *(const uint32_t *)src; break;
        }
    }
    const dl_bool_field_t *tb = dl_config_bool_fields(&n);
    for (size_t i = 0; i < n; i++) {
        if (!bit_test(args->set_bool, i)) continue;
        bool       *dst = (bool *)((char *)cfg + tb[i].offset);
        const bool *src = (const bool *)((const char *)&args->overrides + tb[i].offset);
        *dst = *src;
    }
    const dl_str_field_t  *ts = dl_config_str_fields (&n);
    for (size_t i = 0; i < n; i++) {
        if (!bit_test(args->set_str, i)) continue;
        char       *dst = (char *)cfg + ts[i].offset;
        const char *src = (const char *)&args->overrides + ts[i].offset;
        snprintf(dst, DL_CONF_MAX_STR, "%s", src);
    }
}

int main(int argc, char **argv) {
    cli_args_t args;
    int pa = parse_args(argc, argv, &args);
    if (pa != 0) return pa < 0 ? 2 : 0;

    dl_log_init("dl-applier", args.log_debug ? DL_LOG_DEBUG : DL_LOG_INFO);
    dl_log_info("dynamic-link applier starting");

    dl_config_t cfg;
    dl_config_defaults(&cfg);
    if (args.config_path && dl_config_load(args.config_path, &cfg) < 0) {
        dl_log_fatal("config load failed");
        return 3;
    }
    apply_cli_overrides(&cfg, &args);
    if (dl_config_validate(&cfg) != 0) {
        dl_log_fatal("config validation failed");
        return 3;
    }

    dl_dbg_init(&cfg);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    int listen_fd = open_listen_socket(&cfg);
    if (listen_fd < 0) return 4;

    /* GS-side tunnel endpoint for drone→GS traffic. Originally opened
     * only under cfg.debug_enable for PONG replies to debug PINGs;
     * P4a's HELLO handshake (DLHE) also needs this socket in
     * production, so it is now opened unconditionally. The send path
     * tolerates a routing failure (uses sendto per packet, no
     * connect), and a -1 fd just silently drops outbound traffic. */
    int gs_tunnel_fd = -1;
    struct sockaddr_in gs_tunnel_dst = {0};
    gs_tunnel_fd = open_gs_tunnel_socket(&cfg, &gs_tunnel_dst);

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

    dl_idr_listen_t *idr_listen = dl_idr_listen_open(cfg.idr_listen_addr,
                                                     cfg.idr_listen_port);

    dl_latency_init();
    dl_backend_tx_t    *bt = dl_backend_tx_open(&cfg);
    dl_backend_radio_t *br = dl_backend_radio_open(&cfg);
    dl_backend_enc_t   *be = dl_backend_enc_open(&cfg);
    dl_osd_t           *osd = dl_osd_open(&cfg);
    dl_mavlink_t       *mav = dl_mavlink_open(&cfg);

    if (!bt) dl_log_warn("tx backend not available; FEC/DEPTH/RADIO drops");

    dl_watchdog_t wd;
    dl_watchdog_init(&wd, cfg.health_timeout_ms);

    dl_hello_sm_t hello;
    int hello_inited = dl_hello_init(&hello, &cfg);
    int hello_timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (hello_timer_fd < 0) {
        dl_log_err("hello: timerfd_create: %s", strerror(errno));
    } else if (hello_inited == 0) {
        /* Arm immediately so the first DLHE goes out ASAP. */
        struct itimerspec t = {0};
        t.it_value.tv_nsec = 1 * 1000 * 1000;  /* 1 ms */
        timerfd_settime(hello_timer_fd, 0, &t, NULL);
    }

    /* One prev-state per backend — each tracks only the knobs it
     * actually applies, and mutating one must not fool the others. */
    dl_decision_t last_tx = {0};
    dl_decision_t last_radio = {0};
    /* last_enc is no longer maintained by the encoder backend (it owns
     * its own per-knob prev now). We still track it here purely to
     * feed dl_apply_direction's UP/DOWN/EQUAL classification, which
     * keys off bitrate. */
    dl_decision_t last_enc = {0};
    dl_decision_t last_applied = {0};  /* for OSD display only */
    dl_dedup_t dedup;
    dl_dedup_init(&dedup);
    int      last_rssi_dBm = 0;  /* Phase 2 will populate from MAVLink */

    apply_state_t apply_state   = APPLY_IDLE;
    dl_decision_t apply_pending = {0};

    struct pollfd pfds[5];
    pfds[0].fd = listen_fd;      pfds[0].events = POLLIN;
    pfds[1].fd = tick_fd;        pfds[1].events = POLLIN;
    pfds[2].fd = gap_fd;         pfds[2].events = POLLIN;
    pfds[3].fd = hello_timer_fd; pfds[3].events = POLLIN;
    pfds[4].fd = dl_idr_listen_fd(idr_listen);
    pfds[4].events = POLLIN;

    while (!g_stop) {
        int n = poll(pfds, 5, -1);
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
            dl_packet_kind_t kind = (got >= 0)
                ? dl_wire_peek_kind(buf, (size_t)got)
                : DL_PKT_UNKNOWN;
            if (got < 0) {
                if (errno != EAGAIN && errno != EINTR) {
                    dl_log_warn("recvfrom: %s", strerror(errno));
                }
            } else if (kind == DL_PKT_PING) {
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
            } else if (kind == DL_PKT_HELLO_ACK) {
                dl_hello_ack_t ack;
                if (dl_wire_decode_hello_ack(buf, (size_t)got, &ack) == DL_DECODE_OK) {
                    dl_hello_on_ack(&hello, &ack);
                } else {
                    dl_log_debug("hello_ack: decode failed (%zd bytes from %s:%u)",
                                 got, inet_ntoa(src.sin_addr),
                                 ntohs(src.sin_port));
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
                    useconds_t sub_pace_us =
                        (useconds_t)cfg.apply_sub_pace_ms * 1000u;
                    if (cfg.apply_stagger_ms == 0 ||
                        dir == DL_APPLY_DIR_EQUAL) {
                        if (bt && dl_backend_tx_apply(bt, &d, &last_tx) < 0) drc = -1;
                        if (br && dl_backend_radio_apply(br, &d, &last_radio) < 0) drc = -1;
                        if (be && dl_backend_enc_apply(be, &d) < 0) drc = -1;
                        last_enc = d;
                    } else if (dir == DL_APPLY_DIR_UP) {
                        /* Power up BEFORE MCS up so the new (higher)
                         * MCS rate has the headroom on the very first
                         * frame transmitted at it. Then tx (FEC/depth/
                         * MCS) — paced from the iw exit so wfb_tx's
                         * radiotap update lands on a steady-state
                         * power level. Encoder bitrate expands after
                         * the outer gap. */
                        if (br && dl_backend_radio_apply(br, &d, &last_radio) < 0) drc = -1;
                        if (sub_pace_us > 0) usleep(sub_pace_us);
                        if (bt && dl_backend_tx_apply(bt, &d, &last_tx) < 0) drc = -1;
                        apply_pending = d;
                        apply_state   = APPLY_UP_GAP;
                        arm_gap(gap_fd, cfg.apply_stagger_ms);
                    } else {  /* DL_APPLY_DIR_DOWN */
                        /* Throttle producer first, then narrow
                         * capacity after the gap. IDR rides with
                         * the encoder phase. tx (MCS down) before
                         * radio (power down) so we don't transmit
                         * the old high MCS at reduced power. */
                        if (be && dl_backend_enc_apply(be, &d) < 0) drc = -1;
                        last_enc = d;
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
                if (be && dl_backend_enc_apply(be, &apply_pending) < 0)
                    drc = -1;
                last_enc = apply_pending;
            } else if (apply_state == APPLY_DOWN_GAP) {
                if (bt && dl_backend_tx_apply(bt, &apply_pending, &last_tx) < 0)
                    drc = -1;
                if (cfg.apply_sub_pace_ms > 0) {
                    usleep((useconds_t)cfg.apply_sub_pace_ms * 1000u);
                }
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

        if (pfds[3].revents & POLLIN) {
            uint64_t expirations;
            ssize_t r = read(hello_timer_fd, &expirations, sizeof(expirations));
            (void)r;
            if (hello.state == DL_HELLO_STATE_KEEPALIVE) {
                /* Drop-back to ANNOUNCING is automatic on too many
                 * missed acks; either way we send a HELLO now. */
                dl_hello_on_keepalive_tick(&hello);
            }
            if (hello.state != DL_HELLO_STATE_DISABLED && gs_tunnel_fd >= 0) {
                uint8_t out[DL_HELLO_ON_WIRE_SIZE];
                size_t n_out = dl_hello_build_announce(&hello, out, sizeof(out));
                if (n_out > 0) {
                    ssize_t s = sendto(gs_tunnel_fd, out, n_out, 0,
                                       (struct sockaddr *)&gs_tunnel_dst,
                                       sizeof(gs_tunnel_dst));
                    if (s < 0) {
                        dl_log_debug("hello: sendto: %s", strerror(errno));
                    }
                }
            }
            /* Re-arm. timerfd_settime with an all-zero it_value disarms
             * the timer, so coerce a 0-ms delay to 1 ms when we still
             * want to fire. DISABLED leaves it_value zero -> timer
             * stays parked. */
            uint32_t delay_ms = dl_hello_next_delay_ms(&hello);
            struct itimerspec t = {0};
            t.it_value.tv_sec = delay_ms / 1000;
            t.it_value.tv_nsec = (long)(delay_ms % 1000) * 1000000L;
            if (delay_ms == 0 && hello.state != DL_HELLO_STATE_DISABLED) {
                t.it_value.tv_nsec = 1 * 1000 * 1000;
            }
            timerfd_settime(hello_timer_fd, 0, &t, NULL);
        }

        if (pfds[4].revents & POLLIN) {
            size_t got = dl_idr_listen_drain(idr_listen);
            if (got > 0) {
                dl_log_debug("idr_listen: drained %zu datagram(s)", got);
                dl_osd_bump_idr(osd);
                dl_backend_enc_request_idr(be, now_monotonic_ms());
            }
        }
    }

    dl_log_info("dl-applier shutting down");
    close(listen_fd);
    close(tick_fd);
    close(gap_fd);
    if (gs_tunnel_fd >= 0) close(gs_tunnel_fd);
    if (hello_timer_fd >= 0) close(hello_timer_fd);
    dl_idr_listen_close(idr_listen);
    dl_mavlink_close(mav);
    dl_osd_close(osd);
    dl_backend_enc_close(be);
    dl_backend_radio_close(br);
    dl_backend_tx_close(bt);
    dl_dbg_close();
    return 0;
}
