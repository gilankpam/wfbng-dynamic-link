/* dl_inject.c — craft one decision packet and send it via UDP.
 *
 * Used by Phase 1 operators and by tests to drive dl-applier without
 * the GS (Phase 2 wires the live sender).
 */
#include "dl_wire.h"

#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s --target HOST:PORT \\\n"
        "         --mcs N --bandwidth {20|40} --tx-power DBM \\\n"
        "         --k N --n N --depth N \\\n"
        "         --bitrate KBPS [--roi-qp QP] [--fps FPS] \\\n"
        "         [--idr] [--sequence N]\n"
        "       %s --ping --gs-seq N --gs-mono US [--target HOST:PORT]\n"
        "       %s --hello --gen-id N --mtu N --fps N [--build-sha N] --dry-run\n"
        "       %s --hello-ack --gen-id N --dry-run\n"
        "       %s --dry-run ... (prints hex bytes to stdout; no send)\n",
        prog, prog, prog, prog, prog);
}

static int parse_host_port(const char *s, char *host, size_t hostlen,
                           uint16_t *port) {
    const char *colon = strrchr(s, ':');
    if (!colon) return -1;
    size_t hl = (size_t)(colon - s);
    if (hl == 0 || hl >= hostlen) return -1;
    memcpy(host, s, hl);
    host[hl] = '\0';
    long p = strtol(colon + 1, NULL, 10);
    if (p <= 0 || p > 65535) return -1;
    *port = (uint16_t)p;
    return 0;
}

int main(int argc, char **argv) {
    static struct option opts[] = {
        { "target",    required_argument, 0, 't' },
        { "mcs",       required_argument, 0, 'M' },
        { "bandwidth", required_argument, 0, 'B' },
        { "tx-power",  required_argument, 0, 'P' },
        { "k",         required_argument, 0, 'k' },
        { "n",         required_argument, 0, 'n' },
        { "depth",     required_argument, 0, 'd' },
        { "bitrate",   required_argument, 0, 'b' },
        { "roi-qp",    required_argument, 0, 'r' },
        { "fps",       required_argument, 0, 'f' },
        { "idr",       no_argument,       0, 'I' },
        { "sequence",  required_argument, 0, 's' },
        { "dry-run",   no_argument,       0, 'D' },
        { "ping",      no_argument,       0, 'p' },
        { "gs-seq",    required_argument, 0, 'q' },
        { "gs-mono",   required_argument, 0, 'm' },
        { "hello",     no_argument,       0, 'H' },
        { "hello-ack", no_argument,       0, 'A' },
        { "gen-id",    required_argument, 0, 'g' },
        { "mtu",       required_argument, 0, 'u' },
        { "build-sha", required_argument, 0, 'S' },
        { "help",      no_argument,       0, 'h' },
        { 0 }
    };

    const char *target = NULL;
    bool dry_run = false;
    bool ping_mode = false;
    uint32_t ping_gs_seq = 0;
    uint64_t ping_gs_mono_us = 0;
    dl_decision_t d = {
        .magic = DL_WIRE_MAGIC,
        .version = DL_WIRE_VERSION,
        .mcs = 7,
        .bandwidth = 20,
        .tx_power_dBm = 20,
        .k = 8,
        .n = 12,
        .depth = 1,
        .bitrate_kbps = 8000,
        .roi_qp = 0,
        .fps = 0,
        .flags = 0,
    };
    uint32_t explicit_seq = 0;
    bool have_seq = false;
    bool hello_mode = false;
    bool hello_ack_mode = false;
    dl_hello_t hello = { .version = DL_WIRE_VERSION };
    dl_hello_ack_t hello_ack = { .version = DL_WIRE_VERSION };

    int c;
    while ((c = getopt_long(argc, argv, "t:M:B:P:k:n:d:b:r:f:Is:Dpq:m:HAg:u:S:h",
                            opts, NULL)) != -1) {
        switch (c) {
            case 't': target = optarg; break;
            case 'M': d.mcs = (uint8_t)atoi(optarg); break;
            case 'B': d.bandwidth = (uint8_t)atoi(optarg); break;
            case 'P': d.tx_power_dBm = (int8_t)atoi(optarg); break;
            case 'k': d.k = (uint8_t)atoi(optarg); break;
            case 'n': d.n = (uint8_t)atoi(optarg); break;
            case 'd': d.depth = (uint8_t)atoi(optarg); break;
            case 'b': d.bitrate_kbps = (uint16_t)atoi(optarg); break;
            case 'r': d.roi_qp = (uint8_t)atoi(optarg); break;
            case 'f': d.fps = (uint8_t)atoi(optarg);
                      hello.fps = (uint16_t)atoi(optarg); break;
            case 'I': d.flags |= DL_FLAG_IDR_REQUEST; break;
            case 's': explicit_seq = (uint32_t)strtoul(optarg, NULL, 10);
                      have_seq = true; break;
            case 'D': dry_run = true; break;
            case 'p': ping_mode = true; break;
            case 'q': ping_gs_seq = (uint32_t)strtoul(optarg, NULL, 10); break;
            case 'm': ping_gs_mono_us = strtoull(optarg, NULL, 10); break;
            case 'H': hello_mode = true; break;
            case 'A': hello_ack_mode = true; break;
            case 'g': {
                unsigned long v = strtoul(optarg, NULL, 0);
                hello.generation_id = (uint32_t)v;
                hello_ack.generation_id_echo = (uint32_t)v;
                break;
            }
            case 'u': hello.mtu_bytes = (uint16_t)atoi(optarg); break;
            case 'S': {
                unsigned long v = strtoul(optarg, NULL, 0);
                hello.applier_build_sha = (uint32_t)v;
                break;
            }
            case 'h': usage(argv[0]); return 0;
            default:  usage(argv[0]); return 2;
        }
    }
    if (!target && !dry_run) { usage(argv[0]); return 2; }

    if (hello_mode) {
        uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
        size_t n = dl_wire_encode_hello(&hello, buf, sizeof(buf));
        if (n == 0) { fprintf(stderr, "encode failed\n"); return 5; }
        if (dry_run) {
            for (size_t i = 0; i < n; ++i) printf("%02x", buf[i]);
            printf("\n");
            return 0;
        }
        fprintf(stderr, "hello: live-send not implemented; use --dry-run\n");
        return 6;
    }
    if (hello_ack_mode) {
        uint8_t buf[DL_HELLO_ACK_ON_WIRE_SIZE];
        size_t n = dl_wire_encode_hello_ack(&hello_ack, buf, sizeof(buf));
        if (n == 0) { fprintf(stderr, "encode failed\n"); return 5; }
        if (dry_run) {
            for (size_t i = 0; i < n; ++i) printf("%02x", buf[i]);
            printf("\n");
            return 0;
        }
        fprintf(stderr, "hello-ack: live-send not implemented; use --dry-run\n");
        return 6;
    }

    if (ping_mode) {
        dl_ping_t p = {
            .flags = 0,
            .gs_seq = ping_gs_seq,
            .gs_mono_us = ping_gs_mono_us,
        };
        uint8_t pbuf[DL_PING_ON_WIRE_SIZE];
        size_t pn = dl_wire_encode_ping(&p, pbuf, sizeof(pbuf));
        if (pn != DL_PING_ON_WIRE_SIZE) {
            fprintf(stderr, "ping encode failed\n");
            return 3;
        }
        if (dry_run) {
            for (size_t i = 0; i < pn; ++i) printf("%02x", pbuf[i]);
            printf("\n");
            return 0;
        }
        char host[128] = {0};
        uint16_t port = 0;
        if (parse_host_port(target, host, sizeof(host), &port) != 0) {
            fprintf(stderr, "bad --target %s\n", target);
            return 2;
        }
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) { perror("socket"); return 4; }
        struct sockaddr_in dst = {0};
        dst.sin_family = AF_INET;
        dst.sin_port = htons(port);
        if (inet_pton(AF_INET, host, &dst.sin_addr) != 1) {
            fprintf(stderr, "bad host: %s\n", host);
            return 4;
        }
        ssize_t s = sendto(fd, pbuf, pn, 0,
                           (struct sockaddr *)&dst, sizeof(dst));
        if (s != (ssize_t)pn) { perror("sendto"); close(fd); return 5; }
        close(fd);
        fprintf(stderr, "sent ping seq=%u gs_mono_us=%llu -> %s:%u\n",
                ping_gs_seq, (unsigned long long)ping_gs_mono_us, host, port);
        return 0;
    }

    char host[128] = {0};
    uint16_t port = 0;
    if (target) {
        if (parse_host_port(target, host, sizeof(host), &port) != 0) {
            fprintf(stderr, "bad --target %s (want HOST:PORT)\n", target);
            return 2;
        }
    }

    if (have_seq) {
        d.sequence = explicit_seq;
    } else {
        /* Time-based default so independent invocations don't collide. */
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        d.sequence = (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
    }
    d.timestamp_ms = d.sequence;

    uint8_t buf[DL_WIRE_ON_WIRE_SIZE];
    size_t nbytes = dl_wire_encode(&d, buf, sizeof(buf));
    if (nbytes != DL_WIRE_ON_WIRE_SIZE) {
        fprintf(stderr, "wire_encode failed\n");
        return 3;
    }

    if (dry_run) {
        /* Emit the encoded bytes as lowercase hex to stdout, no spaces,
         * terminated by a newline. Used by the GS-side contract test. */
        for (size_t i = 0; i < nbytes; ++i) {
            printf("%02x", buf[i]);
        }
        printf("\n");
        return 0;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 4; }

    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &dst.sin_addr) != 1) {
        fprintf(stderr, "bad host: %s\n", host);
        return 4;
    }
    ssize_t sent = sendto(fd, buf, nbytes, 0,
                          (struct sockaddr *)&dst, sizeof(dst));
    if (sent != (ssize_t)nbytes) {
        perror("sendto");
        close(fd);
        return 5;
    }
    close(fd);
    fprintf(stderr,
            "sent seq=%u mcs=%u bw=%u tx=%d k=%u n=%u depth=%u "
            "bitrate=%u roi_qp=%u fps=%u%s -> %s:%u\n",
            d.sequence, d.mcs, d.bandwidth, d.tx_power_dBm,
            d.k, d.n, d.depth, d.bitrate_kbps, d.roi_qp, d.fps,
            (d.flags & DL_FLAG_IDR_REQUEST) ? " IDR" : "",
            host, port);
    return 0;
}
