/* dl_latency.c — see dl_latency.h. */
#include "dl_latency.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DL_LAT_WINDOW 128

struct dl_lat_slot {
    uint16_t samples_ms[DL_LAT_WINDOW];
    uint8_t  head;
    uint8_t  filled;
    uint64_t n_total;
    uint64_t n_err;
    uint16_t max_ms;
};

static struct dl_lat_slot g_slots[DL_LAT__COUNT];

static const char *tag_of(dl_lat_call_t w) {
    switch (w) {
    case DL_LAT_FEC:   return "FEC  ";
    case DL_LAT_DPTH:  return "DPTH ";
    case DL_LAT_RADIO: return "RADIO";
    case DL_LAT_TXPWR: return "TXPWR";
    case DL_LAT_ENC:   return "ENC  ";
    case DL_LAT_IDR:   return "IDR  ";
    case DL_LAT__COUNT: break;
    }
    return "?????";
}

void dl_latency_init(void) {
    memset(g_slots, 0, sizeof(g_slots));
}

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

dl_lat_handle_t dl_latency_begin(dl_lat_call_t which) {
    dl_lat_handle_t h = { .t_ns = now_ns(), .which = which };
    return h;
}

static void record_clamped(dl_lat_call_t which, uint64_t ms_u64, int rc) {
    if ((unsigned)which >= DL_LAT__COUNT) return;
    struct dl_lat_slot *s = &g_slots[which];
    uint16_t ms = (ms_u64 > UINT16_MAX) ? UINT16_MAX : (uint16_t)ms_u64;
    s->samples_ms[s->head] = ms;
    s->head = (uint8_t)((s->head + 1) % DL_LAT_WINDOW);
    if (s->filled < DL_LAT_WINDOW) s->filled++;
    s->n_total++;
    if (rc != 0) s->n_err++;
    if (ms > s->max_ms) s->max_ms = ms;
}

void dl_latency_end(dl_lat_handle_t h, int rc) {
    uint64_t end = now_ns();
    uint64_t ms = (end - h.t_ns + 500000ull) / 1000000ull;
    record_clamped(h.which, ms, rc);
}

#ifdef DL_TESTING
void dl_latency_record_raw(dl_lat_call_t which, uint64_t ms_u64, int rc) {
    record_clamped(which, ms_u64, rc);
}
#endif

static int cmp_u16(const void *a, const void *b) {
    uint16_t x = *(const uint16_t *)a;
    uint16_t y = *(const uint16_t *)b;
    return (x > y) - (x < y);
}

static void compute_percentiles(const struct dl_lat_slot *s,
                                uint16_t *p50, uint16_t *p95) {
    uint16_t tmp[DL_LAT_WINDOW];
    memcpy(tmp, s->samples_ms, sizeof(uint16_t) * s->filled);
    qsort(tmp, s->filled, sizeof(uint16_t), cmp_u16);
    *p50 = tmp[s->filled / 2];
    *p95 = tmp[(s->filled * 95) / 100];
}

/* msposd directive: `&Lxx` is color*10 + position-zone (zones:
 * 0=TopLeft, 1=TopCenter, 2=TopRight, 3=TopMoving, 4=BottomLeft,
 * 5=BottomCenter, 6=BottomRight, 7=BottomMoving). `&L50` = yellow +
 * TopLeft, matching the existing status line so debug lines stack
 * directly under it. Earlier code treated the second digit as a
 * row number, which scattered the lines across all eight zones. */
#define DL_LATENCY_PREFIX "&L50&F30 "

size_t dl_latency_render(char *out, size_t out_len) {
    size_t off = 0;
    for (int i = 0; i < DL_LAT__COUNT; ++i) {
        const struct dl_lat_slot *s = &g_slots[i];
        const char *tag = tag_of((dl_lat_call_t)i);
        int n;
        if (s->filled == 0) {
            n = snprintf(out + off, out_len - off,
                         DL_LATENCY_PREFIX
                         "%s p50= -- p95= -- mx= -- n=%llu e=%llu\n",
                         tag,
                         (unsigned long long)s->n_total,
                         (unsigned long long)s->n_err);
        } else {
            uint16_t p50 = 0, p95 = 0;
            compute_percentiles(s, &p50, &p95);
            n = snprintf(out + off, out_len - off,
                         DL_LATENCY_PREFIX
                         "%s p50=%3u p95=%3u mx=%3u n=%llu e=%llu\n",
                         tag,
                         (unsigned)p50, (unsigned)p95, (unsigned)s->max_ms,
                         (unsigned long long)s->n_total,
                         (unsigned long long)s->n_err);
        }
        if (n < 0 || (size_t)n >= out_len - off) {
            if (off < out_len) out[off] = '\0';
            return off;
        }
        off += (size_t)n;
    }
    return off;
}
