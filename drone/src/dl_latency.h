/* dl_latency.h — per-apply-call latency stats for the debug OSD.
 *
 * Process-singleton ring buffers (N=128) per tracked call site, plus
 * cumulative call count, error count and sticky max. Single-threaded.
 * Always records; the OSD writer decides whether to render based on
 * cfg->osd_debug_latency.
 *
 * See docs/superpowers/specs/2026-05-13-debug-osd-apply-latency-design.md.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum {
    DL_LAT_FEC = 0,
    DL_LAT_DPTH,
    DL_LAT_RADIO,
    DL_LAT_TXPWR,
    DL_LAT_ENC,
    DL_LAT_IDR,
    DL_LAT__COUNT,
} dl_lat_call_t;

/* Opaque; carries the start timestamp and the call id so `end` can't
 * accidentally desync from `begin`. */
typedef struct {
    uint64_t      t_ns;
    dl_lat_call_t which;
} dl_lat_handle_t;

void dl_latency_init(void);

dl_lat_handle_t dl_latency_begin(dl_lat_call_t which);
void dl_latency_end(dl_lat_handle_t h, int rc);

/* Renders all six lines into `out`, including msposd `&L<row>&F30 `
 * prefixes and trailing newlines. Returns bytes written (excluding
 * trailing NUL). */
size_t dl_latency_render(char *out, size_t out_len);

#ifdef DL_TESTING
/* Test-only direct sample injection. Bypasses clock_gettime so unit
 * tests can drive the module deterministically. `ms_u64` is clamped
 * to UINT16_MAX before storage. */
void dl_latency_record_raw(dl_lat_call_t which, uint64_t ms_u64, int rc);
#endif
