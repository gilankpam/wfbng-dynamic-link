/* test_dl_latency.c — unit tests for dl_latency. */
#include "dl_latency.h"
#include "test_main.h"

#include <string.h>

DL_TEST(latency_empty_renders_dashes) {
    dl_latency_init();
    char buf[1024];
    size_t n = dl_latency_render(buf, sizeof(buf));
    DL_ASSERT(n > 0);
    DL_ASSERT(n < sizeof(buf));
    DL_ASSERT(strstr(buf, "FEC  ") != NULL);
    DL_ASSERT(strstr(buf, "DPTH ") != NULL);
    DL_ASSERT(strstr(buf, "RADIO") != NULL);
    DL_ASSERT(strstr(buf, "TXPWR") != NULL);
    DL_ASSERT(strstr(buf, "ENC  ") != NULL);
    DL_ASSERT(strstr(buf, "IDR  ") != NULL);
    DL_ASSERT(strstr(buf, "p50= --") != NULL);
    DL_ASSERT(strstr(buf, "p95= --") != NULL);
    DL_ASSERT(strstr(buf, "mx= --") != NULL);
    DL_ASSERT(strstr(buf, "n=0") != NULL);
    DL_ASSERT(strstr(buf, "e=0") != NULL);
}

DL_TEST(latency_records_single_sample) {
    dl_latency_init();
    dl_latency_record_raw(DL_LAT_FEC, 7, 0);

    char buf[1024];
    dl_latency_render(buf, sizeof(buf));

    const char *line = strstr(buf, "FEC  ");
    DL_ASSERT(line != NULL);
    const char *eol = strchr(line, '\n');
    DL_ASSERT(eol != NULL);
    char one_line[256];
    size_t llen = (size_t)(eol - line);
    DL_ASSERT(llen < sizeof(one_line));
    memcpy(one_line, line, llen);
    one_line[llen] = '\0';

    DL_ASSERT(strstr(one_line, "p50=  7") != NULL);
    DL_ASSERT(strstr(one_line, "p95=  7") != NULL);
    DL_ASSERT(strstr(one_line, "mx=  7") != NULL);
    DL_ASSERT(strstr(one_line, "n=1")   != NULL);
    DL_ASSERT(strstr(one_line, "e=0")   != NULL);
}

DL_TEST(latency_records_error_counter) {
    dl_latency_init();
    dl_latency_record_raw(DL_LAT_FEC, 4, -1);
    dl_latency_record_raw(DL_LAT_FEC, 5,  0);
    dl_latency_record_raw(DL_LAT_FEC, 6, -1);

    char buf[1024];
    dl_latency_render(buf, sizeof(buf));
    const char *line = strstr(buf, "FEC  ");
    DL_ASSERT(line != NULL);
    DL_ASSERT(strstr(line, "n=3") != NULL);
    DL_ASSERT(strstr(line, "e=2") != NULL);
}

DL_TEST(latency_ring_wraps_at_128) {
    dl_latency_init();
    /* Push 200 distinct samples: 1..200. The last 128 (73..200) win. */
    for (uint32_t i = 1; i <= 200; ++i) {
        dl_latency_record_raw(DL_LAT_RADIO, i, 0);
    }
    char buf[1024];
    dl_latency_render(buf, sizeof(buf));
    const char *line = strstr(buf, "RADIO");
    DL_ASSERT(line != NULL);
    const char *eol = strchr(line, '\n');
    DL_ASSERT(eol != NULL);

    /* p50 over [73..200] = element at index 64 (sorted) = 73+64 = 137. */
    DL_ASSERT(strstr(line, "p50=137") != NULL);
    /* p95 over 128 sorted samples = index (128*95)/100 = 121, value 73+121 = 194. */
    DL_ASSERT(strstr(line, "p95=194") != NULL);
    /* mx sticky over all 200 = 200. */
    DL_ASSERT(strstr(line, "mx=200") != NULL);
    DL_ASSERT(strstr(line, "n=200") != NULL);
}

DL_TEST(latency_max_is_sticky_after_rollout) {
    dl_latency_init();
    /* One huge sample, then 128 tiny ones; the huge one rolls out of
     * the percentile window but max_ms stays. */
    dl_latency_record_raw(DL_LAT_ENC, 500, 0);
    for (int i = 0; i < 128; ++i) {
        dl_latency_record_raw(DL_LAT_ENC, 1, 0);
    }
    char buf[1024];
    dl_latency_render(buf, sizeof(buf));
    const char *line = strstr(buf, "ENC  ");
    DL_ASSERT(line != NULL);
    DL_ASSERT(strstr(line, "p50=  1") != NULL);
    DL_ASSERT(strstr(line, "p95=  1") != NULL);
    DL_ASSERT(strstr(line, "mx=500") != NULL);
}

DL_TEST(latency_clamps_giant_value) {
    dl_latency_init();
    dl_latency_record_raw(DL_LAT_TXPWR, 999999, 0);  /* > UINT16_MAX */
    char buf[1024];
    dl_latency_render(buf, sizeof(buf));
    const char *line = strstr(buf, "TXPWR");
    DL_ASSERT(line != NULL);
    /* UINT16_MAX = 65535. */
    DL_ASSERT(strstr(line, "p50=65535") != NULL);
    DL_ASSERT(strstr(line, "mx=65535") != NULL);
}

DL_TEST(latency_render_fits_buffer) {
    dl_latency_init();
    /* Worst case: large n_total / n_err on every slot. */
    for (int w = 0; w < DL_LAT__COUNT; ++w) {
        for (int i = 0; i < 1000; ++i) {
            dl_latency_record_raw((dl_lat_call_t)w, 123, (i & 1) ? -1 : 0);
        }
    }
    char buf[512];
    size_t n = dl_latency_render(buf, sizeof(buf));
    DL_ASSERT(n > 0);
    DL_ASSERT(n < sizeof(buf));
    DL_ASSERT(buf[n] == '\0');
}
