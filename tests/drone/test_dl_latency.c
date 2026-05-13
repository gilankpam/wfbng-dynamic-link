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
