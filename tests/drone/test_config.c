/* test_config.c — drone.conf parser + validator. */
#include "test_main.h"
#include "dl_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Write `body` to a temp file, return its path (stored in out). */
static int write_tmp(const char *body, char *out, size_t outlen) {
    static int counter = 0;
    snprintf(out, outlen, "/tmp/dltest_%d_%d.conf", getpid(), counter++);
    FILE *fd = fopen(out, "w");
    if (!fd) return -1;
    fputs(body, fd);
    fclose(fd);
    return 0;
}

DL_TEST(test_config_defaults_are_sane) {
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_STR_EQ(c.listen_addr, "0.0.0.0");
    DL_ASSERT_EQ(c.listen_port, 5800);
    DL_ASSERT_EQ(c.depth_max, 3);
    DL_ASSERT_EQ(c.video_k_max, 8);
    DL_ASSERT_EQ(c.health_timeout_ms, 10000);
    DL_ASSERT(dl_config_validate(&c) == 0);
}

DL_TEST(test_config_parses_good_file) {
    const char *body =
        "# sample\n"
        "listen_addr = 127.0.0.1\n"
        "listen_port = 5900\n"
        "wfb_tx_ctrl_port = 8010\n"
        "video_k_min = 2\n"
        "video_k_max = 8\n"
        "video_n_max = 16\n"
        "depth_max = 3\n"
        "mcs_max = 7\n"
        "tx_power_max_dBm = 20\n"
        "encoder_kind = waybeam\n"
        "encoder_port = 8080\n"
        "osd_enable = false\n";
    char path[64];
    DL_ASSERT_EQ(write_tmp(body, path, sizeof(path)), 0);

    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_load(path, &c), 0);
    DL_ASSERT_STR_EQ(c.listen_addr, "127.0.0.1");
    DL_ASSERT_EQ(c.listen_port, 5900);
    DL_ASSERT_EQ(c.wfb_tx_ctrl_port, 8010);
    DL_ASSERT_STR_EQ(c.encoder_kind, "waybeam");
    DL_ASSERT_EQ(c.encoder_port, 8080);
    DL_ASSERT(!c.osd_enable);
    DL_ASSERT_EQ(dl_config_validate(&c), 0);

    unlink(path);
}

DL_TEST(test_config_rejects_depth_above_hw_limit) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.depth_max = 9;  /* > MAX_INTERLEAVE_DEPTH=8 */
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_rejects_depth2_with_n_over_32) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.depth_max = 2;
    c.video_n_max = 33;
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_rejects_safe_k_above_ceiling) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.video_k_max = 4;
    c.safe_k = 8;
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_rejects_safe_tx_power_above_ceiling) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.tx_power_max_dBm = 20;
    c.safe_tx_power_dBm = 30;
    DL_ASSERT(dl_config_validate(&c) < 0);
}

DL_TEST(test_config_missing_file_returns_error) {
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_load("/tmp/does-not-exist-dl", &c), -1);
}

DL_TEST(test_config_bad_value_is_error) {
    const char *body = "listen_port = not-a-number\n";
    char path[64];
    DL_ASSERT_EQ(write_tmp(body, path, sizeof(path)), 0);
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_load(path, &c), -1);
    unlink(path);
}
