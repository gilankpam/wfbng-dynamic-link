/* test_ceiling.c — failsafe 4 rejection logic. */
#include "test_main.h"
#include "dl_ceiling.h"
#include "dl_config.h"

static dl_config_t default_cfg(void) {
    dl_config_t c;
    dl_config_defaults(&c);
    return c;
}

static dl_decision_t default_dec(void) {
    dl_decision_t d = {
        .mcs = 5, .bandwidth = 20, .tx_power_dBm = 15,
        .k = 8, .n = 12, .depth = 1, .bitrate_kbps = 10000,
    };
    return d;
}

DL_TEST(test_ceiling_accepts_default) {
    dl_config_t c = default_cfg();
    dl_decision_t d = default_dec();
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_OK);
}

DL_TEST(test_ceiling_rejects_k_below_min) {
    dl_config_t c = default_cfg();
    dl_decision_t d = default_dec();
    d.k = 1;
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_K_OUT_OF_RANGE);
}

DL_TEST(test_ceiling_rejects_k_above_max) {
    dl_config_t c = default_cfg();
    dl_decision_t d = default_dec();
    d.k = c.video_k_max + 1;
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_K_OUT_OF_RANGE);
}

DL_TEST(test_ceiling_rejects_n_above_max) {
    dl_config_t c = default_cfg();
    dl_decision_t d = default_dec();
    d.n = c.video_n_max + 1;
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_N_TOO_LARGE);
}

DL_TEST(test_ceiling_rejects_depth_above_max) {
    dl_config_t c = default_cfg();
    dl_decision_t d = default_dec();
    d.depth = c.depth_max + 1;
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_DEPTH_TOO_LARGE);
}

DL_TEST(test_ceiling_rejects_mcs_above_cap) {
    dl_config_t c = default_cfg();
    c.mcs_max = 5;
    dl_decision_t d = default_dec();
    d.mcs = 6;
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_MCS_TOO_HIGH);
}

DL_TEST(test_ceiling_rejects_bandwidth_80) {
    dl_config_t c = default_cfg();
    dl_decision_t d = default_dec();
    d.bandwidth = 80;
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_BANDWIDTH_BAD);
}

DL_TEST(test_ceiling_rejects_tx_power_too_high) {
    dl_config_t c = default_cfg();
    c.tx_power_max_dBm = 20;
    dl_decision_t d = default_dec();
    d.tx_power_dBm = 21;
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_TX_POWER_OUT_OF_RANGE);
}

DL_TEST(test_ceiling_rejects_tx_power_too_low) {
    dl_config_t c = default_cfg();
    c.tx_power_min_dBm = 5;
    dl_decision_t d = default_dec();
    d.tx_power_dBm = 4;
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_TX_POWER_OUT_OF_RANGE);
}

DL_TEST(test_ceiling_rejects_depth_gt1_with_n_gt_32) {
    dl_config_t c = default_cfg();
    c.video_n_max = 64;   /* ceiling allows it on paper … */
    c.depth_max = 3;
    dl_decision_t d = default_dec();
    d.depth = 2;
    d.n = 48;             /* … but wfb-ng TX rejects this combo. */
    DL_ASSERT_EQ(dl_ceiling_check(&d, &c), DL_CEILING_DEPTH_N_CONFLICT);
}
