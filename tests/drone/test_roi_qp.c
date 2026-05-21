/* test_roi_qp.c — pure-function tests for compute_roi_qp. */
#include "test_main.h"
#include "dl_backend_enc.h"
#include "dl_config.h"

static dl_config_t default_cfg(void) {
    dl_config_t c;
    dl_config_defaults(&c);
    return c;
}

DL_TEST(test_roi_qp_above_threshold_is_zero) {
    dl_config_t cfg = default_cfg();
    DL_ASSERT_EQ(dl_compute_roi_qp(6000, &cfg), 0);
    DL_ASSERT_EQ(dl_compute_roi_qp(10000, &cfg), 0);
}

DL_TEST(test_roi_qp_at_low_anchor_is_floor) {
    dl_config_t cfg = default_cfg();
    DL_ASSERT_EQ(dl_compute_roi_qp(2000, &cfg), -24);
}

DL_TEST(test_roi_qp_below_low_anchor_clamps_at_floor) {
    dl_config_t cfg = default_cfg();
    DL_ASSERT_EQ(dl_compute_roi_qp(1500, &cfg), -24);
    DL_ASSERT_EQ(dl_compute_roi_qp(500, &cfg), -24);
    DL_ASSERT_EQ(dl_compute_roi_qp(0, &cfg), -24);
}

DL_TEST(test_roi_qp_midpoint_ramps_linearly) {
    /* Defaults: threshold=6000, anchor=2000, floor=-24, step=3.
     * At bitrate=4000 raw = -24 * (4000-2000)/4000 ... wait recheck:
     * span = 6000-2000 = 4000
     * delta = 4000-2000 = 2000
     * raw = -24 * (4000 - 2000) / 4000 = -24 * 2000 / 4000 = -12.
     * -12 is a multiple of 3, so quantized = -12. */
    dl_config_t cfg = default_cfg();
    DL_ASSERT_EQ(dl_compute_roi_qp(4000, &cfg), -12);
    /* At 5000: raw = -24 * (4000 - 3000) / 4000 = -6. */
    DL_ASSERT_EQ(dl_compute_roi_qp(5000, &cfg), -6);
    /* At 3000: raw = -24 * (4000 - 1000) / 4000 = -18. */
    DL_ASSERT_EQ(dl_compute_roi_qp(3000, &cfg), -18);
}

DL_TEST(test_roi_qp_quantization_lands_on_step_multiples) {
    dl_config_t cfg = default_cfg();
    /* Sweep 2000..6000 in 50-kbps increments; every result must be a
     * multiple of step (3) and must be in [floor, 0]. */
    for (int br = 2000; br <= 6000; br += 50) {
        int q = dl_compute_roi_qp((uint16_t)br, &cfg);
        DL_ASSERT(q <= 0);
        DL_ASSERT(q >= cfg.roi_qp_floor);
        DL_ASSERT_EQ(q % cfg.roi_qp_step, 0);
    }
}

DL_TEST(test_roi_qp_custom_config) {
    /* threshold=8000, anchor=3000, floor=-18, step=2. */
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    cfg.roi_qp_threshold_kbps  = 8000;
    cfg.roi_qp_low_anchor_kbps = 3000;
    cfg.roi_qp_floor           = -18;
    cfg.roi_qp_step            = 2;
    /* Endpoints */
    DL_ASSERT_EQ(dl_compute_roi_qp(8000, &cfg), 0);
    DL_ASSERT_EQ(dl_compute_roi_qp(3000, &cfg), -18);
    /* Midpoint: span=5000, delta=2500, raw = -18 * 2500/5000 = -9.
     * Quantize step=2: -9 / 2 = -4 (C truncates toward zero), * 2 = -8. */
    DL_ASSERT_EQ(dl_compute_roi_qp(5500, &cfg), -8);
}
