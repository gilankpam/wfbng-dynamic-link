/* test_apply_stagger.c — direction helper for staggered profile apply. */
#include "test_main.h"
#include "dl_apply.h"

DL_TEST(test_apply_dir_first_decision_is_equal) {
    /* First decision (no prior state) skips staggering even if the
     * incoming bitrate differs from the zero-init baseline. */
    DL_ASSERT_EQ(dl_apply_direction(0, 12000, true), DL_APPLY_DIR_EQUAL);
    DL_ASSERT_EQ(dl_apply_direction(8000, 0, true), DL_APPLY_DIR_EQUAL);
}

DL_TEST(test_apply_dir_equal_bitrate_is_equal) {
    DL_ASSERT_EQ(dl_apply_direction(8000, 8000, false), DL_APPLY_DIR_EQUAL);
    DL_ASSERT_EQ(dl_apply_direction(0, 0, false),       DL_APPLY_DIR_EQUAL);
}

DL_TEST(test_apply_dir_higher_bitrate_is_up) {
    DL_ASSERT_EQ(dl_apply_direction(8000, 12000, false), DL_APPLY_DIR_UP);
    DL_ASSERT_EQ(dl_apply_direction(0,    1,     false), DL_APPLY_DIR_UP);
}

DL_TEST(test_apply_dir_lower_bitrate_is_down) {
    DL_ASSERT_EQ(dl_apply_direction(12000, 8000, false), DL_APPLY_DIR_DOWN);
    DL_ASSERT_EQ(dl_apply_direction(1,     0,    false), DL_APPLY_DIR_DOWN);
}

DL_TEST(test_apply_dir_extremes) {
    /* uint16_t boundaries — make sure no signed-comparison surprise. */
    DL_ASSERT_EQ(dl_apply_direction(65535, 0,     false), DL_APPLY_DIR_DOWN);
    DL_ASSERT_EQ(dl_apply_direction(0,     65535, false), DL_APPLY_DIR_UP);
}
