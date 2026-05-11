#include "dl_yaml_get.h"
#include "test_main.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

/* Path is relative to the cwd of `make -C drone test`, which is the
 * drone/ subdirectory — mirrors the Makefile's TESTDIR convention. */
#define FIX(name) "../tests/drone/fixtures/" name

DL_TEST(yaml_reads_mlink_from_wfb_basic) {
    int v = -1;
    int rc = dl_yaml_get_int(FIX("wfb_basic.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 3994);
}

DL_TEST(yaml_reads_fps_from_majestic_basic) {
    int v = -1;
    int rc = dl_yaml_get_int(FIX("majestic_basic.yaml"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 60);
}

DL_TEST(yaml_returns_einval_when_key_missing) {
    int v = 999;
    int rc = dl_yaml_get_int(FIX("wfb_no_mlink.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
    DL_ASSERT_EQ(v, 999);  /* unchanged on failure */
}

DL_TEST(yaml_returns_einval_when_block_missing) {
    int v = 0;
    int rc = dl_yaml_get_int(FIX("wfb_no_mlink.yaml"), "telemetry", "router", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(yaml_returns_enoent_when_file_missing) {
    int v = 0;
    int rc = dl_yaml_get_int(FIX("does_not_exist.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, -ENOENT);
}

DL_TEST(yaml_handles_comments_and_inline) {
    int v = -1;
    int rc = dl_yaml_get_int(FIX("wfb_with_comments.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 4096);
}

DL_TEST(yaml_rejects_malformed_file) {
    int v = 0;
    int rc = dl_yaml_get_int(FIX("wfb_malformed.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(yaml_ignores_lookalike_key_at_wrong_indent) {
    /* A top-level "mlink:" line outside a "wireless:" block must not
     * match. The malformed fixture has exactly this case. */
    int v = 0;
    int rc = dl_yaml_get_int(FIX("wfb_malformed.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}
