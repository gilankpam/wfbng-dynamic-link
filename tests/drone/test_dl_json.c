#include "dl_json_get.h"
#include "test_main.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#define FIX(name) "../tests/drone/fixtures/" name

DL_TEST(json_reads_video0_fps_from_basic) {
    int v = -1;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 60);
}

DL_TEST(json_reads_record_fps_distinctly_from_video0) {
    /* The block-anchoring invariant: record.fps = 0 must NOT be
     * returned when caller asks for video0.fps, and vice versa. */
    int v = -1;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "record", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 0);

    v = -1;
    rc = dl_json_get_int(FIX("waybeam_basic.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 60);
}

DL_TEST(json_returns_enoent_when_file_missing) {
    int v = 99;
    int rc = dl_json_get_int(FIX("does_not_exist.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, -ENOENT);
    DL_ASSERT_EQ(v, 99);  /* unchanged on failure */
}

DL_TEST(json_returns_einval_when_block_missing) {
    int v = 0;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "bogus", "fps", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(json_returns_einval_when_key_missing) {
    int v = 0;
    int rc = dl_json_get_int(FIX("waybeam_no_fps.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(json_returns_einval_for_non_integer_value) {
    int v = 0;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "video0", "size", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(json_handles_compact_no_whitespace) {
    int v = -1;
    int rc = dl_json_get_int(FIX("waybeam_compact.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 60);
}

DL_TEST(json_accepts_zero_value) {
    int v = -1;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "record", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 0);
}
