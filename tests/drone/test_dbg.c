/* test_dbg.c — dl_dbg SD-card writer. Uses a tmp dir per test. */
#include "test_main.h"
#include "dl_config.h"
#include "dl_dbg.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

static int tmpdir(char *out, size_t outlen) {
    static int counter = 0;
    snprintf(out, outlen, "/tmp/dldbg_%d_%d", getpid(), counter++);
    if (mkdir(out, 0755) < 0 && errno != EEXIST) return -1;
    return 0;
}

static long file_size(const char *path) {
    struct stat st;
    if (stat(path, &st) < 0) return -1;
    return (long)st.st_size;
}

static int read_file(const char *path, char *buf, size_t buflen) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    size_t n = fread(buf, 1, buflen - 1, f);
    fclose(f);
    buf[n] = '\0';
    return (int)n;
}

DL_TEST(test_dbg_master_off_is_noop) {
    char dir[64];
    DL_ASSERT_EQ(tmpdir(dir, sizeof(dir)), 0);

    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = false;
    c.dbg_log_enable = -1;
    snprintf(c.dbg_log_dir, sizeof(c.dbg_log_dir), "%s", dir);

    dl_dbg_init(&c);
    DL_ASSERT(!dl_dbg_enabled());
    dl_dbg_emit("TEST_REASON", DL_DBG_SEV_WARN, "{}");
    dl_dbg_close();

    /* No file should have been created. */
    char path[128];
    snprintf(path, sizeof(path), "%s/dl-events.jsonl", dir);
    DL_ASSERT(file_size(path) < 0);
}

DL_TEST(test_dbg_emit_writes_jsonl_line) {
    char dir[64];
    DL_ASSERT_EQ(tmpdir(dir, sizeof(dir)), 0);

    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = true;
    snprintf(c.dbg_log_dir, sizeof(c.dbg_log_dir), "%s", dir);

    dl_dbg_init(&c);
    DL_ASSERT(dl_dbg_enabled());
    dl_dbg_emit("ENC_RESPONSE_BAD", DL_DBG_SEV_WARN,
                "{\"http\":500,\"body\":\"oops\"}");
    dl_dbg_close();

    char path[128];
    snprintf(path, sizeof(path), "%s/dl-events.jsonl", dir);
    DL_ASSERT(file_size(path) > 0);

    char buf[512];
    DL_ASSERT(read_file(path, buf, sizeof(buf)) > 0);
    DL_ASSERT(strstr(buf, "\"reason\":\"ENC_RESPONSE_BAD\"") != NULL);
    DL_ASSERT(strstr(buf, "\"sev\":\"warn\"") != NULL);
    DL_ASSERT(strstr(buf, "\"http\":500") != NULL);
    DL_ASSERT(strstr(buf, "\"seq\":0") != NULL);
}

DL_TEST(test_dbg_seq_increments_per_emit) {
    char dir[64];
    DL_ASSERT_EQ(tmpdir(dir, sizeof(dir)), 0);

    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = true;
    snprintf(c.dbg_log_dir, sizeof(c.dbg_log_dir), "%s", dir);

    dl_dbg_init(&c);
    dl_dbg_emit("A", DL_DBG_SEV_INFO, NULL);
    dl_dbg_emit("B", DL_DBG_SEV_INFO, "{}");
    dl_dbg_emit("C", DL_DBG_SEV_INFO, "{}");
    dl_dbg_close();

    char path[128];
    snprintf(path, sizeof(path), "%s/dl-events.jsonl", dir);
    char buf[1024];
    DL_ASSERT(read_file(path, buf, sizeof(buf)) > 0);
    DL_ASSERT(strstr(buf, "\"seq\":0") != NULL);
    DL_ASSERT(strstr(buf, "\"seq\":1") != NULL);
    DL_ASSERT(strstr(buf, "\"seq\":2") != NULL);
}

DL_TEST(test_dbg_per_feature_force_off) {
    char dir[64];
    DL_ASSERT_EQ(tmpdir(dir, sizeof(dir)), 0);

    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = true;
    c.dbg_log_enable = 0;     /* explicit force-off overrides master */
    snprintf(c.dbg_log_dir, sizeof(c.dbg_log_dir), "%s", dir);

    dl_dbg_init(&c);
    DL_ASSERT(!dl_dbg_enabled());
    dl_dbg_close();
}

DL_TEST(test_dbg_emit_http_escapes_body) {
    char dir[64];
    DL_ASSERT_EQ(tmpdir(dir, sizeof(dir)), 0);

    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = true;
    snprintf(c.dbg_log_dir, sizeof(c.dbg_log_dir), "%s", dir);

    dl_dbg_init(&c);
    /* Body contains a quote and a newline — both must be escaped to
     * keep the JSON valid. */
    const char *body = "bad \"value\"\nnext line";
    dl_dbg_emit_http("ENC_RESPONSE_BAD", DL_DBG_SEV_WARN,
                     500, body, strlen(body));
    dl_dbg_close();

    char path[128];
    snprintf(path, sizeof(path), "%s/dl-events.jsonl", dir);
    char buf[1024];
    DL_ASSERT(read_file(path, buf, sizeof(buf)) > 0);
    DL_ASSERT(strstr(buf, "\\\"value\\\"") != NULL);
    DL_ASSERT(strstr(buf, "\\n") != NULL);
}

DL_TEST(test_dbg_rotate_when_over_cap) {
    char dir[64];
    DL_ASSERT_EQ(tmpdir(dir, sizeof(dir)), 0);

    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = true;
    c.dbg_max_bytes = 4096;   /* tiny cap to force rotation */
    snprintf(c.dbg_log_dir, sizeof(c.dbg_log_dir), "%s", dir);

    dl_dbg_init(&c);
    /* Each emit is roughly 100 bytes; 80 emits ≈ 8 KB → at least one
     * rotation. */
    for (int i = 0; i < 80; ++i) {
        dl_dbg_emit("FILLER", DL_DBG_SEV_INFO,
                    "{\"k\":\"some padding to push past the cap\"}");
    }
    dl_dbg_close();

    char active[128];
    char rotated[128];
    snprintf(active,  sizeof(active),  "%s/dl-events.jsonl",   dir);
    snprintf(rotated, sizeof(rotated), "%s/dl-events.jsonl.1", dir);
    DL_ASSERT(file_size(active) > 0);
    DL_ASSERT(file_size(rotated) > 0);
}

DL_TEST(test_dbg_truncates_huge_detail_safely) {
    char dir[64];
    DL_ASSERT_EQ(tmpdir(dir, sizeof(dir)), 0);

    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = true;
    snprintf(c.dbg_log_dir, sizeof(c.dbg_log_dir), "%s", dir);

    dl_dbg_init(&c);
    /* 4 KB detail blob. dl_dbg should cap and still emit a closed
     * line (}\n). */
    char *huge = malloc(4096);
    DL_ASSERT(huge != NULL);
    memset(huge, 'x', 4095);
    huge[4095] = '\0';
    dl_dbg_emit("HUGE", DL_DBG_SEV_INFO, huge);
    free(huge);
    dl_dbg_close();

    char path[128];
    snprintf(path, sizeof(path), "%s/dl-events.jsonl", dir);
    char buf[4096];
    int n = read_file(path, buf, sizeof(buf));
    DL_ASSERT(n > 0);
    /* Last char must be newline (line was closed) */
    DL_ASSERT(buf[n - 1] == '\n');
}
