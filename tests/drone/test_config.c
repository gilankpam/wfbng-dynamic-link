/* test_config.c — drone.conf parser. */
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
    DL_ASSERT_EQ(c.safe_k, 8);
    DL_ASSERT_EQ(c.health_timeout_ms, 10000);
}

DL_TEST(test_config_rejects_removed_ceiling_key) {
    /* Any of the seven removed keys must cause a load failure. */
    const char *body = "mcs_max = 7\n";
    char path[64];
    DL_ASSERT_EQ(write_tmp(body, path, sizeof(path)), 0);
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_load(path, &c), -1);
    unlink(path);
}

DL_TEST(test_config_parses_good_file) {
    const char *body =
        "# sample\n"
        "listen_addr = 127.0.0.1\n"
        "listen_port = 5900\n"
        "wfb_tx_ctrl_port = 8010\n"
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

    unlink(path);
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

DL_TEST(test_config_debug_defaults_off) {
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT(!c.debug_enable);
    DL_ASSERT_EQ(c.dbg_log_enable, -1);
    DL_ASSERT_STR_EQ(c.gs_tunnel_addr, "10.5.0.1");
    DL_ASSERT_EQ(c.gs_tunnel_port, 5801);
    DL_ASSERT_STR_EQ(c.dbg_log_dir, "/sdcard/dl-events");
    DL_ASSERT_EQ(c.dbg_max_bytes, 32u * 1024u * 1024u);
    DL_ASSERT(!c.dbg_fsync_each);
}

DL_TEST(test_config_dbg_log_resolves_follow_master) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.dbg_log_enable = -1;
    c.debug_enable = false;
    DL_ASSERT(!dl_config_dbg_log_resolved(&c));
    c.debug_enable = true;
    DL_ASSERT(dl_config_dbg_log_resolved(&c));
}

DL_TEST(test_config_dbg_log_resolves_force_off) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = true;
    c.dbg_log_enable = 0;
    DL_ASSERT(!dl_config_dbg_log_resolved(&c));
}

DL_TEST(test_config_dbg_log_resolves_force_on) {
    dl_config_t c;
    dl_config_defaults(&c);
    c.debug_enable = false;
    c.dbg_log_enable = 1;
    DL_ASSERT(dl_config_dbg_log_resolved(&c));
}

DL_TEST(test_config_parses_debug_block) {
    const char *body =
        "debug_enable   = 1\n"
        "dbg_log_enable = 0\n"
        "gs_tunnel_addr = 10.0.0.5\n"
        "gs_tunnel_port = 5901\n"
        "dbg_log_dir    = /mnt/sd/events\n"
        "dbg_max_bytes  = 65536\n"
        "dbg_fsync_each = 1\n";
    char path[64];
    DL_ASSERT_EQ(write_tmp(body, path, sizeof(path)), 0);
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_load(path, &c), 0);
    DL_ASSERT(c.debug_enable);
    DL_ASSERT_EQ(c.dbg_log_enable, 0);
    DL_ASSERT_STR_EQ(c.gs_tunnel_addr, "10.0.0.5");
    DL_ASSERT_EQ(c.gs_tunnel_port, 5901);
    DL_ASSERT_STR_EQ(c.dbg_log_dir, "/mnt/sd/events");
    DL_ASSERT_EQ(c.dbg_max_bytes, 65536);
    DL_ASSERT(c.dbg_fsync_each);
    /* tristate=0 forces off even if master says on */
    DL_ASSERT(!dl_config_dbg_log_resolved(&c));
    unlink(path);
}

DL_TEST(config_interleaving_supported_default_true) {
    dl_config_t cfg;
    dl_config_defaults(&cfg);
    DL_ASSERT_EQ(cfg.interleaving_supported, true);
}

DL_TEST(config_interleaving_supported_parses_zero) {
    char path[] = "/tmp/dl_test_conf_XXXXXX";
    int fd = mkstemp(path);
    DL_ASSERT(fd >= 0);
    const char *content = "interleaving_supported = 0\n";
    DL_ASSERT_EQ((int)write(fd, content, strlen(content)), (int)strlen(content));
    close(fd);

    dl_config_t cfg;
    dl_config_defaults(&cfg);
    int rc = dl_config_load(path, &cfg);
    unlink(path);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(cfg.interleaving_supported, false);
}

DL_TEST(config_interleaving_supported_parses_one) {
    char path[] = "/tmp/dl_test_conf_XXXXXX";
    int fd = mkstemp(path);
    DL_ASSERT(fd >= 0);
    const char *content = "interleaving_supported = 1\n";
    DL_ASSERT_EQ((int)write(fd, content, strlen(content)), (int)strlen(content));
    close(fd);

    dl_config_t cfg;
    dl_config_defaults(&cfg);
    cfg.interleaving_supported = false;
    int rc = dl_config_load(path, &cfg);
    unlink(path);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(cfg.interleaving_supported, true);
}

DL_TEST(config_hello_waybeam_json_path_default) {
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_STR_EQ(c.hello_waybeam_json_path, "/etc/waybeam.json");
}

DL_TEST(config_hello_waybeam_json_path_parses) {
    const char *body = "hello_waybeam_json_path = /tmp/wb.json\n";
    char path[64];
    DL_ASSERT_EQ(write_tmp(body, path, sizeof(path)), 0);
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_load(path, &c), 0);
    DL_ASSERT_STR_EQ(c.hello_waybeam_json_path, "/tmp/wb.json");
    unlink(path);
}
