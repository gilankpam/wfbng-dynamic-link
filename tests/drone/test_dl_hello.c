#include "dl_hello.h"
#include "dl_config.h"
#include "test_main.h"

#include <stdio.h>
#include <string.h>

static void setup_cfg(dl_config_t *cfg) {
    dl_config_defaults(cfg);
    strncpy(cfg->hello_wfb_yaml_path,
            "../tests/drone/fixtures/wfb_basic.yaml", DL_CONF_MAX_STR - 1);
    strncpy(cfg->hello_majestic_yaml_path,
            "../tests/drone/fixtures/majestic_basic.yaml", DL_CONF_MAX_STR - 1);
    cfg->hello_announce_initial_ms = 500;
    cfg->hello_announce_steady_ms = 5000;
    cfg->hello_keepalive_ms = 10000;
    cfg->hello_announce_initial_count = 3;
}

DL_TEST(hello_init_reads_mtu_and_fps_from_fixtures) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_sm_t h;
    int rc = dl_hello_init(&h, &cfg);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_ANNOUNCING);
    DL_ASSERT_EQ(h.mtu_bytes, 3994);
    DL_ASSERT_EQ(h.fps, 60);
    DL_ASSERT(h.generation_id != 0);
}

DL_TEST(hello_init_fails_when_wfb_yaml_unreadable) {
    dl_config_t cfg; setup_cfg(&cfg);
    strncpy(cfg.hello_wfb_yaml_path,
            "../tests/drone/fixtures/does_not_exist.yaml",
            DL_CONF_MAX_STR - 1);
    dl_hello_sm_t h;
    int rc = dl_hello_init(&h, &cfg);
    DL_ASSERT_EQ(rc, -1);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_DISABLED);
}

DL_TEST(hello_announcing_first_delay_is_immediate) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_sm_t h;
    dl_hello_init(&h, &cfg);
    DL_ASSERT_EQ(dl_hello_next_delay_ms(&h), 0);
}

DL_TEST(hello_announcing_uses_initial_ms_for_first_retries) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_sm_t h;
    dl_hello_init(&h, &cfg);
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    dl_hello_build_announce(&h, buf, sizeof(buf));
    DL_ASSERT_EQ(dl_hello_next_delay_ms(&h), 500);
    dl_hello_build_announce(&h, buf, sizeof(buf));
    dl_hello_build_announce(&h, buf, sizeof(buf));
    DL_ASSERT_EQ(dl_hello_next_delay_ms(&h), 5000);
}

DL_TEST(hello_ack_matching_genid_transitions_to_keepalive) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_sm_t h;
    dl_hello_init(&h, &cfg);
    dl_hello_ack_t ack = { .generation_id_echo = h.generation_id };
    int matched = dl_hello_on_ack(&h, &ack);
    DL_ASSERT_EQ(matched, 1);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_KEEPALIVE);
    DL_ASSERT_EQ(dl_hello_next_delay_ms(&h), 10000);
}

DL_TEST(hello_ack_mismatching_genid_ignored) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_sm_t h;
    dl_hello_init(&h, &cfg);
    uint32_t orig_state = h.state;
    dl_hello_ack_t ack = { .generation_id_echo = h.generation_id ^ 0xFFFFFFFFu };
    int matched = dl_hello_on_ack(&h, &ack);
    DL_ASSERT_EQ(matched, 0);
    DL_ASSERT_EQ(h.state, orig_state);
}

DL_TEST(hello_keepalive_without_ack_drops_back_to_announcing) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_sm_t h;
    dl_hello_init(&h, &cfg);
    dl_hello_ack_t ack = { .generation_id_echo = h.generation_id };
    dl_hello_on_ack(&h, &ack);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_KEEPALIVE);
    DL_ASSERT_EQ(dl_hello_on_keepalive_tick(&h), 0);
    DL_ASSERT_EQ(dl_hello_on_keepalive_tick(&h), 0);
    DL_ASSERT_EQ(dl_hello_on_keepalive_tick(&h), 1);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_ANNOUNCING);
}

DL_TEST(hello_build_announce_sets_packet_fields_from_init) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_sm_t h;
    dl_hello_init(&h, &cfg);
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    size_t n = dl_hello_build_announce(&h, buf, sizeof(buf));
    DL_ASSERT_EQ(n, DL_HELLO_ON_WIRE_SIZE);
    dl_hello_t decoded;
    DL_ASSERT_EQ(dl_wire_decode_hello(buf, n, &decoded), DL_DECODE_OK);
    DL_ASSERT_EQ(decoded.mtu_bytes, 3994);
    DL_ASSERT_EQ(decoded.fps, 60);
    DL_ASSERT_EQ(decoded.generation_id, h.generation_id);
}

DL_TEST(hello_announce_flags_zero_when_interleaving_supported) {
    dl_config_t cfg; setup_cfg(&cfg);
    cfg.interleaving_supported = true;
    dl_hello_sm_t h;
    DL_ASSERT_EQ(dl_hello_init(&h, &cfg), 0);
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    size_t n = dl_hello_build_announce(&h, buf, sizeof(buf));
    DL_ASSERT_EQ((int)n, DL_HELLO_ON_WIRE_SIZE);
    /* Byte 5 is the flags field. Capable drone → zero. */
    DL_ASSERT_EQ(buf[5], 0x00);
}

DL_TEST(hello_announce_flags_sets_vanilla_bit_when_unsupported) {
    dl_config_t cfg; setup_cfg(&cfg);
    cfg.interleaving_supported = false;
    dl_hello_sm_t h;
    DL_ASSERT_EQ(dl_hello_init(&h, &cfg), 0);
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    size_t n = dl_hello_build_announce(&h, buf, sizeof(buf));
    DL_ASSERT_EQ((int)n, DL_HELLO_ON_WIRE_SIZE);
    DL_ASSERT_EQ(buf[5] & DL_HELLO_FLAG_VANILLA_WFB_NG, DL_HELLO_FLAG_VANILLA_WFB_NG);
}

DL_TEST(hello_init_reads_fps_from_waybeam_json) {
    dl_config_t cfg; setup_cfg(&cfg);
    /* Force majestic path to fail so only the waybeam JSON read can
     * succeed — pins the dispatch branch. */
    strncpy(cfg.hello_majestic_yaml_path,
            "../tests/drone/fixtures/does_not_exist.yaml",
            DL_CONF_MAX_STR - 1);
    strncpy(cfg.encoder_kind, "waybeam", DL_CONF_MAX_STR - 1);
    strncpy(cfg.hello_waybeam_json_path,
            "../tests/drone/fixtures/waybeam_basic.json",
            DL_CONF_MAX_STR - 1);
    dl_hello_sm_t h;
    int rc = dl_hello_init(&h, &cfg);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_ANNOUNCING);
    DL_ASSERT_EQ(h.fps, 60);
    DL_ASSERT_EQ(h.mtu_bytes, 3994);
}

DL_TEST(hello_init_fails_when_waybeam_json_unreadable) {
    dl_config_t cfg; setup_cfg(&cfg);
    strncpy(cfg.encoder_kind, "waybeam", DL_CONF_MAX_STR - 1);
    strncpy(cfg.hello_waybeam_json_path,
            "../tests/drone/fixtures/does_not_exist.json",
            DL_CONF_MAX_STR - 1);
    dl_hello_sm_t h;
    int rc = dl_hello_init(&h, &cfg);
    DL_ASSERT_EQ(rc, -1);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_DISABLED);
}

DL_TEST(hello_init_fails_for_unknown_encoder_kind) {
    dl_config_t cfg; setup_cfg(&cfg);
    strncpy(cfg.encoder_kind, "bogus", DL_CONF_MAX_STR - 1);
    dl_hello_sm_t h;
    int rc = dl_hello_init(&h, &cfg);
    DL_ASSERT_EQ(rc, -1);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_DISABLED);
}
