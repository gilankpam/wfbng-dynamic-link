/* test_config_cli.c — field table, by-name setters, validate. */
#include "test_main.h"
#include "dl_config.h"

#include <string.h>

DL_TEST(test_int_fields_table_nonempty) {
    size_t n = 0;
    const dl_int_field_t *t = dl_config_int_fields(&n);
    DL_ASSERT(t != NULL);
    DL_ASSERT(n > 0);
}

DL_TEST(test_bool_fields_table_nonempty) {
    size_t n = 0;
    DL_ASSERT(dl_config_bool_fields(&n) != NULL);
    DL_ASSERT(n > 0);
}

DL_TEST(test_str_fields_table_nonempty) {
    size_t n = 0;
    DL_ASSERT(dl_config_str_fields(&n) != NULL);
    DL_ASSERT(n > 0);
}

DL_TEST(test_set_int_by_name_writes_field) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_int_by_name(&c, "safe_mcs", "3"), 0);
    DL_ASSERT_EQ(c.safe_mcs, 3);
}

DL_TEST(test_set_int_by_name_kebab_works) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_int_by_name(&c, "safe-mcs", "5"), 0);
    DL_ASSERT_EQ(c.safe_mcs, 5);
}

DL_TEST(test_set_int_by_name_rejects_out_of_range_high) {
    dl_config_t c; dl_config_defaults(&c);
    /* safe_mcs is u8 0..7. */
    DL_ASSERT_EQ(dl_config_set_int_by_name(&c, "safe_mcs", "9"), -1);
    DL_ASSERT_EQ(c.safe_mcs, 1);  /* default unchanged */
}

DL_TEST(test_set_int_by_name_rejects_unknown) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_int_by_name(&c, "no_such_field", "1"), -1);
}

DL_TEST(test_set_int_by_name_handles_i8) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_int_by_name(&c, "safe_tx_power_dBm", "-5"), 0);
    DL_ASSERT_EQ(c.safe_tx_power_dBm, -5);
    DL_ASSERT_EQ(dl_config_set_int_by_name(&c, "safe_tx_power_dBm", "-11"), -1);
}

DL_TEST(test_set_int_by_name_handles_u32) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_int_by_name(&c, "health_timeout_ms", "20000"), 0);
    DL_ASSERT_EQ(c.health_timeout_ms, 20000u);
}

DL_TEST(test_set_bool_by_name_writes_field) {
    dl_config_t c; dl_config_defaults(&c);
    c.osd_debug_latency = false;
    DL_ASSERT_EQ(dl_config_set_bool_by_name(&c, "osd_debug_latency", true), 0);
    DL_ASSERT_EQ(c.osd_debug_latency, true);
}

DL_TEST(test_set_bool_by_name_kebab_works) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_bool_by_name(&c, "osd-debug-latency", true), 0);
    DL_ASSERT_EQ(c.osd_debug_latency, true);
}

DL_TEST(test_set_bool_by_name_rejects_unknown) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_bool_by_name(&c, "no_such_bool", true), -1);
}

DL_TEST(test_set_str_by_name_writes_field) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_str_by_name(&c, "encoder_host", "192.168.1.1"), 0);
    DL_ASSERT_STR_EQ(c.encoder_host, "192.168.1.1");
}

DL_TEST(test_set_str_by_name_kebab_works) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_str_by_name(&c, "encoder-host", "10.0.0.1"), 0);
    DL_ASSERT_STR_EQ(c.encoder_host, "10.0.0.1");
}

DL_TEST(test_set_str_by_name_rejects_too_long) {
    dl_config_t c; dl_config_defaults(&c);
    char big[DL_CONF_MAX_STR + 16];
    memset(big, 'x', sizeof(big) - 1);
    big[sizeof(big) - 1] = '\0';
    DL_ASSERT_EQ(dl_config_set_str_by_name(&c, "encoder_host", big), -1);
}

DL_TEST(test_set_str_by_name_rejects_unknown) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_str_by_name(&c, "no_such_str", "x"), -1);
}

DL_TEST(test_phase3_debug_keys_absent_from_tables) {
    /* These conf keys are intentionally NOT exposed via the field
     * tables (the conf parser keeps them in its legacy stanza). The
     * by-name setters must reject them so the CLI can't reach them. */
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_set_bool_by_name(&c, "debug_enable", true), -1);
    DL_ASSERT_EQ(dl_config_set_int_by_name (&c, "dbg_log_enable", "1"), -1);
    DL_ASSERT_EQ(dl_config_set_str_by_name (&c, "dbg_log_dir", "/x"), -1);
    DL_ASSERT_EQ(dl_config_set_int_by_name (&c, "dbg_max_bytes", "4096"), -1);
    DL_ASSERT_EQ(dl_config_set_bool_by_name(&c, "dbg_fsync_each", true), -1);
}

DL_TEST(test_no_name_in_multiple_tables) {
    /* dl_bool_field_t and dl_str_field_t are structurally identical;
     * a stray F_BOOL/F_STR misuse would compile silently. Guard against
     * any field appearing in more than one table. */
    size_t ni, nb, ns;
    const dl_int_field_t  *ti = dl_config_int_fields(&ni);
    const dl_bool_field_t *tb = dl_config_bool_fields(&nb);
    const dl_str_field_t  *ts = dl_config_str_fields(&ns);
    for (size_t i = 0; i < ni; i++) {
        for (size_t j = 0; j < nb; j++)
            DL_ASSERT(strcmp(ti[i].name, tb[j].name) != 0);
        for (size_t j = 0; j < ns; j++)
            DL_ASSERT(strcmp(ti[i].name, ts[j].name) != 0);
    }
    for (size_t i = 0; i < nb; i++)
        for (size_t j = 0; j < ns; j++)
            DL_ASSERT(strcmp(tb[i].name, ts[j].name) != 0);
}

DL_TEST(test_validate_accepts_defaults) {
    dl_config_t c; dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_validate(&c), 0);
}

DL_TEST(test_validate_rejects_inverted_roi_qp) {
    dl_config_t c; dl_config_defaults(&c);
    c.roi_qp_threshold_kbps  = 1000;
    c.roi_qp_low_anchor_kbps = 2000;
    DL_ASSERT_EQ(dl_config_validate(&c), -1);
}

DL_TEST(test_validate_rejects_equal_roi_qp) {
    dl_config_t c; dl_config_defaults(&c);
    c.roi_qp_threshold_kbps  = 2000;
    c.roi_qp_low_anchor_kbps = 2000;
    DL_ASSERT_EQ(dl_config_validate(&c), -1);
}
