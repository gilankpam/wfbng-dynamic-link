/* test_mavlink.c — STATUSTEXT encoder byte layout + CRC. */
#include "test_main.h"
#include "dl_mavlink.h"

#include <stdint.h>
#include <string.h>

/* Independent X.25 CRC reference — match MAVLink's crc_accumulate
 * loop by a different author to cross-check. */
static uint16_t ref_x25(const uint8_t *buf, size_t len, uint8_t extra) {
    uint16_t crc = 0xFFFF;
    uint8_t tmp;
    for (size_t i = 0; i < len; ++i) {
        tmp = buf[i] ^ (uint8_t)(crc & 0xFF);
        tmp ^= (uint8_t)(tmp << 4);
        crc = (uint16_t)((crc >> 8)
                       ^ ((uint16_t)tmp << 8)
                       ^ ((uint16_t)tmp << 3)
                       ^ ((uint16_t)tmp >> 4));
    }
    tmp = extra ^ (uint8_t)(crc & 0xFF);
    tmp ^= (uint8_t)(tmp << 4);
    crc = (uint16_t)((crc >> 8)
                   ^ ((uint16_t)tmp << 8)
                   ^ ((uint16_t)tmp << 3)
                   ^ ((uint16_t)tmp >> 4));
    return crc;
}

DL_TEST(test_mavlink_frame_layout) {
    uint8_t buf[128];
    size_t n = dl_mavlink_encode_statustext(buf, sizeof(buf),
                                            /*seq=*/42,
                                            /*sysid=*/250,
                                            /*compid=*/191,
                                            /*sev=*/4,
                                            "DL REJECT mcs_too_high");
    DL_ASSERT_EQ(n, 59);      /* 6 header + 51 payload + 2 CRC */
    DL_ASSERT_EQ(buf[0], 0xFE);  /* MAVLink v1 STX */
    DL_ASSERT_EQ(buf[1], 51);    /* STATUSTEXT payload length */
    DL_ASSERT_EQ(buf[2], 42);    /* seq */
    DL_ASSERT_EQ(buf[3], 250);   /* sysid */
    DL_ASSERT_EQ(buf[4], 191);   /* compid */
    DL_ASSERT_EQ(buf[5], 253);   /* STATUSTEXT msgid */
    DL_ASSERT_EQ(buf[6], 4);     /* severity = WARNING */
    /* text starts at offset 7; first 22 bytes are "DL REJECT mcs_too_high" */
    const char *expect = "DL REJECT mcs_too_high";
    DL_ASSERT(memcmp(&buf[7], expect, 22) == 0);
    /* Remaining 28 bytes must be zero-padded. */
    for (int i = 0; i < 28; ++i) {
        DL_ASSERT_EQ(buf[7 + 22 + i], 0);
    }
}

DL_TEST(test_mavlink_crc_matches_x25_over_msg_and_extra) {
    uint8_t buf[128];
    const char *text = "DL WATCHDOG safe_defaults";
    size_t n = dl_mavlink_encode_statustext(buf, sizeof(buf), 7, 1, 2, 3, text);
    DL_ASSERT_EQ(n, 59);

    /* CRC should cover bytes [1..56] (LEN through last payload byte)
     * plus the STATUSTEXT extra-byte (83 per MAVLink v1 common.xml). */
    uint16_t expected = ref_x25(&buf[1], 56, 83);
    uint16_t got = (uint16_t)buf[57] | ((uint16_t)buf[58] << 8);
    DL_ASSERT_EQ(got, expected);
}

DL_TEST(test_mavlink_crc_accumulate_matches_ref) {
    /* Exported accumulator must produce the same bytes as the ref. */
    uint16_t a = 0xFFFF;
    uint16_t b = 0xFFFF;
    const uint8_t data[] = { 0x01, 0x02, 0x03, 0xAB, 0xCD };
    for (size_t i = 0; i < sizeof(data); ++i) {
        dl_mavlink_crc_accumulate(data[i], &a);
    }
    b = ref_x25(data, sizeof(data), 0);
    /* Apply extra=0 tail to `a` too so both see the same total. */
    dl_mavlink_crc_accumulate(0, &a);
    DL_ASSERT_EQ(a, b);
}

DL_TEST(test_mavlink_rejects_small_buffer) {
    uint8_t small[10];
    size_t n = dl_mavlink_encode_statustext(small, sizeof(small),
                                            0, 1, 2, 3, "hi");
    DL_ASSERT_EQ(n, 0);
}

DL_TEST(test_mavlink_truncates_text_over_50_chars) {
    uint8_t buf[128];
    /* 60-char string; only first 50 should land in the payload. */
    const char *long_text =
        "0123456789012345678901234567890123456789012345678901234567890123";
    size_t n = dl_mavlink_encode_statustext(buf, sizeof(buf),
                                            0, 1, 2, 3, long_text);
    DL_ASSERT_EQ(n, 59);
    /* Text at offset 7..56 must equal the first 50 chars of long_text. */
    DL_ASSERT(memcmp(&buf[7], long_text, 50) == 0);
}

DL_TEST(test_mavlink_empty_text_zero_pads_all_50) {
    uint8_t buf[128];
    size_t n = dl_mavlink_encode_statustext(buf, sizeof(buf), 0, 1, 2, 3, "");
    DL_ASSERT_EQ(n, 59);
    for (int i = 0; i < 50; ++i) {
        DL_ASSERT_EQ(buf[7 + i], 0);
    }
}
