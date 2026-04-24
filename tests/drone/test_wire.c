/* test_wire.c — dl_wire encode/decode + CRC. */
#include "test_main.h"
#include "dl_wire.h"

#include <string.h>

DL_TEST(test_wire_round_trip) {
    dl_decision_t d = {
        .flags = DL_FLAG_IDR_REQUEST,
        .sequence = 0xDEADBEEF,
        .timestamp_ms = 12345,
        .mcs = 5,
        .bandwidth = 20,
        .tx_power_dBm = -10,
        .k = 8,
        .n = 14,
        .depth = 2,
        .bitrate_kbps = 12000,
        .roi_qp = 30,
        .fps = 60,
    };
    uint8_t buf[DL_WIRE_ON_WIRE_SIZE];
    DL_ASSERT_EQ(dl_wire_encode(&d, buf, sizeof(buf)), DL_WIRE_ON_WIRE_SIZE);

    dl_decision_t r = {0};
    DL_ASSERT_EQ(dl_wire_decode(buf, sizeof(buf), &r), DL_DECODE_OK);
    DL_ASSERT_EQ(r.sequence, d.sequence);
    DL_ASSERT_EQ(r.timestamp_ms, d.timestamp_ms);
    DL_ASSERT_EQ(r.mcs, d.mcs);
    DL_ASSERT_EQ(r.bandwidth, d.bandwidth);
    DL_ASSERT_EQ(r.tx_power_dBm, d.tx_power_dBm);
    DL_ASSERT_EQ(r.k, d.k);
    DL_ASSERT_EQ(r.n, d.n);
    DL_ASSERT_EQ(r.depth, d.depth);
    DL_ASSERT_EQ(r.bitrate_kbps, d.bitrate_kbps);
    DL_ASSERT_EQ(r.roi_qp, d.roi_qp);
    DL_ASSERT_EQ(r.fps, d.fps);
    DL_ASSERT_EQ(r.flags & DL_FLAG_IDR_REQUEST, DL_FLAG_IDR_REQUEST);
}

DL_TEST(test_wire_endianness_big) {
    dl_decision_t d = { .sequence = 0x01020304, .timestamp_ms = 0x05060708 };
    uint8_t buf[DL_WIRE_ON_WIRE_SIZE];
    dl_wire_encode(&d, buf, sizeof(buf));
    /* Magic 0x444C4B31 at [0..3] big-endian */
    DL_ASSERT_EQ(buf[0], 0x44);
    DL_ASSERT_EQ(buf[1], 0x4C);
    DL_ASSERT_EQ(buf[2], 0x4B);
    DL_ASSERT_EQ(buf[3], 0x31);
    /* sequence at [8..11] big-endian */
    DL_ASSERT_EQ(buf[8], 0x01);
    DL_ASSERT_EQ(buf[9], 0x02);
    DL_ASSERT_EQ(buf[10], 0x03);
    DL_ASSERT_EQ(buf[11], 0x04);
    /* timestamp_ms at [12..15] */
    DL_ASSERT_EQ(buf[12], 0x05);
    DL_ASSERT_EQ(buf[13], 0x06);
    DL_ASSERT_EQ(buf[14], 0x07);
    DL_ASSERT_EQ(buf[15], 0x08);
}

DL_TEST(test_wire_rejects_short) {
    dl_decision_t r;
    DL_ASSERT_EQ(dl_wire_decode((const uint8_t *)"short", 5, &r), DL_DECODE_SHORT);
}

DL_TEST(test_wire_rejects_bad_magic) {
    dl_decision_t d = {0};
    uint8_t buf[DL_WIRE_ON_WIRE_SIZE];
    dl_wire_encode(&d, buf, sizeof(buf));
    buf[0] = 0xFF;
    dl_decision_t r;
    DL_ASSERT_EQ(dl_wire_decode(buf, sizeof(buf), &r), DL_DECODE_BAD_MAGIC);
}

DL_TEST(test_wire_rejects_bad_version) {
    dl_decision_t d = {0};
    uint8_t buf[DL_WIRE_ON_WIRE_SIZE];
    dl_wire_encode(&d, buf, sizeof(buf));
    buf[4] = 99;  /* version byte */
    dl_decision_t r;
    DL_ASSERT_EQ(dl_wire_decode(buf, sizeof(buf), &r), DL_DECODE_BAD_VERSION);
}

DL_TEST(test_wire_rejects_bad_crc) {
    dl_decision_t d = { .mcs = 5, .k = 8, .n = 12, .depth = 1 };
    uint8_t buf[DL_WIRE_ON_WIRE_SIZE];
    dl_wire_encode(&d, buf, sizeof(buf));
    buf[20] ^= 0xFF;  /* corrupt a payload byte; CRC no longer matches */
    dl_decision_t r;
    DL_ASSERT_EQ(dl_wire_decode(buf, sizeof(buf), &r), DL_DECODE_BAD_CRC);
}

DL_TEST(test_wire_crc32_empty) {
    /* CRC32 of empty == 0 */
    DL_ASSERT_EQ(dl_wire_crc32((const uint8_t *)"", 0), 0u);
}

DL_TEST(test_wire_crc32_known) {
    /* "123456789" → 0xCBF43926 (standard IEEE 802.3 test vector). */
    uint32_t crc = dl_wire_crc32((const uint8_t *)"123456789", 9);
    DL_ASSERT_EQ(crc, 0xCBF43926u);
}

DL_TEST(test_wire_signed_tx_power) {
    dl_decision_t d = { .tx_power_dBm = -5 };
    uint8_t buf[DL_WIRE_ON_WIRE_SIZE];
    dl_wire_encode(&d, buf, sizeof(buf));
    DL_ASSERT_EQ(buf[18], 0xFB);  /* -5 as int8 two's complement */
    dl_decision_t r;
    DL_ASSERT_EQ(dl_wire_decode(buf, sizeof(buf), &r), DL_DECODE_OK);
    DL_ASSERT_EQ(r.tx_power_dBm, -5);
}
