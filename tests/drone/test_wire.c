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

DL_TEST(test_wire_ping_round_trip) {
    dl_ping_t p = { .flags = 0, .gs_seq = 0xCAFEBABEu,
                    .gs_mono_us = 0x0102030405060708ull };
    uint8_t buf[DL_PING_ON_WIRE_SIZE];
    DL_ASSERT_EQ(dl_wire_encode_ping(&p, buf, sizeof(buf)),
                 DL_PING_ON_WIRE_SIZE);

    /* Magic at [0..3] = 'DLPG' = 0x44 0x4C 0x50 0x47 */
    DL_ASSERT_EQ(buf[0], 0x44);
    DL_ASSERT_EQ(buf[1], 0x4C);
    DL_ASSERT_EQ(buf[2], 0x50);
    DL_ASSERT_EQ(buf[3], 0x47);

    /* gs_mono_us at [12..19] big-endian */
    DL_ASSERT_EQ(buf[12], 0x01);
    DL_ASSERT_EQ(buf[19], 0x08);

    dl_ping_t r;
    DL_ASSERT_EQ(dl_wire_decode_ping(buf, sizeof(buf), &r), DL_DECODE_OK);
    DL_ASSERT_EQ(r.gs_seq, p.gs_seq);
    DL_ASSERT_EQ(r.gs_mono_us, p.gs_mono_us);
}

DL_TEST(test_wire_pong_round_trip) {
    dl_pong_t p = {
        .flags = 0,
        .gs_seq = 7,
        .gs_mono_us_echo    = 1000000ull,
        .drone_mono_recv_us = 2000000ull,
        .drone_mono_send_us = 2000050ull,
    };
    uint8_t buf[DL_PONG_ON_WIRE_SIZE];
    DL_ASSERT_EQ(dl_wire_encode_pong(&p, buf, sizeof(buf)),
                 DL_PONG_ON_WIRE_SIZE);

    /* Magic 'DLPN' */
    DL_ASSERT_EQ(buf[0], 0x44);
    DL_ASSERT_EQ(buf[1], 0x4C);
    DL_ASSERT_EQ(buf[2], 0x50);
    DL_ASSERT_EQ(buf[3], 0x4E);

    dl_pong_t r;
    DL_ASSERT_EQ(dl_wire_decode_pong(buf, sizeof(buf), &r), DL_DECODE_OK);
    DL_ASSERT_EQ(r.gs_seq, p.gs_seq);
    DL_ASSERT_EQ(r.gs_mono_us_echo, p.gs_mono_us_echo);
    DL_ASSERT_EQ(r.drone_mono_recv_us, p.drone_mono_recv_us);
    DL_ASSERT_EQ(r.drone_mono_send_us, p.drone_mono_send_us);
}

DL_TEST(test_wire_peek_kind_dispatches) {
    dl_decision_t d = {0};
    uint8_t dbuf[DL_WIRE_ON_WIRE_SIZE];
    dl_wire_encode(&d, dbuf, sizeof(dbuf));
    DL_ASSERT_EQ(dl_wire_peek_kind(dbuf, sizeof(dbuf)), DL_PKT_DECISION);

    dl_ping_t p = {0};
    uint8_t pbuf[DL_PING_ON_WIRE_SIZE];
    dl_wire_encode_ping(&p, pbuf, sizeof(pbuf));
    DL_ASSERT_EQ(dl_wire_peek_kind(pbuf, sizeof(pbuf)), DL_PKT_PING);

    dl_pong_t pong = {0};
    uint8_t obuf[DL_PONG_ON_WIRE_SIZE];
    dl_wire_encode_pong(&pong, obuf, sizeof(obuf));
    DL_ASSERT_EQ(dl_wire_peek_kind(obuf, sizeof(obuf)), DL_PKT_PONG);

    uint8_t junk[8] = { 0xDE, 0xAD, 0xBE, 0xEF };
    DL_ASSERT_EQ(dl_wire_peek_kind(junk, sizeof(junk)), DL_PKT_UNKNOWN);
    DL_ASSERT_EQ(dl_wire_peek_kind(junk, 2), DL_PKT_UNKNOWN);
}

DL_TEST(test_wire_ping_rejects_bad_crc) {
    dl_ping_t p = { .gs_seq = 1, .gs_mono_us = 1 };
    uint8_t buf[DL_PING_ON_WIRE_SIZE];
    dl_wire_encode_ping(&p, buf, sizeof(buf));
    buf[10] ^= 0xFF;
    dl_ping_t r;
    DL_ASSERT_EQ(dl_wire_decode_ping(buf, sizeof(buf), &r), DL_DECODE_BAD_CRC);
}

DL_TEST(wire_hello_encode_decode_roundtrip) {
    dl_hello_t in = {
        .version = DL_WIRE_VERSION,
        .flags = 0,
        .generation_id = 0xCAFEBABEu,
        .mtu_bytes = 3994,
        .fps = 60,
        .applier_build_sha = 0xDEADBEEFu,
    };
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    size_t n = dl_wire_encode_hello(&in, buf, sizeof(buf));
    DL_ASSERT_EQ(n, DL_HELLO_ON_WIRE_SIZE);

    dl_hello_t out;
    dl_decode_result_t rc = dl_wire_decode_hello(buf, n, &out);
    DL_ASSERT_EQ(rc, DL_DECODE_OK);
    DL_ASSERT_EQ(out.generation_id, in.generation_id);
    DL_ASSERT_EQ(out.mtu_bytes, in.mtu_bytes);
    DL_ASSERT_EQ(out.fps, in.fps);
    DL_ASSERT_EQ(out.applier_build_sha, in.applier_build_sha);
}

DL_TEST(wire_hello_ack_encode_decode_roundtrip) {
    dl_hello_ack_t in = {
        .version = DL_WIRE_VERSION,
        .generation_id_echo = 0x12345678u,
    };
    uint8_t buf[DL_HELLO_ACK_ON_WIRE_SIZE];
    size_t n = dl_wire_encode_hello_ack(&in, buf, sizeof(buf));
    DL_ASSERT_EQ(n, DL_HELLO_ACK_ON_WIRE_SIZE);

    dl_hello_ack_t out;
    dl_decode_result_t rc = dl_wire_decode_hello_ack(buf, n, &out);
    DL_ASSERT_EQ(rc, DL_DECODE_OK);
    DL_ASSERT_EQ(out.generation_id_echo, in.generation_id_echo);
}

DL_TEST(wire_hello_bad_crc_rejected) {
    dl_hello_t in = { .version = DL_WIRE_VERSION, .generation_id = 1,
                      .mtu_bytes = 1400, .fps = 30, .applier_build_sha = 0 };
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    dl_wire_encode_hello(&in, buf, sizeof(buf));
    buf[DL_HELLO_ON_WIRE_SIZE - 1] ^= 0x01;  /* corrupt CRC */
    dl_hello_t out;
    DL_ASSERT_EQ(dl_wire_decode_hello(buf, sizeof(buf), &out),
                 DL_DECODE_BAD_CRC);
}

DL_TEST(wire_hello_bad_magic_rejected) {
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE] = {0};
    /* All-zero magic. */
    dl_hello_t out;
    DL_ASSERT_EQ(dl_wire_decode_hello(buf, sizeof(buf), &out),
                 DL_DECODE_BAD_MAGIC);
}

DL_TEST(wire_peek_kind_hello_ack) {
    dl_hello_ack_t in = { .version = DL_WIRE_VERSION,
                          .generation_id_echo = 7 };
    uint8_t buf[DL_HELLO_ACK_ON_WIRE_SIZE];
    dl_wire_encode_hello_ack(&in, buf, sizeof(buf));
    DL_ASSERT_EQ(dl_wire_peek_kind(buf, sizeof(buf)), DL_PKT_HELLO_ACK);
}

DL_TEST(wire_peek_kind_hello) {
    dl_hello_t in = { .version = DL_WIRE_VERSION, .generation_id = 1,
                      .mtu_bytes = 1400, .fps = 30 };
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    dl_wire_encode_hello(&in, buf, sizeof(buf));
    DL_ASSERT_EQ(dl_wire_peek_kind(buf, sizeof(buf)), DL_PKT_HELLO);
}

DL_TEST(wire_hello_ack_bad_crc_rejected) {
    dl_hello_ack_t in = { .generation_id_echo = 7 };
    uint8_t buf[DL_HELLO_ACK_ON_WIRE_SIZE];
    dl_wire_encode_hello_ack(&in, buf, sizeof(buf));
    buf[DL_HELLO_ACK_ON_WIRE_SIZE - 1] ^= 0x01;
    dl_hello_ack_t out;
    DL_ASSERT_EQ(dl_wire_decode_hello_ack(buf, sizeof(buf), &out),
                 DL_DECODE_BAD_CRC);
}

DL_TEST(wire_hello_ack_bad_magic_rejected) {
    uint8_t buf[DL_HELLO_ACK_ON_WIRE_SIZE] = {0};
    dl_hello_ack_t out;
    DL_ASSERT_EQ(dl_wire_decode_hello_ack(buf, sizeof(buf), &out),
                 DL_DECODE_BAD_MAGIC);
}

DL_TEST(wire_hello_short_buffer_rejected) {
    dl_hello_t in = { .version = DL_WIRE_VERSION, .generation_id = 1,
                      .mtu_bytes = 1400, .fps = 30 };
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    dl_wire_encode_hello(&in, buf, sizeof(buf));
    dl_hello_t out;
    DL_ASSERT_EQ(dl_wire_decode_hello(buf, DL_HELLO_ON_WIRE_SIZE - 1, &out),
                 DL_DECODE_SHORT);
}

DL_TEST(wire_hello_bad_version_rejected) {
    dl_hello_t in = { .version = DL_WIRE_VERSION, .generation_id = 1,
                      .mtu_bytes = 1400, .fps = 30 };
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    dl_wire_encode_hello(&in, buf, sizeof(buf));
    buf[4] = DL_WIRE_VERSION + 1;
    /* Recompute CRC so we test bad-version, not bad-crc. */
    uint32_t new_crc = dl_wire_crc32(buf, DL_HELLO_PAYLOAD_SIZE);
    buf[DL_HELLO_PAYLOAD_SIZE]     = (new_crc >> 24) & 0xFF;
    buf[DL_HELLO_PAYLOAD_SIZE + 1] = (new_crc >> 16) & 0xFF;
    buf[DL_HELLO_PAYLOAD_SIZE + 2] = (new_crc >> 8)  & 0xFF;
    buf[DL_HELLO_PAYLOAD_SIZE + 3] = new_crc & 0xFF;
    dl_hello_t out;
    DL_ASSERT_EQ(dl_wire_decode_hello(buf, sizeof(buf), &out),
                 DL_DECODE_BAD_VERSION);
}

DL_TEST(wire_hello_ack_short_buffer_rejected) {
    dl_hello_ack_t in = { .generation_id_echo = 1 };
    uint8_t buf[DL_HELLO_ACK_ON_WIRE_SIZE];
    dl_wire_encode_hello_ack(&in, buf, sizeof(buf));
    dl_hello_ack_t out;
    DL_ASSERT_EQ(dl_wire_decode_hello_ack(buf, DL_HELLO_ACK_ON_WIRE_SIZE - 1, &out),
                 DL_DECODE_SHORT);
}

DL_TEST(wire_hello_ack_bad_version_rejected) {
    dl_hello_ack_t in = { .generation_id_echo = 1 };
    uint8_t buf[DL_HELLO_ACK_ON_WIRE_SIZE];
    dl_wire_encode_hello_ack(&in, buf, sizeof(buf));
    buf[4] = DL_WIRE_VERSION + 1;
    uint32_t new_crc = dl_wire_crc32(buf, DL_HELLO_ACK_PAYLOAD_SIZE);
    buf[DL_HELLO_ACK_PAYLOAD_SIZE]     = (new_crc >> 24) & 0xFF;
    buf[DL_HELLO_ACK_PAYLOAD_SIZE + 1] = (new_crc >> 16) & 0xFF;
    buf[DL_HELLO_ACK_PAYLOAD_SIZE + 2] = (new_crc >> 8)  & 0xFF;
    buf[DL_HELLO_ACK_PAYLOAD_SIZE + 3] = new_crc & 0xFF;
    dl_hello_ack_t out;
    DL_ASSERT_EQ(dl_wire_decode_hello_ack(buf, sizeof(buf), &out),
                 DL_DECODE_BAD_VERSION);
}
