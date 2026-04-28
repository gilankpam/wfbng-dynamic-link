/* dl_wire.h — GS → drone decision-packet wire format.
 *
 * Fixed-size big-endian struct carried as one UDP datagram over the
 * wfb-ng `tunnel` stream. Contract version 1.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define DL_WIRE_MAGIC           0x444C4B31u   /* "DLK1" */
#define DL_WIRE_VERSION         1
#define DL_WIRE_PAYLOAD_SIZE    28            /* payload bytes */
#define DL_WIRE_ON_WIRE_SIZE    32            /* payload + 4-byte CRC */

/* Phase 3: timesync ping/pong over the same tunnel UDP socket. */
#define DL_PING_MAGIC           0x444C5047u   /* "DLPG" */
#define DL_PING_PAYLOAD_SIZE    20
#define DL_PING_ON_WIRE_SIZE    24            /* payload + 4-byte CRC */

#define DL_PONG_MAGIC           0x444C504Eu   /* "DLPN" */
#define DL_PONG_PAYLOAD_SIZE    36
#define DL_PONG_ON_WIRE_SIZE    40            /* payload + 4-byte CRC */

/* flag bits */
#define DL_FLAG_IDR_REQUEST 0x01u

/* Parsed, host-byte-order representation of a decision packet. */
typedef struct {
    uint32_t magic;            /* always DL_WIRE_MAGIC after decode */
    uint8_t  version;
    uint8_t  flags;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint8_t  mcs;              /* 0..7 */
    uint8_t  bandwidth;        /* 20 or 40 */
    int8_t   tx_power_dBm;
    uint8_t  k;                /* 1..8 */
    uint8_t  n;                /* 4..16 typical */
    uint8_t  depth;            /* 1..3 typical */
    uint16_t bitrate_kbps;     /* 0..65535 */
    uint8_t  roi_qp;           /* 0 = unset */
    uint8_t  fps;              /* 0 = unset */
} dl_decision_t;

typedef enum {
    DL_DECODE_OK = 0,
    DL_DECODE_SHORT,           /* fewer than DL_WIRE_SIZE bytes */
    DL_DECODE_BAD_MAGIC,
    DL_DECODE_BAD_VERSION,
    DL_DECODE_BAD_CRC,
} dl_decode_result_t;

/* Encode `d` into `buf` (must be at least DL_WIRE_SIZE bytes).
 * Returns DL_WIRE_SIZE on success. */
size_t dl_wire_encode(const dl_decision_t *d, uint8_t *buf, size_t buflen);

/* Decode `buf` of `len` bytes into `d`. Returns DL_DECODE_OK or a
 * specific failure code. `d` is only populated on success. */
dl_decode_result_t dl_wire_decode(const uint8_t *buf, size_t len,
                                  dl_decision_t *d);

/* CRC-32 (IEEE 802.3, polynomial 0xEDB88320, reflected, init 0xFFFFFFFF,
 * final XOR 0xFFFFFFFF). Exposed for tests. */
uint32_t dl_wire_crc32(const uint8_t *buf, size_t len);

/* ---- Ping/pong (Phase 3, timesync) ---------------------------------- */

typedef struct {
    uint32_t magic;            /* DL_PING_MAGIC after decode */
    uint8_t  version;
    uint8_t  flags;
    uint32_t gs_seq;
    uint64_t gs_mono_us;
} dl_ping_t;

typedef struct {
    uint32_t magic;            /* DL_PONG_MAGIC after decode */
    uint8_t  version;
    uint8_t  flags;
    uint32_t gs_seq;
    uint64_t gs_mono_us_echo;
    uint64_t drone_mono_recv_us;
    uint64_t drone_mono_send_us;
} dl_pong_t;

/* All four directions (encode+decode on both sides) so the C/Python
 * hex-diff contract stays symmetric. The drone runtime uses
 * decode_ping + encode_pong; the GS runtime uses encode_ping +
 * decode_pong; tests exercise the unused half. */
size_t dl_wire_encode_ping(const dl_ping_t *p, uint8_t *buf, size_t buflen);
dl_decode_result_t dl_wire_decode_ping(const uint8_t *buf, size_t len,
                                       dl_ping_t *p);

size_t dl_wire_encode_pong(const dl_pong_t *p, uint8_t *buf, size_t buflen);
dl_decode_result_t dl_wire_decode_pong(const uint8_t *buf, size_t len,
                                       dl_pong_t *p);

/* Quick magic-peek for the recv-path dispatcher. Returns one of the
 * three known magics or 0 for unknown. Does not validate length or
 * CRC — callers must follow up with the full decoder. */
typedef enum {
    DL_PKT_UNKNOWN  = 0,
    DL_PKT_DECISION = 1,
    DL_PKT_PING     = 2,
    DL_PKT_PONG     = 3,
} dl_packet_kind_t;

dl_packet_kind_t dl_wire_peek_kind(const uint8_t *buf, size_t len);
