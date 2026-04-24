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
