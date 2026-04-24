/* dl_wire.c — encode/decode the 28-byte dl_decision wire packet.
 *
 * Layout (big-endian / network byte order):
 *
 *   off  size  field
 *    0    4    magic       = 0x444C4B31 ("DLK1")
 *    4    1    version     = 1
 *    5    1    flags
 *    6    2    _pad (0)
 *    8    4    sequence
 *   12    4    timestamp_ms
 *   16    1    mcs
 *   17    1    bandwidth
 *   18    1    tx_power_dBm (signed)
 *   19    1    k
 *   20    1    n
 *   21    1    depth
 *   22    2    bitrate_kbps
 *   24    1    roi_qp
 *   25    1    fps
 *   26    2    _pad2 (0)
 *   28                        — CRC32 is NOT a struct field; the decoder
 *                               validates it over bytes [0..27] EXCEPT
 *                               the 4-byte trailer? No — simpler:
 *                               the on-wire packet is actually 32 bytes;
 *                               bytes [0..27] are the payload and [28..31]
 *                               are crc32(payload).
 *
 * NOTE: DL_WIRE_SIZE = 28 names the payload size. Callers must pass a
 * 32-byte buffer for encode, and at least 32 bytes for decode. See
 * DL_WIRE_ON_WIRE_SIZE below.
 */
#include "dl_wire.h"

#include <string.h>

/* Big-endian helpers. */
static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8) & 0xFF);
    p[3] = (uint8_t)(v & 0xFF);
}
static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}
static uint32_t get_u32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24)
         | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)
         |  (uint32_t)p[3];
}

/* Table-free CRC-32 (IEEE 802.3). Small, slow — we process <32 bytes
 * per packet at ≤ 10 Hz, so a table buys nothing meaningful. */
uint32_t dl_wire_crc32(const uint8_t *buf, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int b = 0; b < 8; ++b) {
            uint32_t mask = -(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

size_t dl_wire_encode(const dl_decision_t *d, uint8_t *buf, size_t buflen) {
    if (buflen < DL_WIRE_ON_WIRE_SIZE) return 0;

    memset(buf, 0, DL_WIRE_ON_WIRE_SIZE);
    put_u32(&buf[0], DL_WIRE_MAGIC);
    buf[4] = DL_WIRE_VERSION;
    buf[5] = d->flags;
    /* buf[6..7] = _pad */
    put_u32(&buf[8],  d->sequence);
    put_u32(&buf[12], d->timestamp_ms);
    buf[16] = d->mcs;
    buf[17] = d->bandwidth;
    buf[18] = (uint8_t)d->tx_power_dBm; /* signed-to-unsigned via cast */
    buf[19] = d->k;
    buf[20] = d->n;
    buf[21] = d->depth;
    put_u16(&buf[22], d->bitrate_kbps);
    buf[24] = d->roi_qp;
    buf[25] = d->fps;
    /* buf[26..27] = _pad2 */
    uint32_t crc = dl_wire_crc32(buf, DL_WIRE_PAYLOAD_SIZE);
    put_u32(&buf[28], crc);
    return DL_WIRE_ON_WIRE_SIZE;
}

dl_decode_result_t dl_wire_decode(const uint8_t *buf, size_t len,
                                  dl_decision_t *d) {
    if (len < DL_WIRE_ON_WIRE_SIZE) return DL_DECODE_SHORT;

    uint32_t magic = get_u32(&buf[0]);
    if (magic != DL_WIRE_MAGIC) return DL_DECODE_BAD_MAGIC;

    uint8_t version = buf[4];
    if (version != DL_WIRE_VERSION) return DL_DECODE_BAD_VERSION;

    uint32_t crc_wire = get_u32(&buf[28]);
    uint32_t crc_calc = dl_wire_crc32(buf, DL_WIRE_PAYLOAD_SIZE);
    if (crc_wire != crc_calc) return DL_DECODE_BAD_CRC;

    memset(d, 0, sizeof(*d));
    d->magic        = magic;
    d->version      = version;
    d->flags        = buf[5];
    d->sequence     = get_u32(&buf[8]);
    d->timestamp_ms = get_u32(&buf[12]);
    d->mcs          = buf[16];
    d->bandwidth    = buf[17];
    d->tx_power_dBm = (int8_t)buf[18];
    d->k            = buf[19];
    d->n            = buf[20];
    d->depth        = buf[21];
    d->bitrate_kbps = get_u16(&buf[22]);
    d->roi_qp       = buf[24];
    d->fps          = buf[25];
    return DL_DECODE_OK;
}
