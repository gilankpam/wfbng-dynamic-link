/*
 * Vendored from wfb-ng/src/tx_cmd.h
 *
 * Source repo:  https://github.com/wfb-ng/wfb-ng
 * Source SHA:   208ec1d71e9d26bc8fbe57f5cb0903f968fafb06
 * Pinned at commit e71b317: "Phase 1 Step A: --interleave-depth flag
 * and SESSION plumbing" (feat/interleaving_uep branch).
 *
 * Changes from upstream:
 *   - Added #include <stdbool.h> (upstream relies on includes from its
 *     .cpp callers; we want this header to compile standalone).
 *
 * Refresh procedure: see ./README.md.
 *
 * Contract version check: WFB_IPC_CONTRACT_VERSION lives in
 * wfb-ng/src/wifibroadcast.hpp; the applier cross-checks the value at
 * startup (see dl_backend_tx.c).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CMD_SET_FEC                 1
#define CMD_SET_RADIO               2
#define CMD_GET_FEC                 3
#define CMD_GET_RADIO               4
// Phase 1: interleaver-depth runtime control. Set drains the current
// FEC block and the interleaver, then re-broadcasts SESSION with the
// new depth on the same session_key (plan v2.1 R1 "option 1C").
// Stub handlers in Step A return EINVAL; real semantics arrive in
// Step C.
#define CMD_SET_INTERLEAVE_DEPTH    5
#define CMD_GET_INTERLEAVE_DEPTH    6

typedef struct {
    uint32_t req_id;
    uint8_t cmd_id;
    union {
        struct
        {
            uint8_t k;
            uint8_t n;
        } __attribute__ ((packed)) cmd_set_fec;

        struct
        {
            uint8_t stbc;
            bool ldpc;
            bool short_gi;
            uint8_t bandwidth;
            uint8_t mcs_index;
            bool vht_mode;
            uint8_t vht_nss;
        } __attribute__ ((packed)) cmd_set_radio;

        struct
        {
            uint8_t depth;  // 1..255
        } __attribute__ ((packed)) cmd_set_interleave_depth;
    } __attribute__ ((packed)) u;
} __attribute__ ((packed)) cmd_req_t;


typedef struct {
    uint32_t req_id;
    uint32_t rc;
    union {
        struct
        {
            uint8_t k;
            uint8_t n;
        } __attribute__ ((packed)) cmd_get_fec;

        struct
        {
            uint8_t stbc;
            bool ldpc;
            bool short_gi;
            uint8_t bandwidth;
            uint8_t mcs_index;
            bool vht_mode;
            uint8_t vht_nss;
        } __attribute__ ((packed)) cmd_get_radio;

        struct
        {
            uint8_t depth;
        } __attribute__ ((packed)) cmd_get_interleave_depth;
    } __attribute__ ((packed)) u;
} __attribute__ ((packed)) cmd_resp_t;
