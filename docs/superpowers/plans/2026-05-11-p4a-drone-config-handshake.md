# P4a — Drone Config Handshake Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a drone→GS config handshake so the GS learns the drone's authoritative `mtu_bytes` (from `/etc/wfb.yaml` `wireless.mlink`) and `fps` (from `/etc/majestic.yaml` `video0.fps`) at startup, with reboot detection via `generation_id`. GS holds `safe_defaults` until the first HELLO+ACK round-trip completes.

**Architecture:** Two new packet types on the existing wfb-ng tunnel UDP path — `DLHE` (drone→GS, 32 B) and `DLHA` (GS→drone, 32 B), mirroring the `DLK1`/`DLPG`/`DLPN` precedent (magic, version, CRC32). New drone module `dl_hello` runs a small state machine (ANNOUNCING → KEEPALIVE) driven by a `timerfd`. New GS module `drone_config` tracks `(generation_id, mtu, fps)` and gates the policy's emit path.

**Scope of P4a:** Wire-format plumbing + state machines + GS-side gate (safe_defaults until synced). **No FEC algorithm changes.** The existing `fec_table` lookup stays. Dynamic FEC computation is P4b.

**Tech Stack:** C11 (drone, POSIX timerfd, no libraries), Python 3.11+ (GS, stdlib asyncio + pytest).

**Spec:** `docs/superpowers/specs/2026-05-11-drone-config-handshake-and-dynamic-fec-design.md`.

---

## File Structure

### Created
- `drone/src/dl_yaml_get.h` / `dl_yaml_get.c` — minimal "find key under block" parser for two specific YAML shapes.
- `drone/src/dl_hello.h` / `dl_hello.c` — drone-side state machine, timer, packet send/recv.
- `tests/drone/test_dl_yaml.c` — unit tests for the YAML parser.
- `tests/drone/test_dl_hello.c` — state machine unit tests.
- `tests/drone/fixtures/wfb_basic.yaml` — sample `/etc/wfb.yaml`-style fixture (the one from the spec).
- `tests/drone/fixtures/wfb_no_mlink.yaml`, `wfb_malformed.yaml`, `wfb_with_comments.yaml` — mutated variants.
- `tests/drone/fixtures/majestic_basic.yaml` — sample `/etc/majestic.yaml`-style fixture.
- `gs/dynamic_link/drone_config.py` — GS-side state machine + handler.
- `tests/test_drone_config.py` — GS-side state machine unit tests.

### Modified
- `drone/src/dl_wire.h` / `dl_wire.c` — add `dl_hello_t`, `dl_hello_ack_t`, encoders/decoders, magic constants, `peek_kind` enum.
- `drone/src/dl_inject.c` — add `--hello`, `--hello-ack`, `--gen-id`, `--mtu`, `--fps`, `--build-sha` flags for dry-run contract testing.
- `drone/src/dl_applier.c` — wire `dl_hello` into the poll loop (4th fd: hello timer; dispatch `DL_PKT_HELLO_ACK` from listen socket).
- `drone/src/dl_config.c` / `dl_config.h` — add `hello_announce_initial_ms`, `hello_announce_steady_ms`, `hello_keepalive_ms`, `hello_wfb_yaml_path`, `hello_majestic_yaml_path` (so tests can override paths).
- `drone/Makefile` — add new sources to `APPLIER_SRCS`, `INJECT_SRCS`, `TEST_SRCS`.
- `gs/dynamic_link/wire.py` — add `Hello`, `HelloAck` dataclasses + encoders/decoders + `peek_kind` cases.
- `gs/dynamic_link/tunnel_listener.py` — dispatch `DLHE` to a new `HelloHandler`.
- `gs/dynamic_link/return_link.py` — add `send_hello_ack(bytes)` method.
- `gs/dynamic_link/policy.py` — `Policy.tick()` returns `safe_defaults`-shaped Decision when `DroneConfigState` is in AWAITING.
- `gs/dynamic_link/service.py` — wire the `DroneConfigState`, register HELLO handler with the listener, pass to `Policy`.
- `conf/drone.conf.sample` — document new `hello.*` keys.
- `tests/test_wire_contract.py` — extend with HELLO / HELLO-ACK contract tests.
- `tests/test_drone_e2e.py` — handshake-then-decide path; drone-restart path; bad-YAML path.

---

## Task 1: `dl_yaml_get` parser

**Files:**
- Create: `drone/src/dl_yaml_get.h`
- Create: `drone/src/dl_yaml_get.c`
- Create: `tests/drone/test_dl_yaml.c`
- Create: `tests/drone/fixtures/wfb_basic.yaml`
- Create: `tests/drone/fixtures/wfb_no_mlink.yaml`
- Create: `tests/drone/fixtures/wfb_malformed.yaml`
- Create: `tests/drone/fixtures/wfb_with_comments.yaml`
- Create: `tests/drone/fixtures/majestic_basic.yaml`
- Modify: `drone/Makefile`

- [ ] **Step 1: Write fixture files**

`tests/drone/fixtures/wfb_basic.yaml`:
```yaml
wireless:
  txpower: 50
  channel: 132
  width: 20
  mlink: 3994
  wlan_adapter: bl-m8812eu2
  link_control: alink
broadcast:
  mcs_index: 4
  tun_index: 1
  fec_k: 10
  fec_n: 15
  stbc: 1
  ldpc: 1
  link_id: 7669206
telemetry:
  router: msposd
  serial: ttyS2
  osd_fps: 20
```

`tests/drone/fixtures/wfb_no_mlink.yaml`:
```yaml
wireless:
  txpower: 50
  channel: 132
  width: 20
broadcast:
  mcs_index: 4
```

`tests/drone/fixtures/wfb_malformed.yaml`:
```yaml
this is not yaml at all
just some random text
mlink: 9999
```

`tests/drone/fixtures/wfb_with_comments.yaml`:
```yaml
# top-level comment
wireless:
  # block comment
  txpower: 50
  mlink: 4096  # inline comment
broadcast:
  mcs_index: 4
```

`tests/drone/fixtures/majestic_basic.yaml`:
```yaml
system:
  logLevel: info
video0:
  enabled: true
  codec: h265
  size: 1920x1080
  fps: 60
  bitrate: 8192
records:
  enabled: false
```

- [ ] **Step 2: Write failing tests**

`tests/drone/test_dl_yaml.c`:
```c
#include "dl_yaml_get.h"
#include "test_main.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#define FIX(name) "tests/drone/fixtures/" name

DL_TEST(yaml_reads_mlink_from_wfb_basic) {
    int v = -1;
    int rc = dl_yaml_get_int(FIX("wfb_basic.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 3994);
}

DL_TEST(yaml_reads_fps_from_majestic_basic) {
    int v = -1;
    int rc = dl_yaml_get_int(FIX("majestic_basic.yaml"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 60);
}

DL_TEST(yaml_returns_einval_when_key_missing) {
    int v = 999;
    int rc = dl_yaml_get_int(FIX("wfb_no_mlink.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
    DL_ASSERT_EQ(v, 999);  /* unchanged on failure */
}

DL_TEST(yaml_returns_einval_when_block_missing) {
    int v = 0;
    int rc = dl_yaml_get_int(FIX("wfb_no_mlink.yaml"), "telemetry", "router", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(yaml_returns_enoent_when_file_missing) {
    int v = 0;
    int rc = dl_yaml_get_int(FIX("does_not_exist.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, -ENOENT);
}

DL_TEST(yaml_handles_comments_and_inline) {
    int v = -1;
    int rc = dl_yaml_get_int(FIX("wfb_with_comments.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 4096);
}

DL_TEST(yaml_rejects_malformed_file) {
    int v = 0;
    int rc = dl_yaml_get_int(FIX("wfb_malformed.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(yaml_ignores_lookalike_key_at_wrong_indent) {
    /* A top-level "mlink:" line outside a "wireless:" block must not
     * match. The malformed fixture has exactly this case. */
    int v = 0;
    int rc = dl_yaml_get_int(FIX("wfb_malformed.yaml"), "wireless", "mlink", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}
```

- [ ] **Step 3: Verify tests fail to compile**

Run: `make -C drone test 2>&1 | head -20`
Expected: `dl_yaml_get.h: No such file or directory`

- [ ] **Step 4: Implement the header**

`drone/src/dl_yaml_get.h`:
```c
/* dl_yaml_get.h — tiny line-scan helper for finding `<key>: <integer>`
 * inside a top-level `<block>:` section. Robust enough for the two
 * specific YAML shapes we read (/etc/wfb.yaml, /etc/majestic.yaml).
 * Not a general YAML parser. */
#pragma once

/* Find `key: <int>` indented under top-level block `block:` in `path`.
 * On success, writes the parsed integer into *out and returns 0.
 * On failure returns a negative errno-ish code: -ENOENT if file not
 * found, -EINVAL for any structural / parsing problem (block not
 * found, key not found inside block, non-integer value, etc.).
 * `*out` is only written on success. */
int dl_yaml_get_int(const char *path,
                    const char *block,
                    const char *key,
                    int *out);
```

- [ ] **Step 5: Implement the parser**

`drone/src/dl_yaml_get.c`:
```c
/* dl_yaml_get.c — see dl_yaml_get.h. */
#include "dl_yaml_get.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 512

/* Returns 1 if `line` begins with `block:` in column 0 (i.e. the
 * first non-whitespace byte is at column 0 and the colon comes
 * right after `block`). */
static int is_block_header(const char *line, const char *block) {
    size_t blen = strlen(block);
    if (strncmp(line, block, blen) != 0) return 0;
    if (line[blen] != ':') return 0;
    /* Anything after `:` should be whitespace or comment or EOL. */
    const char *p = line + blen + 1;
    while (*p == ' ' || *p == '\t') p++;
    return (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#');
}

/* Strip trailing CR / LF / inline-`#`-comment. Operates in place. */
static void strip_eol_and_comment(char *line) {
    char *p = line;
    while (*p) {
        if (*p == '\n' || *p == '\r' || *p == '#') {
            *p = '\0';
            return;
        }
        p++;
    }
}

/* Return 1 if `line` starts with whitespace (indented) and contains
 * `<key>:` after the leading whitespace, *not* deeper than one
 * nesting level (we don't try to track nesting depth — just refuse
 * lines with no leading whitespace). On match, parse the value as a
 * base-10 integer and write to *out; return 1 on full match (key &
 * integer parse OK), 0 if not the key, -1 if key matched but value
 * is not an integer. */
static int try_match_key_line(const char *line, const char *key,
                              int *out) {
    /* Require at least one leading space — i.e. line is indented
     * under a block. */
    if (*line != ' ' && *line != '\t') return 0;
    const char *p = line;
    while (*p == ' ' || *p == '\t') p++;
    size_t klen = strlen(key);
    if (strncmp(p, key, klen) != 0) return 0;
    if (p[klen] != ':') return 0;
    p += klen + 1;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') return -1;
    char *endp = NULL;
    errno = 0;
    long v = strtol(p, &endp, 10);
    if (errno != 0 || endp == p) return -1;
    /* Trailing must be whitespace or end. */
    while (*endp == ' ' || *endp == '\t') endp++;
    if (*endp != '\0') return -1;
    if (v < INT_MIN || v > INT_MAX) return -1;
    *out = (int)v;
    return 1;
}

int dl_yaml_get_int(const char *path,
                    const char *block,
                    const char *key,
                    int *out) {
    FILE *fp = fopen(path, "r");
    if (!fp) return -ENOENT;

    char line[MAX_LINE];
    int in_block = 0;
    int rc = -EINVAL;

    while (fgets(line, sizeof(line), fp)) {
        strip_eol_and_comment(line);
        /* Skip blank lines. */
        const char *q = line;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == '\0') continue;

        if (is_block_header(line, block)) {
            in_block = 1;
            continue;
        }
        /* A column-0 line that's not our block header ends the block. */
        if (line[0] != ' ' && line[0] != '\t') {
            in_block = 0;
            continue;
        }
        if (!in_block) continue;

        int v = 0;
        int m = try_match_key_line(line, key, &v);
        if (m == 1) {
            *out = v;
            rc = 0;
            break;
        }
        if (m == -1) {
            rc = -EINVAL;
            break;
        }
    }
    fclose(fp);
    return rc;
}
```

Note: add `#include <limits.h>` at the top for `INT_MIN`/`INT_MAX`.

- [ ] **Step 6: Add to Makefile**

Modify `drone/Makefile` line 18-30 (APPLIER_SRCS) to include `$(SRCDIR)/dl_yaml_get.c`. Modify `TEST_SRCS` (line 38-53) to include both `$(SRCDIR)/dl_yaml_get.c` and `$(TESTDIR)/test_dl_yaml.c`.

```makefile
APPLIER_SRCS := \
    $(SRCDIR)/dl_applier.c \
    $(SRCDIR)/dl_backend_enc.c \
    $(SRCDIR)/dl_backend_radio.c \
    $(SRCDIR)/dl_backend_tx.c \
    $(SRCDIR)/dl_config.c \
    $(SRCDIR)/dl_dbg.c \
    $(SRCDIR)/dl_dedup.c \
    $(SRCDIR)/dl_log.c \
    $(SRCDIR)/dl_mavlink.c \
    $(SRCDIR)/dl_osd.c \
    $(SRCDIR)/dl_watchdog.c \
    $(SRCDIR)/dl_wire.c \
    $(SRCDIR)/dl_yaml_get.c
```

```makefile
TEST_SRCS := \
    $(TESTDIR)/test_main.c \
    $(TESTDIR)/test_apply_stagger.c \
    $(TESTDIR)/test_wire.c \
    $(TESTDIR)/test_config.c \
    $(TESTDIR)/test_dbg.c \
    $(TESTDIR)/test_dedup.c \
    $(TESTDIR)/test_mavlink.c \
    $(TESTDIR)/test_watchdog.c \
    $(TESTDIR)/test_dl_yaml.c \
    $(SRCDIR)/dl_wire.c \
    $(SRCDIR)/dl_config.c \
    $(SRCDIR)/dl_dbg.c \
    $(SRCDIR)/dl_dedup.c \
    $(SRCDIR)/dl_log.c \
    $(SRCDIR)/dl_mavlink.c \
    $(SRCDIR)/dl_watchdog.c \
    $(SRCDIR)/dl_yaml_get.c
```

- [ ] **Step 7: Run tests and verify pass**

Run: `make -C drone test`
Expected: All `yaml_*` tests pass (8 new tests). All existing tests still pass.

- [ ] **Step 8: Commit**

```bash
git add drone/src/dl_yaml_get.h drone/src/dl_yaml_get.c \
        tests/drone/test_dl_yaml.c tests/drone/fixtures/ \
        drone/Makefile
git commit -m "$(cat <<'EOF'
P4a: dl_yaml_get — line-scan parser for /etc/wfb.yaml + /etc/majestic.yaml

Tiny helper that reads `<block>.<key>: <integer>` shapes. Robust to
comments, inline `#`, CRLF, blank lines. Refuses anything weirder —
the goal is "read two known files," not "parse YAML." Five fixture
files cover the happy path plus mutated variants.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: HELLO + HELLO-ACK wire format (C side)

**Files:**
- Modify: `drone/src/dl_wire.h`
- Modify: `drone/src/dl_wire.c`
- Modify: `tests/drone/test_wire.c`

- [ ] **Step 1: Add the failing test first**

Append to `tests/drone/test_wire.c`:
```c
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
        .flags = 0,
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
```

- [ ] **Step 2: Run tests to verify failure**

Run: `make -C drone test 2>&1 | head -10`
Expected: compile error — `dl_hello_t` undefined, `DL_HELLO_ON_WIRE_SIZE` undefined, etc.

- [ ] **Step 3: Extend `dl_wire.h` with new types**

In `drone/src/dl_wire.h`, after the existing `DL_PONG_*` constants (around line 24) add:

```c
/* P4a: drone-reported config (mtu, fps, generation_id). */
#define DL_HELLO_MAGIC          0x444C4845u   /* "DLHE" */
#define DL_HELLO_PAYLOAD_SIZE   28
#define DL_HELLO_ON_WIRE_SIZE   32            /* payload + 4-byte CRC */

#define DL_HELLO_ACK_MAGIC          0x444C4841u   /* "DLHA" */
#define DL_HELLO_ACK_PAYLOAD_SIZE   28
#define DL_HELLO_ACK_ON_WIRE_SIZE   32            /* payload + 4-byte CRC */
```

In the same file, after the `dl_pong_t` struct (around line 86), add:

```c
typedef struct {
    uint32_t magic;             /* DL_HELLO_MAGIC after decode */
    uint8_t  version;
    uint8_t  flags;
    uint32_t generation_id;     /* random per drone boot */
    uint16_t mtu_bytes;
    uint16_t fps;
    uint32_t applier_build_sha; /* low 4 bytes of git SHA, debug only */
} dl_hello_t;

typedef struct {
    uint32_t magic;                  /* DL_HELLO_ACK_MAGIC after decode */
    uint8_t  version;
    uint8_t  flags;
    uint32_t generation_id_echo;     /* copied from acked HELLO */
} dl_hello_ack_t;

size_t dl_wire_encode_hello(const dl_hello_t *h, uint8_t *buf, size_t buflen);
dl_decode_result_t dl_wire_decode_hello(const uint8_t *buf, size_t len,
                                        dl_hello_t *h);

size_t dl_wire_encode_hello_ack(const dl_hello_ack_t *h, uint8_t *buf, size_t buflen);
dl_decode_result_t dl_wire_decode_hello_ack(const uint8_t *buf, size_t len,
                                            dl_hello_ack_t *h);
```

Extend the `dl_packet_kind_t` enum:

```c
typedef enum {
    DL_PKT_UNKNOWN     = 0,
    DL_PKT_DECISION    = 1,
    DL_PKT_PING        = 2,
    DL_PKT_PONG        = 3,
    DL_PKT_HELLO       = 4,
    DL_PKT_HELLO_ACK   = 5,
} dl_packet_kind_t;
```

- [ ] **Step 4: Extend `dl_wire.c` with the encoders / decoders**

Append to `drone/src/dl_wire.c` (after the existing pong encoder/decoder):

```c
/* ---- HELLO / HELLO_ACK (P4a) --------------------------------------- */
/* DL_HELLO layout (big-endian, 32 bytes = 28 payload + 4 CRC):
 *    0  4  magic = 0x444C4845 ('DLHE')
 *    4  1  version
 *    5  1  flags
 *    6  2  _pad
 *    8  4  generation_id
 *   12  2  mtu_bytes
 *   14  2  fps
 *   16  4  applier_build_sha
 *   20  8  reserved (zero)
 *   28  4  crc32(bytes[0..27])
 *
 * DL_HELLO_ACK layout (big-endian, 32 bytes = 28 payload + 4 CRC):
 *    0  4  magic = 0x444C4841 ('DLHA')
 *    4  1  version
 *    5  3  _pad
 *    8  4  generation_id_echo
 *   12  16 reserved (zero)
 *   28  4  crc32(bytes[0..27])
 */

size_t dl_wire_encode_hello(const dl_hello_t *h, uint8_t *buf, size_t buflen) {
    if (buflen < DL_HELLO_ON_WIRE_SIZE) return 0;
    memset(buf, 0, DL_HELLO_ON_WIRE_SIZE);
    write_u32(buf + 0, DL_HELLO_MAGIC);
    buf[4] = h->version;
    buf[5] = h->flags;
    /* [6..7] _pad already zero */
    write_u32(buf + 8,  h->generation_id);
    write_u16(buf + 12, h->mtu_bytes);
    write_u16(buf + 14, h->fps);
    write_u32(buf + 16, h->applier_build_sha);
    /* [20..27] reserved already zero */
    uint32_t crc = dl_wire_crc32(buf, DL_HELLO_PAYLOAD_SIZE);
    write_u32(buf + DL_HELLO_PAYLOAD_SIZE, crc);
    return DL_HELLO_ON_WIRE_SIZE;
}

dl_decode_result_t dl_wire_decode_hello(const uint8_t *buf, size_t len,
                                        dl_hello_t *h) {
    if (len < DL_HELLO_ON_WIRE_SIZE) return DL_DECODE_SHORT;
    uint32_t magic = read_u32(buf + 0);
    if (magic != DL_HELLO_MAGIC) return DL_DECODE_BAD_MAGIC;
    if (buf[4] != DL_WIRE_VERSION) return DL_DECODE_BAD_VERSION;
    uint32_t crc_wire = read_u32(buf + DL_HELLO_PAYLOAD_SIZE);
    uint32_t crc_calc = dl_wire_crc32(buf, DL_HELLO_PAYLOAD_SIZE);
    if (crc_wire != crc_calc) return DL_DECODE_BAD_CRC;
    h->magic = magic;
    h->version = buf[4];
    h->flags = buf[5];
    h->generation_id = read_u32(buf + 8);
    h->mtu_bytes = read_u16(buf + 12);
    h->fps = read_u16(buf + 14);
    h->applier_build_sha = read_u32(buf + 16);
    return DL_DECODE_OK;
}

size_t dl_wire_encode_hello_ack(const dl_hello_ack_t *h, uint8_t *buf, size_t buflen) {
    if (buflen < DL_HELLO_ACK_ON_WIRE_SIZE) return 0;
    memset(buf, 0, DL_HELLO_ACK_ON_WIRE_SIZE);
    write_u32(buf + 0, DL_HELLO_ACK_MAGIC);
    buf[4] = h->version;
    /* [5..7] _pad already zero */
    write_u32(buf + 8, h->generation_id_echo);
    /* [12..27] reserved already zero */
    uint32_t crc = dl_wire_crc32(buf, DL_HELLO_ACK_PAYLOAD_SIZE);
    write_u32(buf + DL_HELLO_ACK_PAYLOAD_SIZE, crc);
    return DL_HELLO_ACK_ON_WIRE_SIZE;
}

dl_decode_result_t dl_wire_decode_hello_ack(const uint8_t *buf, size_t len,
                                            dl_hello_ack_t *h) {
    if (len < DL_HELLO_ACK_ON_WIRE_SIZE) return DL_DECODE_SHORT;
    uint32_t magic = read_u32(buf + 0);
    if (magic != DL_HELLO_ACK_MAGIC) return DL_DECODE_BAD_MAGIC;
    if (buf[4] != DL_WIRE_VERSION) return DL_DECODE_BAD_VERSION;
    uint32_t crc_wire = read_u32(buf + DL_HELLO_ACK_PAYLOAD_SIZE);
    uint32_t crc_calc = dl_wire_crc32(buf, DL_HELLO_ACK_PAYLOAD_SIZE);
    if (crc_wire != crc_calc) return DL_DECODE_BAD_CRC;
    h->magic = magic;
    h->version = buf[4];
    h->flags = 0;
    h->generation_id_echo = read_u32(buf + 8);
    return DL_DECODE_OK;
}
```

If `read_u32` / `write_u32` / `read_u16` / `write_u16` are static helpers in `dl_wire.c`, reuse them as-is; if they're not, look at how `dl_wire_encode_ping` reads/writes — replicate the same pattern (it uses inline byte arithmetic or `htonl` — match the existing style).

- [ ] **Step 5: Extend `dl_wire_peek_kind`**

In `dl_wire.c`, find the existing `dl_wire_peek_kind` function and extend it:

```c
dl_packet_kind_t dl_wire_peek_kind(const uint8_t *buf, size_t len) {
    if (len < 4) return DL_PKT_UNKNOWN;
    uint32_t m = read_u32(buf);
    if (m == DL_WIRE_MAGIC)      return DL_PKT_DECISION;
    if (m == DL_PING_MAGIC)      return DL_PKT_PING;
    if (m == DL_PONG_MAGIC)      return DL_PKT_PONG;
    if (m == DL_HELLO_MAGIC)     return DL_PKT_HELLO;
    if (m == DL_HELLO_ACK_MAGIC) return DL_PKT_HELLO_ACK;
    return DL_PKT_UNKNOWN;
}
```

- [ ] **Step 6: Run tests to verify pass**

Run: `make -C drone test`
Expected: All `wire_hello_*` and `wire_peek_kind_hello*` tests pass. All existing tests still pass.

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_wire.h drone/src/dl_wire.c tests/drone/test_wire.c
git commit -m "$(cat <<'EOF'
P4a: DLHE / DLHA wire format

Two new 32-byte packet types mirroring the DLK1/DLPG/DLPN pattern:
DLHE carries (generation_id, mtu_bytes, fps, applier_build_sha) from
drone to GS on the wfb-ng back-tunnel; DLHA echoes generation_id from
GS to drone on the existing decision UDP wire. Both versioned and
CRC32-checked. peek_kind extended.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: dl-inject `--hello` / `--hello-ack` dry-run flags

**Files:**
- Modify: `drone/src/dl_inject.c`

- [ ] **Step 1: Add the failing manual test (will become a contract test later)**

Run: `make -C drone && ./drone/build/dl-inject --hello --dry-run --gen-id 0xCAFEBABE --mtu 3994 --fps 60 --build-sha 0xDEADBEEF`
Expected: exits with usage error or unrecognized option.

- [ ] **Step 2: Extend the option table**

Modify `drone/src/dl_inject.c`. In the `opts` struct array (around line 45), add:

```c
{ "hello",      no_argument,       0, 'H' },
{ "hello-ack",  no_argument,       0, 'A' },
{ "gen-id",     required_argument, 0, 'g' },
{ "mtu",        required_argument, 0, 'u' },
{ "build-sha",  required_argument, 0, 'S' },
```

Note: the existing `--fps` option (short `f`) is reused for the HELLO packet's fps too. Don't duplicate.

Update the `getopt_long` short-options string (around line 89) to add `HAg:u:S:`:

```c
while ((c = getopt_long(argc, argv, "t:M:B:P:k:n:d:b:r:f:Is:Dpq:m:HAg:u:S:h",
                        opts, NULL)) != -1) {
```

- [ ] **Step 3: Add the option handlers and the hello/hello-ack send/print logic**

After the existing locals at the top of `main` (around line 70), add:

```c
bool hello_mode = false;
bool hello_ack_mode = false;
dl_hello_t hello = { .version = DL_WIRE_VERSION };
dl_hello_ack_t hello_ack = { .version = DL_WIRE_VERSION };
```

In the option-handler switch (around line 91-110), add cases:

```c
case 'H': hello_mode = true; break;
case 'A': hello_ack_mode = true; break;
case 'g': {
    /* Accept 0x-prefixed hex or decimal. */
    char *end;
    unsigned long v = strtoul(optarg, &end, 0);
    hello.generation_id = (uint32_t)v;
    hello_ack.generation_id_echo = (uint32_t)v;
    break;
}
case 'u': hello.mtu_bytes = (uint16_t)atoi(optarg); break;
case 'S': {
    char *end;
    unsigned long v = strtoul(optarg, &end, 0);
    hello.applier_build_sha = (uint32_t)v;
    break;
}
```

In the existing `case 'f':` handler, make sure it also writes to `hello.fps`:

```c
case 'f':
    d.fps = (uint8_t)atoi(optarg);
    hello.fps = (uint16_t)atoi(optarg);
    break;
```

- [ ] **Step 4: Add the HELLO / HELLO-ACK send/dry-run path**

After the existing ping-mode block but before the decision send (find the existing `if (ping_mode)` block), add an analogous block. The hello-mode and hello-ack-mode are mutually exclusive with each other and with ping-mode and decision-mode. Use the pattern from ping_mode:

```c
if (hello_mode) {
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    size_t n = dl_wire_encode_hello(&hello, buf, sizeof(buf));
    if (n == 0) { fprintf(stderr, "encode failed\n"); return 5; }
    if (dry_run) {
        for (size_t i = 0; i < n; i++) printf("%02x", buf[i]);
        printf("\n");
        return 0;
    }
    /* TODO: socket send path — not needed for P4a, the real sender
     * is dl_hello.c. dl-inject only needs dry-run for the contract
     * test. */
    fprintf(stderr, "hello: live-send not implemented; use --dry-run\n");
    return 6;
}
if (hello_ack_mode) {
    uint8_t buf[DL_HELLO_ACK_ON_WIRE_SIZE];
    size_t n = dl_wire_encode_hello_ack(&hello_ack, buf, sizeof(buf));
    if (n == 0) { fprintf(stderr, "encode failed\n"); return 5; }
    if (dry_run) {
        for (size_t i = 0; i < n; i++) printf("%02x", buf[i]);
        printf("\n");
        return 0;
    }
    fprintf(stderr, "hello-ack: live-send not implemented; use --dry-run\n");
    return 6;
}
```

- [ ] **Step 5: Update the usage string**

Modify the `usage()` function (around line 18) to mention the new flags:

```c
static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s --target HOST:PORT \\\n"
        "         --mcs N --bandwidth {20|40} --tx-power DBM \\\n"
        "         --k N --n N --depth N \\\n"
        "         --bitrate KBPS [--roi-qp QP] [--fps FPS] \\\n"
        "         [--idr] [--sequence N]\n"
        "       %s --ping --gs-seq N --gs-mono US [--target HOST:PORT]\n"
        "       %s --hello --gen-id N --mtu N --fps N [--build-sha N] --dry-run\n"
        "       %s --hello-ack --gen-id N --dry-run\n"
        "       %s --dry-run ... (prints hex bytes to stdout; no send)\n",
        prog, prog, prog, prog, prog);
}
```

- [ ] **Step 6: Verify the dry-run works**

```bash
make -C drone
./drone/build/dl-inject --hello --dry-run --gen-id 0xCAFEBABE --mtu 3994 --fps 60 --build-sha 0xDEADBEEF
./drone/build/dl-inject --hello-ack --dry-run --gen-id 0xCAFEBABE
```

Expected: Both print 64 hex chars (32 bytes each).

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_inject.c
git commit -m "$(cat <<'EOF'
P4a: dl-inject --hello / --hello-ack for contract testing

Dry-run hex output for the two new packet types so the Python
encoder can be diffed against the C encoder in test_wire_contract.py.
Live-send paths not implemented — the real sender is dl_hello.c.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: GS-side wire encoders / decoders for HELLO / HELLO-ACK

**Files:**
- Modify: `gs/dynamic_link/wire.py`

- [ ] **Step 1: Write failing tests inline (will run with the contract test in Task 5)**

Add to the **end** of `gs/dynamic_link/wire.py` (this is the module under test; we'll exercise it via the contract test). Skip writing pytest tests for the encoders directly — the contract test in Task 5 is the load-bearing test.

- [ ] **Step 2: Add the new constants, dataclasses, and encoders/decoders**

Append to `gs/dynamic_link/wire.py`:

```python
# ---- P4a HELLO / HELLO-ACK ----------------------------------------------
#
# DL_HELLO (drone→GS, 32 bytes):
#    0  4  magic = HELLO_MAGIC ('DLHE')
#    4  1  version
#    5  1  flags
#    6  2  _pad
#    8  4  generation_id
#   12  2  mtu_bytes
#   14  2  fps
#   16  4  applier_build_sha
#   20  8  reserved (zero)
#   28  4  crc32(bytes[0..27])
#
# DL_HELLO_ACK (GS→drone, 32 bytes):
#    0  4  magic = HELLO_ACK_MAGIC ('DLHA')
#    4  1  version
#    5  3  _pad
#    8  4  generation_id_echo
#   12  16 reserved (zero)
#   28  4  crc32(bytes[0..27])

HELLO_MAGIC          = 0x444C4845    # 'DLHE'
HELLO_PAYLOAD_SIZE   = 28
HELLO_ON_WIRE_SIZE   = 32

HELLO_ACK_MAGIC          = 0x444C4841    # 'DLHA'
HELLO_ACK_PAYLOAD_SIZE   = 28
HELLO_ACK_ON_WIRE_SIZE   = 32


@dataclass(frozen=True)
class Hello:
    generation_id: int
    mtu_bytes: int
    fps: int
    applier_build_sha: int = 0
    flags: int = 0


@dataclass(frozen=True)
class HelloAck:
    generation_id_echo: int
    flags: int = 0


def encode_hello(h: Hello) -> bytes:
    payload = bytearray(HELLO_PAYLOAD_SIZE)
    struct.pack_into(">I", payload, 0, HELLO_MAGIC)
    payload[4] = VERSION & 0xFF
    payload[5] = h.flags & 0xFF
    struct.pack_into(">I", payload, 8,  h.generation_id & 0xFFFFFFFF)
    struct.pack_into(">H", payload, 12, h.mtu_bytes & 0xFFFF)
    struct.pack_into(">H", payload, 14, h.fps & 0xFFFF)
    struct.pack_into(">I", payload, 16, h.applier_build_sha & 0xFFFFFFFF)
    crc = _crc32(bytes(payload))
    return bytes(payload) + struct.pack(">I", crc)


def decode_hello(buf: bytes) -> Hello:
    if len(buf) < HELLO_ON_WIRE_SIZE:
        raise ValueError("hello: short buffer")
    (magic,) = struct.unpack_from(">I", buf, 0)
    if magic != HELLO_MAGIC:
        raise ValueError(f"hello: bad magic 0x{magic:08x}")
    if buf[4] != VERSION:
        raise ValueError(f"hello: bad version {buf[4]}")
    crc_wire = struct.unpack_from(">I", buf, HELLO_PAYLOAD_SIZE)[0]
    crc_calc = _crc32(bytes(buf[:HELLO_PAYLOAD_SIZE]))
    if crc_wire != crc_calc:
        raise ValueError("hello: bad crc")
    return Hello(
        flags=buf[5],
        generation_id=struct.unpack_from(">I", buf, 8)[0],
        mtu_bytes=struct.unpack_from(">H", buf, 12)[0],
        fps=struct.unpack_from(">H", buf, 14)[0],
        applier_build_sha=struct.unpack_from(">I", buf, 16)[0],
    )


def encode_hello_ack(h: HelloAck) -> bytes:
    payload = bytearray(HELLO_ACK_PAYLOAD_SIZE)
    struct.pack_into(">I", payload, 0, HELLO_ACK_MAGIC)
    payload[4] = VERSION & 0xFF
    # [5..7] = _pad
    struct.pack_into(">I", payload, 8, h.generation_id_echo & 0xFFFFFFFF)
    # [12..27] = reserved
    crc = _crc32(bytes(payload))
    return bytes(payload) + struct.pack(">I", crc)


def decode_hello_ack(buf: bytes) -> HelloAck:
    if len(buf) < HELLO_ACK_ON_WIRE_SIZE:
        raise ValueError("hello-ack: short buffer")
    (magic,) = struct.unpack_from(">I", buf, 0)
    if magic != HELLO_ACK_MAGIC:
        raise ValueError(f"hello-ack: bad magic 0x{magic:08x}")
    if buf[4] != VERSION:
        raise ValueError(f"hello-ack: bad version {buf[4]}")
    crc_wire = struct.unpack_from(">I", buf, HELLO_ACK_PAYLOAD_SIZE)[0]
    crc_calc = _crc32(bytes(buf[:HELLO_ACK_PAYLOAD_SIZE]))
    if crc_wire != crc_calc:
        raise ValueError("hello-ack: bad crc")
    return HelloAck(
        generation_id_echo=struct.unpack_from(">I", buf, 8)[0],
    )
```

- [ ] **Step 3: Extend `peek_kind`**

Modify the existing `peek_kind` function:

```python
def peek_kind(buf: bytes) -> str:
    """Return 'decision' | 'ping' | 'pong' | 'hello' | 'hello_ack' | 'unknown'."""
    if len(buf) < 4:
        return "unknown"
    (magic,) = struct.unpack_from(">I", buf, 0)
    if magic == MAGIC:            return "decision"
    if magic == PING_MAGIC:       return "ping"
    if magic == PONG_MAGIC:       return "pong"
    if magic == HELLO_MAGIC:      return "hello"
    if magic == HELLO_ACK_MAGIC:  return "hello_ack"
    return "unknown"
```

- [ ] **Step 4: Smoke-test the round-trip**

Run:
```bash
python3 -c "
from gs.dynamic_link.wire import Hello, HelloAck, encode_hello, decode_hello, encode_hello_ack, decode_hello_ack, peek_kind
h = Hello(generation_id=0xCAFEBABE, mtu_bytes=3994, fps=60, applier_build_sha=0xDEADBEEF)
b = encode_hello(h)
assert len(b) == 32
assert peek_kind(b) == 'hello'
assert decode_hello(b) == h
a = HelloAck(generation_id_echo=0xCAFEBABE)
b = encode_hello_ack(a)
assert len(b) == 32
assert peek_kind(b) == 'hello_ack'
assert decode_hello_ack(b) == a
print('OK')
"
```
Expected: prints `OK`.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/wire.py
git commit -m "$(cat <<'EOF'
P4a: GS wire encoders/decoders for DLHE / DLHA

Mirror of dl_wire.c's new types. peek_kind dispatches to two new
strings ('hello', 'hello_ack'). Contract test in the next commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: HELLO / HELLO-ACK contract tests

**Files:**
- Modify: `tests/test_wire_contract.py`

- [ ] **Step 1: Write failing contract tests**

Append to `tests/test_wire_contract.py`:

```python
from dynamic_link.wire import (
    Hello,
    HelloAck,
    encode_hello,
    encode_hello_ack,
)


def test_contract_hello_basic():
    c_bytes = _dl_inject_hex(
        hello=True,
        gen_id="0xcafebabe",
        mtu=3994,
        fps=60,
        build_sha="0xdeadbeef",
    )
    py_bytes = encode_hello(
        Hello(generation_id=0xCAFEBABE,
              mtu_bytes=3994,
              fps=60,
              applier_build_sha=0xDEADBEEF)
    )
    assert c_bytes == py_bytes, (
        f"hello mismatch:\n  C : {c_bytes.hex()}\n  Py: {py_bytes.hex()}"
    )


def test_contract_hello_max_values():
    c_bytes = _dl_inject_hex(
        hello=True,
        gen_id="0xffffffff",
        mtu=65535,
        fps=65535,
        build_sha="0xffffffff",
    )
    py_bytes = encode_hello(
        Hello(generation_id=0xFFFFFFFF,
              mtu_bytes=0xFFFF,
              fps=0xFFFF,
              applier_build_sha=0xFFFFFFFF)
    )
    assert c_bytes == py_bytes


def test_contract_hello_min_values():
    c_bytes = _dl_inject_hex(
        hello=True, gen_id="0", mtu=1, fps=1, build_sha="0",
    )
    py_bytes = encode_hello(
        Hello(generation_id=0, mtu_bytes=1, fps=1, applier_build_sha=0)
    )
    assert c_bytes == py_bytes


def test_contract_hello_ack():
    c_bytes = _dl_inject_hex(
        hello_ack=True, gen_id="0x12345678",
    )
    py_bytes = encode_hello_ack(HelloAck(generation_id_echo=0x12345678))
    assert c_bytes == py_bytes
```

- [ ] **Step 2: Verify the tests run**

Run: `python3 -m pytest tests/test_wire_contract.py -v --ignore=tests/test_mavlink_status.py`
Expected: 4 new tests pass, all existing wire-contract tests still pass.

If `_dl_inject_hex` chokes on `--gen-id 0xcafebabe` because of an
`int(...)` conversion somewhere (it shouldn't — the helper passes
strings straight to argv), confirm `dl-inject`'s argument parser
accepts hex (it should, since we use `strtoul(optarg, &end, 0)`
which auto-detects base from `0x` prefix).

- [ ] **Step 3: Commit**

```bash
git add tests/test_wire_contract.py
git commit -m "$(cat <<'EOF'
P4a: contract tests for DLHE / DLHA

Hex-diff of dl-inject's encoder against gs/dynamic_link/wire.py's
encoder for HELLO and HELLO-ACK packets, covering basic, max, and
min field values.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: drone-side `dl_hello` state machine

**Files:**
- Create: `drone/src/dl_hello.h`
- Create: `drone/src/dl_hello.c`
- Create: `tests/drone/test_dl_hello.c`
- Modify: `drone/Makefile`
- Modify: `drone/src/dl_config.h`
- Modify: `drone/src/dl_config.c`

- [ ] **Step 1: Add `hello.*` keys to drone.conf parser**

Modify `drone/src/dl_config.h` to add fields to `dl_config_t`:

```c
/* P4a hello state machine: announce/keepalive cadence and
 * authoritative-config file paths (overridable for tests). */
uint32_t hello_announce_initial_ms;   /* default 500 */
uint32_t hello_announce_steady_ms;    /* default 5000 */
uint32_t hello_keepalive_ms;          /* default 10000 */
uint32_t hello_announce_initial_count; /* default 60 (=> 30 s of fast retry) */
char     hello_wfb_yaml_path[DL_CONF_MAX_STR];   /* default "/etc/wfb.yaml" */
char     hello_majestic_yaml_path[DL_CONF_MAX_STR]; /* default "/etc/majestic.yaml" */
```

Modify `dl_config_defaults` in `dl_config.c` to set them:

```c
cfg->hello_announce_initial_ms = 500;
cfg->hello_announce_steady_ms = 5000;
cfg->hello_keepalive_ms = 10000;
cfg->hello_announce_initial_count = 60;
strncpy(cfg->hello_wfb_yaml_path, "/etc/wfb.yaml", DL_CONF_MAX_STR - 1);
strncpy(cfg->hello_majestic_yaml_path, "/etc/majestic.yaml", DL_CONF_MAX_STR - 1);
```

Add the key-parsing cases in `dl_config_load` (follow the existing pattern for `osd_msg_path` / `health_timeout_ms`). Keys: `hello_announce_initial_ms`, `hello_announce_steady_ms`, `hello_keepalive_ms`, `hello_announce_initial_count`, `hello_wfb_yaml_path`, `hello_majestic_yaml_path`.

- [ ] **Step 2: Write the state machine header**

`drone/src/dl_hello.h`:
```c
/* dl_hello.h — drone-side config-announce state machine.
 *
 * Responsibilities:
 *   - At boot: read MTU from /etc/wfb.yaml (wireless.mlink) and FPS
 *     from /etc/majestic.yaml (video0.fps). If either fails, refuse
 *     to enter ANNOUNCING — GS will stay in safe_defaults forever.
 *   - Generate a random generation_id (per-boot identifier).
 *   - ANNOUNCING: send DLHE on a 500 ms cadence (configurable) for
 *     the first 60 retries, then 5 s indefinitely. Transition to
 *     KEEPALIVE on first matching DLHA.
 *   - KEEPALIVE: send DLHE every 10 s. Transition back to ANNOUNCING
 *     if no DLHA arrives for 3 keepalive intervals.
 *
 * Pure I/O: caller owns the UDP socket and the timerfd.
 */
#pragma once

#include "dl_config.h"
#include "dl_wire.h"

#include <stdint.h>

typedef enum {
    DL_HELLO_STATE_INIT       = 0,
    DL_HELLO_STATE_ANNOUNCING = 1,
    DL_HELLO_STATE_KEEPALIVE  = 2,
    DL_HELLO_STATE_DISABLED   = 3,  /* MTU/FPS read failed at boot */
} dl_hello_state_t;

typedef struct {
    dl_hello_state_t state;
    uint32_t generation_id;
    uint16_t mtu_bytes;
    uint16_t fps;
    uint32_t announce_count;            /* sent in current ANNOUNCING run */
    uint32_t announces_without_ack;     /* counter for fast-retry exit */
    uint32_t keepalives_without_ack;
    const dl_config_t *cfg;             /* not owned */
} dl_hello_t;

/* Initialize. Reads wfb.yaml + majestic.yaml. Returns 0 on success
 * (state is ANNOUNCING; first send should happen immediately by the
 * caller arming the timerfd at 0 ms). Returns -1 on YAML failure —
 * state is set to DISABLED; the caller should not start the timer. */
int dl_hello_init(dl_hello_t *h, const dl_config_t *cfg);

/* Returns 0 on success. Encodes the next DLHE into `buf` and returns
 * the byte count. `buflen` must be >= DL_HELLO_ON_WIRE_SIZE.
 * Updates internal counters; caller should send via UDP. Returns 0
 * (and writes nothing) if state is DISABLED. */
size_t dl_hello_build_announce(dl_hello_t *h, uint8_t *buf, size_t buflen);

/* Compute next timer expiry in milliseconds based on current state
 * and counters. Returns 0 if DISABLED (timer should not fire). */
uint32_t dl_hello_next_delay_ms(const dl_hello_t *h);

/* Process an incoming DLHA. Returns 1 if the ACK was accepted (matched
 * generation_id) and transitioned state to KEEPALIVE, 0 otherwise. */
int dl_hello_on_ack(dl_hello_t *h, const dl_hello_ack_t *ack);

/* Called when the timer fires while in KEEPALIVE without seeing an
 * ACK. Returns 1 if the no-ack budget is exhausted and the state
 * dropped back to ANNOUNCING. The caller should re-arm the timer. */
int dl_hello_on_keepalive_tick(dl_hello_t *h);
```

- [ ] **Step 3: Write failing tests**

`tests/drone/test_dl_hello.c`:
```c
#include "dl_hello.h"
#include "dl_config.h"
#include "test_main.h"

#include <stdio.h>
#include <string.h>

static void setup_cfg(dl_config_t *cfg) {
    dl_config_defaults(cfg);
    strncpy(cfg->hello_wfb_yaml_path,
            "tests/drone/fixtures/wfb_basic.yaml", DL_CONF_MAX_STR - 1);
    strncpy(cfg->hello_majestic_yaml_path,
            "tests/drone/fixtures/majestic_basic.yaml", DL_CONF_MAX_STR - 1);
    cfg->hello_announce_initial_ms = 500;
    cfg->hello_announce_steady_ms = 5000;
    cfg->hello_keepalive_ms = 10000;
    cfg->hello_announce_initial_count = 3;  /* low for testing */
}

DL_TEST(hello_init_reads_mtu_and_fps_from_fixtures) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_t h;
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
            "tests/drone/fixtures/does_not_exist.yaml",
            DL_CONF_MAX_STR - 1);
    dl_hello_t h;
    int rc = dl_hello_init(&h, &cfg);
    DL_ASSERT_EQ(rc, -1);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_DISABLED);
}

DL_TEST(hello_announcing_first_delay_is_immediate) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_t h;
    dl_hello_init(&h, &cfg);
    /* Fresh announcing → first send should fire ASAP. The caller
     * arms the timerfd with this value. */
    DL_ASSERT_EQ(dl_hello_next_delay_ms(&h), 0);
}

DL_TEST(hello_announcing_uses_initial_ms_for_first_retries) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_t h;
    dl_hello_init(&h, &cfg);
    uint8_t buf[DL_HELLO_ON_WIRE_SIZE];
    dl_hello_build_announce(&h, buf, sizeof(buf));   /* 1st send */
    DL_ASSERT_EQ(dl_hello_next_delay_ms(&h), 500);
    dl_hello_build_announce(&h, buf, sizeof(buf));   /* 2nd */
    dl_hello_build_announce(&h, buf, sizeof(buf));   /* 3rd — exhausts initial */
    /* After initial budget, falls to steady cadence. */
    DL_ASSERT_EQ(dl_hello_next_delay_ms(&h), 5000);
}

DL_TEST(hello_ack_matching_genid_transitions_to_keepalive) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_t h;
    dl_hello_init(&h, &cfg);
    dl_hello_ack_t ack = { .generation_id_echo = h.generation_id };
    int matched = dl_hello_on_ack(&h, &ack);
    DL_ASSERT_EQ(matched, 1);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_KEEPALIVE);
    DL_ASSERT_EQ(dl_hello_next_delay_ms(&h), 10000);
}

DL_TEST(hello_ack_mismatching_genid_ignored) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_t h;
    dl_hello_init(&h, &cfg);
    uint32_t orig_state = h.state;
    dl_hello_ack_t ack = { .generation_id_echo = h.generation_id ^ 0xFFu };
    int matched = dl_hello_on_ack(&h, &ack);
    DL_ASSERT_EQ(matched, 0);
    DL_ASSERT_EQ(h.state, orig_state);
}

DL_TEST(hello_keepalive_without_ack_drops_back_to_announcing) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_t h;
    dl_hello_init(&h, &cfg);
    dl_hello_ack_t ack = { .generation_id_echo = h.generation_id };
    dl_hello_on_ack(&h, &ack);
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_KEEPALIVE);
    DL_ASSERT_EQ(dl_hello_on_keepalive_tick(&h), 0);  /* 1 missed */
    DL_ASSERT_EQ(dl_hello_on_keepalive_tick(&h), 0);  /* 2 missed */
    DL_ASSERT_EQ(dl_hello_on_keepalive_tick(&h), 1);  /* 3 missed → drop */
    DL_ASSERT_EQ(h.state, DL_HELLO_STATE_ANNOUNCING);
}

DL_TEST(hello_build_announce_sets_packet_fields_from_init) {
    dl_config_t cfg; setup_cfg(&cfg);
    dl_hello_t h;
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
```

- [ ] **Step 4: Verify tests fail to compile**

Run: `make -C drone test 2>&1 | head -10`
Expected: `dl_hello.h: No such file or directory`.

- [ ] **Step 5: Implement the state machine**

`drone/src/dl_hello.c`:
```c
/* dl_hello.c — see dl_hello.h. */
#include "dl_hello.h"
#include "dl_log.h"
#include "dl_yaml_get.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

static uint32_t random_u32(void) {
    /* Read 4 bytes from /dev/urandom; fall back to clock-based mix if
     * that fails. The fallback isn't cryptographically random but is
     * fine for our purpose (uniqueness across drone boots). */
    uint32_t v = 0;
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd >= 0) {
        if (read(fd, &v, sizeof(v)) == (ssize_t)sizeof(v)) {
            close(fd);
            if (v != 0) return v;
        }
        close(fd);
    }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    v = (uint32_t)(ts.tv_sec ^ ts.tv_nsec ^ (uint32_t)getpid());
    if (v == 0) v = 1;
    return v;
}

int dl_hello_init(dl_hello_t *h, const dl_config_t *cfg) {
    memset(h, 0, sizeof(*h));
    h->cfg = cfg;
    h->state = DL_HELLO_STATE_DISABLED;

    int mtu = 0;
    int rc = dl_yaml_get_int(cfg->hello_wfb_yaml_path,
                             "wireless", "mlink", &mtu);
    if (rc != 0) {
        dl_log_err("dl_hello: failed to read mtu (%s wireless.mlink): %d",
                   cfg->hello_wfb_yaml_path, rc);
        return -1;
    }
    if (mtu <= 0 || mtu > 0xFFFF) {
        dl_log_err("dl_hello: mtu out of range: %d", mtu);
        return -1;
    }

    int fps = 0;
    rc = dl_yaml_get_int(cfg->hello_majestic_yaml_path,
                        "video0", "fps", &fps);
    if (rc != 0) {
        dl_log_err("dl_hello: failed to read fps (%s video0.fps): %d",
                   cfg->hello_majestic_yaml_path, rc);
        return -1;
    }
    if (fps <= 0 || fps > 0xFFFF) {
        dl_log_err("dl_hello: fps out of range: %d", fps);
        return -1;
    }

    h->mtu_bytes = (uint16_t)mtu;
    h->fps = (uint16_t)fps;
    h->generation_id = random_u32();
    h->state = DL_HELLO_STATE_ANNOUNCING;
    h->announce_count = 0;
    h->announces_without_ack = 0;
    h->keepalives_without_ack = 0;
    dl_log_info("dl_hello: ANNOUNCING gen=0x%08x mtu=%u fps=%u",
                h->generation_id, h->mtu_bytes, h->fps);
    return 0;
}

size_t dl_hello_build_announce(dl_hello_t *h, uint8_t *buf, size_t buflen) {
    if (h->state == DL_HELLO_STATE_DISABLED) return 0;
    dl_hello_t pkt_h = {
        .version = DL_WIRE_VERSION,
        .flags = 0,
        .generation_id = h->generation_id,
        .mtu_bytes = h->mtu_bytes,
        .fps = h->fps,
        .applier_build_sha = 0,  /* populated at build time if desired */
    };
    size_t n = dl_wire_encode_hello(&pkt_h, buf, buflen);
    if (n == 0) return 0;
    h->announce_count++;
    return n;
}

uint32_t dl_hello_next_delay_ms(const dl_hello_t *h) {
    if (h->state == DL_HELLO_STATE_DISABLED) return 0;
    if (h->state == DL_HELLO_STATE_KEEPALIVE) {
        return h->cfg->hello_keepalive_ms;
    }
    /* ANNOUNCING */
    if (h->announce_count == 0) return 0;  /* first send is immediate */
    if (h->announce_count < h->cfg->hello_announce_initial_count) {
        return h->cfg->hello_announce_initial_ms;
    }
    return h->cfg->hello_announce_steady_ms;
}

int dl_hello_on_ack(dl_hello_t *h, const dl_hello_ack_t *ack) {
    if (h->state == DL_HELLO_STATE_DISABLED) return 0;
    if (ack->generation_id_echo != h->generation_id) return 0;
    if (h->state != DL_HELLO_STATE_KEEPALIVE) {
        dl_log_info("dl_hello: KEEPALIVE gen=0x%08x", h->generation_id);
    }
    h->state = DL_HELLO_STATE_KEEPALIVE;
    h->keepalives_without_ack = 0;
    h->announce_count = 0;
    return 1;
}

int dl_hello_on_keepalive_tick(dl_hello_t *h) {
    if (h->state != DL_HELLO_STATE_KEEPALIVE) return 0;
    h->keepalives_without_ack++;
    if (h->keepalives_without_ack >= 3) {
        dl_log_warn("dl_hello: 3 keepalives unacked, returning to ANNOUNCING");
        h->state = DL_HELLO_STATE_ANNOUNCING;
        h->announce_count = 0;
        h->keepalives_without_ack = 0;
        return 1;
    }
    return 0;
}
```

- [ ] **Step 6: Update Makefile**

Add `$(SRCDIR)/dl_hello.c` to `APPLIER_SRCS` and to `TEST_SRCS`. Add `$(TESTDIR)/test_dl_hello.c` to `TEST_SRCS`.

- [ ] **Step 7: Run tests**

Run: `make -C drone test`
Expected: All new `hello_*` tests pass. Existing tests still pass.

- [ ] **Step 8: Commit**

```bash
git add drone/src/dl_hello.h drone/src/dl_hello.c \
        drone/src/dl_config.h drone/src/dl_config.c \
        tests/drone/test_dl_hello.c drone/Makefile
git commit -m "$(cat <<'EOF'
P4a: dl_hello state machine

Drone-side ANNOUNCING/KEEPALIVE state machine. Reads MTU from
/etc/wfb.yaml and FPS from /etc/majestic.yaml at init; refuses to
announce on parse failure so the GS notices via continued AWAITING.
generation_id from /dev/urandom (fallback: clock+pid mix).

Pure logic; caller owns the UDP socket and timerfd. dl_applier.c
wiring follows in the next commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Wire `dl_hello` into `dl_applier`

**Files:**
- Modify: `drone/src/dl_applier.c`

- [ ] **Step 1: Unconditionalize the GS-tunnel send socket**

Today `gs_tunnel_fd` is only opened when `cfg.debug_enable`. P4a needs it always open (for sending DLHE). Around line 233-237:

```c
int gs_tunnel_fd = -1;
struct sockaddr_in gs_tunnel_dst = {0};
gs_tunnel_fd = open_gs_tunnel_socket(&cfg, &gs_tunnel_dst);
```

(Drop the `if (cfg.debug_enable)` guard.) The send-failure path is already non-fatal, so this is safe even on hardware where the tunnel isn't routable.

- [ ] **Step 2: Add `dl_hello` includes and state**

At the top of `dl_applier.c` (with other includes around line 14-25):
```c
#include "dl_hello.h"
```

Inside `main` (after `dl_watchdog_init(&wd, ...)` around line 261), add:
```c
dl_hello_t hello;
int hello_inited = dl_hello_init(&hello, &cfg);
int hello_timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
if (hello_timer_fd < 0) {
    dl_log_err("hello: timerfd_create: %s", strerror(errno));
} else if (hello_inited == 0) {
    /* Arm immediately so the first send happens ASAP. */
    struct itimerspec t = {0};
    t.it_value.tv_nsec = 1 * 1000 * 1000;  /* 1 ms */
    timerfd_settime(hello_timer_fd, 0, &t, NULL);
}
```

Note: when `hello_inited != 0` (YAML parse failed), `hello.state` is `DL_HELLO_STATE_DISABLED` and `dl_hello_next_delay_ms` returns 0 — the timer is created but never armed; the applier runs without sending HELLO. The GS will stay in AWAITING.

- [ ] **Step 3: Add the timer fd to the poll set**

Change the `pfds` array from 3 to 4 entries, and the `poll(pfds, 3, -1)` call to `poll(pfds, 4, -1)`:
```c
struct pollfd pfds[4];
pfds[0].fd = listen_fd; pfds[0].events = POLLIN;
pfds[1].fd = tick_fd;   pfds[1].events = POLLIN;
pfds[2].fd = gap_fd;    pfds[2].events = POLLIN;
pfds[3].fd = hello_timer_fd; pfds[3].events = POLLIN;
```

Update `poll(pfds, 4, -1)` accordingly.

- [ ] **Step 4: Add the HELLO_ACK dispatch to the recv path**

Find the existing `dl_wire_peek_kind(buf, (size_t)got) == DL_PKT_PING` block around line 302 and add a sibling branch for `DL_PKT_HELLO_ACK`. The structure becomes:

```c
} else {
    dl_packet_kind_t kind = dl_wire_peek_kind(buf, (size_t)got);
    if (kind == DL_PKT_PING) {
        /* ... existing ping handler ... */
    } else if (kind == DL_PKT_HELLO_ACK) {
        dl_hello_ack_t ack;
        if (dl_wire_decode_hello_ack(buf, (size_t)got, &ack) == DL_DECODE_OK) {
            dl_hello_on_ack(&hello, &ack);
        } else {
            dl_log_debug("hello_ack: decode failed (%zd bytes)", got);
        }
    } else {
        /* ... existing decision handler ... */
    }
}
```

(Refactor the existing if/else chain to a switch or sequential if's. The exact form depends on the current code structure.)

- [ ] **Step 5: Add the timer-expiry handler**

After the existing tick / gap handlers, add:

```c
if (pfds[3].revents & POLLIN) {
    uint64_t expirations;
    (void)read(hello_timer_fd, &expirations, sizeof(expirations));
    if (hello.state == DL_HELLO_STATE_KEEPALIVE) {
        dl_hello_on_keepalive_tick(&hello);
        /* If on_keepalive_tick returned 1 it dropped us back to
         * ANNOUNCING; either way we still want to send a HELLO
         * now. */
    }
    if (hello.state != DL_HELLO_STATE_DISABLED && gs_tunnel_fd >= 0) {
        uint8_t out[DL_HELLO_ON_WIRE_SIZE];
        size_t n = dl_hello_build_announce(&hello, out, sizeof(out));
        if (n > 0) {
            ssize_t s = sendto(gs_tunnel_fd, out, n, 0,
                               (struct sockaddr *)&gs_tunnel_dst,
                               sizeof(gs_tunnel_dst));
            if (s < 0) {
                dl_log_debug("hello: sendto: %s", strerror(errno));
            }
        }
    }
    /* Re-arm. */
    uint32_t delay_ms = dl_hello_next_delay_ms(&hello);
    struct itimerspec t = {0};
    t.it_value.tv_sec = delay_ms / 1000;
    t.it_value.tv_nsec = (delay_ms % 1000) * 1000000;
    /* delay_ms == 0 means "immediate"; timerfd_settime with all-zero
     * disarms instead, so coerce to 1 ms. */
    if (delay_ms == 0 && hello.state != DL_HELLO_STATE_DISABLED) {
        t.it_value.tv_nsec = 1 * 1000 * 1000;
    }
    timerfd_settime(hello_timer_fd, 0, &t, NULL);
}
```

- [ ] **Step 6: Close the timer fd on shutdown**

Near the existing `close(gs_tunnel_fd)` (around line 496):
```c
if (hello_timer_fd >= 0) close(hello_timer_fd);
```

- [ ] **Step 7: Build and smoke-test**

```bash
make -C drone
make -C drone test   # all existing tests should still pass
```

There's no live-fire test yet — that comes in Task 11 (E2E). Static linking + existing tests is sufficient for now.

- [ ] **Step 8: Commit**

```bash
git add drone/src/dl_applier.c
git commit -m "$(cat <<'EOF'
P4a: integrate dl_hello into dl_applier poll loop

Adds a 4th fd (hello_timer_fd) and a new dispatch branch for
DL_PKT_HELLO_ACK on the listen socket. Unconditionalizes the
gs_tunnel send socket (previously gated on debug_enable) so HELLO
packets can be sent in production. Timer-expiry handler builds the
next DLHE, sends via UDP, re-arms with the state-machine-derived
delay.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: GS-side `DroneConfigState` module

**Files:**
- Create: `gs/dynamic_link/drone_config.py`
- Create: `tests/test_drone_config.py`

- [ ] **Step 1: Write failing tests**

`tests/test_drone_config.py`:
```python
"""Unit tests for the GS-side drone-config state machine."""
from __future__ import annotations

import pytest

from dynamic_link.drone_config import (
    DroneConfigEvent,
    DroneConfigState,
    State,
)
from dynamic_link.wire import Hello


def _hello(gen: int = 0xCAFEBABE, mtu: int = 3994, fps: int = 60) -> Hello:
    return Hello(generation_id=gen, mtu_bytes=mtu, fps=fps,
                 applier_build_sha=0xDEADBEEF)


def test_initial_state_is_awaiting():
    s = DroneConfigState()
    assert s.state is State.AWAITING
    assert s.mtu_bytes is None
    assert s.fps is None
    assert s.generation_id is None
    assert not s.is_synced()


def test_first_hello_transitions_to_synced():
    s = DroneConfigState()
    events = []
    ev = s.on_hello(_hello())
    assert s.state is State.SYNCED
    assert s.is_synced()
    assert s.mtu_bytes == 3994
    assert s.fps == 60
    assert s.generation_id == 0xCAFEBABE
    assert ev is DroneConfigEvent.SYNCED


def test_subsequent_hellos_same_genid_are_idempotent():
    s = DroneConfigState()
    s.on_hello(_hello())
    ev = s.on_hello(_hello())
    assert s.state is State.SYNCED
    assert ev is DroneConfigEvent.ALREADY_SYNCED


def test_hello_with_new_genid_triggers_reboot_and_resync():
    s = DroneConfigState()
    s.on_hello(_hello(gen=0x1111))
    ev = s.on_hello(_hello(gen=0x2222, mtu=1500, fps=90))
    assert s.state is State.SYNCED   # immediately re-syncs
    assert s.generation_id == 0x2222
    assert s.mtu_bytes == 1500
    assert s.fps == 90
    assert ev is DroneConfigEvent.REBOOT_DETECTED


def test_build_ack_returns_helloack_for_current_genid():
    s = DroneConfigState()
    s.on_hello(_hello(gen=0xABCD))
    ack = s.build_ack()
    assert ack is not None
    assert ack.generation_id_echo == 0xABCD


def test_build_ack_returns_none_when_awaiting():
    s = DroneConfigState()
    assert s.build_ack() is None
```

- [ ] **Step 2: Run tests and verify failure**

Run: `python3 -m pytest tests/test_drone_config.py -v`
Expected: `ModuleNotFoundError: No module named 'dynamic_link.drone_config'`.

- [ ] **Step 3: Implement the module**

`gs/dynamic_link/drone_config.py`:
```python
"""GS-side drone-config state machine.

Tracks the drone's authoritative `(mtu_bytes, fps, generation_id)`,
gates the policy's emit path on whether a HELLO has been received,
and detects drone reboot via `generation_id` change.

See `docs/superpowers/specs/2026-05-11-drone-config-handshake-and-dynamic-fec-design.md`.
"""
from __future__ import annotations

import enum
import logging
from dataclasses import dataclass

from .wire import Hello, HelloAck

log = logging.getLogger(__name__)


class State(enum.Enum):
    AWAITING = "awaiting"
    SYNCED = "synced"


class DroneConfigEvent(enum.Enum):
    """Returned from `on_hello` to let callers decide what to log."""
    SYNCED = "synced"               # AWAITING → SYNCED
    REBOOT_DETECTED = "reboot"      # SYNCED → SYNCED with new gen_id
    ALREADY_SYNCED = "noop"         # same gen_id as current


@dataclass
class DroneConfigState:
    state: State = State.AWAITING
    generation_id: int | None = None
    mtu_bytes: int | None = None
    fps: int | None = None
    applier_build_sha: int | None = None

    def is_synced(self) -> bool:
        return self.state is State.SYNCED

    def on_hello(self, h: Hello) -> DroneConfigEvent:
        """Apply a received HELLO. Returns an event describing the
        transition (for caller-side logging)."""
        if self.state is State.AWAITING:
            self._adopt(h)
            log.info(
                "drone_config sync gen=0x%08x mtu=%d fps=%d sha=0x%08x",
                h.generation_id, h.mtu_bytes, h.fps, h.applier_build_sha,
            )
            return DroneConfigEvent.SYNCED
        # SYNCED
        if h.generation_id == self.generation_id:
            return DroneConfigEvent.ALREADY_SYNCED
        log.warning(
            "drone_reboot_detected old_gen=0x%08x new_gen=0x%08x "
            "new_mtu=%d new_fps=%d",
            self.generation_id, h.generation_id, h.mtu_bytes, h.fps,
        )
        self._adopt(h)
        return DroneConfigEvent.REBOOT_DETECTED

    def build_ack(self) -> HelloAck | None:
        """Return a HelloAck for the current generation_id, or None if
        we haven't seen any HELLO yet."""
        if self.generation_id is None:
            return None
        return HelloAck(generation_id_echo=self.generation_id)

    def _adopt(self, h: Hello) -> None:
        self.state = State.SYNCED
        self.generation_id = h.generation_id
        self.mtu_bytes = h.mtu_bytes
        self.fps = h.fps
        self.applier_build_sha = h.applier_build_sha
```

- [ ] **Step 4: Run tests and verify pass**

Run: `python3 -m pytest tests/test_drone_config.py -v`
Expected: All 6 tests pass.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/drone_config.py tests/test_drone_config.py
git commit -m "$(cat <<'EOF'
P4a: GS-side DroneConfigState

Tracks (generation_id, mtu_bytes, fps) from incoming DLHE.
Detects reboot when a HELLO arrives with a fresh generation_id.
Pure data — no I/O. Wiring into the service follows.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Wire `DroneConfigState` into the GS service

**Files:**
- Modify: `gs/dynamic_link/tunnel_listener.py`
- Modify: `gs/dynamic_link/return_link.py`
- Modify: `gs/dynamic_link/service.py`

- [ ] **Step 1: Add HELLO dispatch to TunnelListener**

In `gs/dynamic_link/tunnel_listener.py`, extend the protocol to accept a `HelloHandler`:

```python
from typing import Callable

from . import wire

HelloHandler = Callable[[wire.Hello], None]

class _Protocol(asyncio.DatagramProtocol):
    def __init__(
        self,
        on_pong: PongHandler,
        on_hello: HelloHandler | None = None,
    ) -> None:
        self._on_pong = on_pong
        self._on_hello = on_hello
        self._unknown_count = 0

    def datagram_received(self, data: bytes, addr) -> None:
        t4 = time.monotonic_ns() // 1000
        kind = wire.peek_kind(data)
        if kind == "pong":
            try:
                pong = wire.decode_pong(data)
            except ValueError as e:
                log.warning("tunnel_listener: bad pong from %s: %s", addr, e)
                return
            self._on_pong(pong, t4)
            return
        if kind == "hello":
            if self._on_hello is None:
                return
            try:
                hello = wire.decode_hello(data)
            except ValueError as e:
                log.warning("tunnel_listener: bad hello from %s: %s", addr, e)
                return
            self._on_hello(hello)
            return
        if kind == "unknown":
            self._unknown_count += 1
            if self._unknown_count <= 3 or self._unknown_count % 100 == 0:
                log.debug("tunnel_listener: unknown-magic packet from %s "
                          "(%d total)", addr, self._unknown_count)
            return
        log.debug("tunnel_listener: unexpected %s packet from %s", kind, addr)
```

Update `TunnelListener.__init__` to accept and pass `on_hello`:

```python
class TunnelListener:
    def __init__(
        self,
        host: str,
        port: int,
        on_pong: PongHandler,
        on_hello: HelloHandler | None = None,
    ) -> None:
        self.host = host
        self.port = port
        self._on_pong = on_pong
        self._on_hello = on_hello
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _Protocol(self._on_pong, self._on_hello),
            local_addr=(self.host, self.port),
            allow_broadcast=False,
        )
        log.info("tunnel_listener: bound %s:%d", self.host, self.port)
```

- [ ] **Step 2: Add `send_hello_ack` to ReturnLink**

In `gs/dynamic_link/return_link.py`, find the existing `send_ping` method and add a sibling:

```python
def send_hello_ack(self, packet: bytes) -> None:
    """Send a pre-encoded DLHA to the drone."""
    try:
        self._sock.sendto(packet, self._addr)
    except OSError as e:
        log.debug("return_link: hello_ack sendto: %s", e)
```

(Look at how `send_ping` is structured and match the style — same socket, same destination.)

- [ ] **Step 3: Add a passthrough test to verify the new hook**

`tests/test_drone_config.py` already covers the state machine; we need a separate test that confirms the listener invokes the hook correctly. Add to `tests/test_drone_config.py`:

```python
import asyncio

import pytest


@pytest.mark.asyncio
async def test_tunnel_listener_dispatches_hello_to_handler():
    """Bind a TunnelListener on an ephemeral port, send a wire-encoded
    HELLO from a UDP socket, verify the handler fires."""
    from socket import AF_INET, SOCK_DGRAM, socket as Socket

    from dynamic_link.tunnel_listener import TunnelListener
    from dynamic_link.wire import Hello, encode_hello

    received: list[Hello] = []
    listener = TunnelListener(
        "127.0.0.1", 0,  # 0 = ephemeral
        on_pong=lambda *_: None,
        on_hello=lambda h: received.append(h),
    )
    await listener.start()
    transport = listener._transport
    sockname = transport.get_extra_info("sockname")  # type: ignore[union-attr]
    port = sockname[1]

    sender = Socket(AF_INET, SOCK_DGRAM)
    sender.sendto(
        encode_hello(Hello(generation_id=0x42, mtu_bytes=1400,
                           fps=30, applier_build_sha=0)),
        ("127.0.0.1", port),
    )
    # Let the event loop process the datagram.
    await asyncio.sleep(0.05)
    sender.close()
    listener.stop()

    assert len(received) == 1
    assert received[0].generation_id == 0x42
```

Note: this test requires `pytest-asyncio`. Confirm it's in `tests/conftest.py` or the project's deps; if not, this test goes in `test_drone_e2e.py` instead under the existing async-test infrastructure there.

- [ ] **Step 4: Wire `DroneConfigState` into `service.py`**

Add imports near the top of `service.py`:
```python
from .drone_config import DroneConfigState
```

In `run_service` (around line 330 where `Policy` is constructed), instantiate the state before `Policy`:

```python
drone_config = DroneConfigState()
policy = Policy(policy_cfg, profile, drone_config=drone_config)
```

(Policy construction is modified in Task 10 to accept this.)

Wire the listener:

Find where `TunnelListener` is currently constructed (search for `TunnelListener(`) and add the `on_hello` callback. The callback should: forward to `drone_config.on_hello(...)`, then build an ACK and send via `return_link.send_hello_ack(...)`:

```python
def on_hello(h):
    event = drone_config.on_hello(h)
    if event in (DroneConfigEvent.SYNCED, DroneConfigEvent.REBOOT_DETECTED):
        # Log already happens inside on_hello.
        pass
    if return_link is not None:
        ack = drone_config.build_ack()
        if ack is not None:
            return_link.send_hello_ack(encode_hello_ack(ack))

# ... wherever TunnelListener is constructed:
tunnel_listener = TunnelListener(
    host=tunnel_listen_host,
    port=tunnel_listen_port,
    on_pong=on_pong,    # existing
    on_hello=on_hello,  # new
)
```

Add the import:
```python
from .drone_config import DroneConfigState, DroneConfigEvent
from .wire import encode_hello_ack
```

- [ ] **Step 5: Run tests**

```bash
python3 -m pytest tests/test_drone_config.py -v --ignore=tests/test_mavlink_status.py
python3 -m pytest --ignore=tests/test_mavlink_status.py
```
Expected: All tests pass. The new dispatch test runs (or is skipped if pytest-asyncio missing).

- [ ] **Step 6: Commit**

```bash
git add gs/dynamic_link/tunnel_listener.py gs/dynamic_link/return_link.py \
        gs/dynamic_link/service.py tests/test_drone_config.py
git commit -m "$(cat <<'EOF'
P4a: wire DroneConfigState into the GS service

TunnelListener gains an on_hello hook; ReturnLink gains
send_hello_ack. service.py constructs DroneConfigState, registers
the hook, and ACKs every received HELLO (idempotent for repeats,
covers ANNOUNCING retries).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: Gate the policy on `is_synced()`

**Files:**
- Modify: `gs/dynamic_link/policy.py`

- [ ] **Step 1: Add a failing test**

In `tests/test_policy_leading.py` (or create `tests/test_policy_gating.py` if cleaner), add:

```python
def test_policy_emits_safe_defaults_until_drone_synced():
    """Until the drone has sent a HELLO, the policy must emit a
    safe-defaults decision regardless of the incoming signals."""
    from dynamic_link.drone_config import DroneConfigState
    from dynamic_link.policy import Policy
    from dynamic_link.wire import Hello

    # Build a minimal Policy fixture. Reuse whatever helper exists
    # in the file for constructing PolicyConfig + RadioProfile. If
    # nothing exists, paste a tiny inline factory.
    policy_cfg, profile = _make_minimal_policy_fixture()
    drone_cfg = DroneConfigState()
    policy = Policy(policy_cfg, profile, drone_config=drone_cfg)

    # Drive a normal RX-signals tick.
    signals = _make_healthy_signals()
    decision_before = policy.tick(signals)
    assert decision_before.k == policy_cfg.safe.k
    assert decision_before.n == policy_cfg.safe.n
    assert decision_before.depth == policy_cfg.safe.depth
    assert decision_before.mcs == policy_cfg.safe.mcs

    # Synthesize a HELLO; policy should now emit "real" decisions.
    drone_cfg.on_hello(Hello(generation_id=1, mtu_bytes=1400, fps=60))
    decision_after = policy.tick(signals)
    # Real selection chose something — even if the signals are calm
    # the MCS selection runs.
    assert decision_after.mcs >= 1  # any MCS, just not the safe sentinel
```

Look in `tests/test_policy_leading.py` for existing helpers like `_make_policy()` and `_make_signals()` and reuse if available; if not, copy the minimal fixture pattern from `tests/test_policy_trailing.py`.

- [ ] **Step 2: Run test to confirm failure**

Run: `python3 -m pytest tests/test_policy_leading.py::test_policy_emits_safe_defaults_until_drone_synced -v`
Expected: failure — `Policy.__init__()` doesn't accept a `drone_config` arg, OR the test passes for the wrong reason (because we haven't wired the gate yet).

- [ ] **Step 3: Extend Policy to take an optional DroneConfigState**

In `gs/dynamic_link/policy.py`, find the `Policy.__init__` method (search for `class Policy` and the `def __init__`):

```python
from .drone_config import DroneConfigState

class Policy:
    def __init__(
        self,
        cfg: PolicyConfig,
        profile: RadioProfile,
        *,
        drone_config: DroneConfigState | None = None,
    ) -> None:
        self.cfg = cfg
        self.profile = profile
        self.drone_config = drone_config  # None == always-synced (back-compat)
        # ... existing init ...
```

- [ ] **Step 4: Add the gate in `tick`**

Near the start of `Policy.tick`, after the signal-aggregation but before the leading selection:

```python
def tick(self, signals: Signals, ts_ms: float | None = None) -> Decision:
    if ts_ms is None:
        ts_ms = signals.session_ts_ms if signals else 0.0

    # P4a: if drone hasn't reported its config yet, emit safe_defaults.
    if self.drone_config is not None and not self.drone_config.is_synced():
        return self._safe_decision(reason="awaiting_drone_config")

    # ... existing body ...
```

If `_safe_decision` doesn't exist, add it:

```python
def _safe_decision(self, *, reason: str) -> Decision:
    safe = self.cfg.safe
    return Decision(
        timestamp=0.0,
        mcs=safe.mcs,
        bandwidth=self.cfg.leading.bandwidth,
        tx_power_dBm=self.cfg.leading.tx_power_min_dBm,
        k=safe.k,
        n=safe.n,
        depth=safe.depth,
        bitrate_kbps=int(self.cfg.bitrate.min_bitrate_kbps),
        idr_request=False,
        reason=reason,
    )
```

(Check the actual `Decision` shape in `gs/dynamic_link/decision.py` for the precise field set — match it.)

- [ ] **Step 5: Run tests and verify pass**

```bash
python3 -m pytest tests/test_policy_leading.py -v
python3 -m pytest --ignore=tests/test_mavlink_status.py
```
Expected: New test passes. All existing tests pass (Policy construction without `drone_config=` still works thanks to the default).

- [ ] **Step 6: Commit**

```bash
git add gs/dynamic_link/policy.py tests/test_policy_leading.py
git commit -m "$(cat <<'EOF'
P4a: gate Policy.tick on drone_config.is_synced()

Until the drone has reported its config (mtu, fps, generation_id)
via DLHE, Policy.tick emits a safe-defaults decision regardless of
incoming signals. This keeps the wire heartbeat alive (preventing
the drone's watchdog from firing) without applying speculative
parameters.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 11: End-to-end test — handshake, decide, reboot, bad YAML

**Files:**
- Modify: `tests/test_drone_e2e.py`

- [ ] **Step 1: Inspect the existing `_sandbox` helper**

Read `tests/test_drone_e2e.py` to understand:
- How `dl-applier` is spawned (probably via subprocess with a temp `drone.conf`).
- How env-vars or config-overrides are passed.
- The mocking of `wfb_tx`, encoder HTTP, MAVLink sink.

The new tests will need to:
1. Place test YAML fixtures at temp paths.
2. Override `hello_wfb_yaml_path` and `hello_majestic_yaml_path` in the generated `drone.conf`.
3. Have the GS side bind on a known port to receive the DLHE.

- [ ] **Step 2: Write the failing tests**

Append to `tests/test_drone_e2e.py`:
```python
def test_drone_e2e_handshake_then_decide(tmp_path):
    """Drone sends HELLO → GS receives → GS sends ACK → drone enters
    KEEPALIVE → subsequent decisions are emitted normally."""
    # Stage fixture YAMLs under tmp_path.
    wfb_yaml = tmp_path / "wfb.yaml"
    wfb_yaml.write_text("wireless:\n  mlink: 3994\n")
    majestic_yaml = tmp_path / "majestic.yaml"
    majestic_yaml.write_text("video0:\n  fps: 60\n")

    with _sandbox(
        extra_drone_conf={
            "hello_wfb_yaml_path": str(wfb_yaml),
            "hello_majestic_yaml_path": str(majestic_yaml),
            "hello_announce_initial_ms": 100,  # fast for test
        }
    ) as ctx:
        # ctx exposes ctx.gs_tunnel_recv_sock (the GS's bound listener)
        # or similar. Adapt to whatever the existing sandbox provides.
        hello_bytes = ctx.recv_one_hello(timeout=2.0)
        assert hello_bytes is not None
        from dynamic_link.wire import decode_hello, HelloAck, encode_hello_ack
        h = decode_hello(hello_bytes)
        assert h.mtu_bytes == 3994
        assert h.fps == 60

        # Send an ACK back; drone should transition to KEEPALIVE.
        ack_bytes = encode_hello_ack(HelloAck(generation_id_echo=h.generation_id))
        ctx.send_to_drone(ack_bytes)
        # KEEPALIVE has a 10s cadence in production — set to 1s here.
        # If the sandbox can introspect drone log lines, assert a
        # "KEEPALIVE" log appears.


def test_drone_e2e_restart_triggers_new_generation_id(tmp_path):
    """Restarting dl-applier mid-run produces a fresh generation_id,
    visible to the GS."""
    wfb_yaml = tmp_path / "wfb.yaml"
    wfb_yaml.write_text("wireless:\n  mlink: 1400\n")
    majestic_yaml = tmp_path / "majestic.yaml"
    majestic_yaml.write_text("video0:\n  fps: 30\n")

    with _sandbox(
        extra_drone_conf={
            "hello_wfb_yaml_path": str(wfb_yaml),
            "hello_majestic_yaml_path": str(majestic_yaml),
            "hello_announce_initial_ms": 100,
        }
    ) as ctx:
        hello1 = decode_hello(ctx.recv_one_hello(timeout=2.0))
        ctx.restart_drone_applier()
        hello2 = decode_hello(ctx.recv_one_hello(timeout=2.0))
        assert hello1.generation_id != hello2.generation_id


def test_drone_e2e_bad_yaml_means_no_hello(tmp_path):
    """If /etc/wfb.yaml has no mlink key, dl-applier must not send a
    HELLO; GS therefore never transitions out of AWAITING."""
    wfb_yaml = tmp_path / "wfb.yaml"
    wfb_yaml.write_text("wireless:\n  txpower: 50\n")  # no mlink
    majestic_yaml = tmp_path / "majestic.yaml"
    majestic_yaml.write_text("video0:\n  fps: 30\n")

    with _sandbox(
        extra_drone_conf={
            "hello_wfb_yaml_path": str(wfb_yaml),
            "hello_majestic_yaml_path": str(majestic_yaml),
            "hello_announce_initial_ms": 100,
        }
    ) as ctx:
        hello_bytes = ctx.recv_one_hello(timeout=2.0)
        assert hello_bytes is None  # nothing received within timeout
```

- [ ] **Step 3: Update the sandbox helper if needed**

If `_sandbox` doesn't already support the new helpers (`recv_one_hello`, `send_to_drone`, `restart_drone_applier`, `extra_drone_conf`), extend it:

- `extra_drone_conf` — merge into the generated `drone.conf` before spawning the applier.
- `recv_one_hello(timeout)` — block on the GS-side tunnel UDP socket; filter by `peek_kind == "hello"`; return bytes or None on timeout.
- `send_to_drone(bytes)` — write to the drone's listen port.
- `restart_drone_applier()` — kill and respawn the dl-applier subprocess; preserve state files.

Match the style of existing sandbox helpers (probably `subprocess.Popen` + signal/wait pattern).

- [ ] **Step 4: Run the new tests**

```bash
python3 -m pytest tests/test_drone_e2e.py -v --ignore=tests/test_mavlink_status.py
```
Expected: 3 new tests pass.

If `recv_one_hello` is flaky (UDP timing), increase the per-test timeout and the `hello_announce_initial_ms` override accordingly.

- [ ] **Step 5: Commit**

```bash
git add tests/test_drone_e2e.py
git commit -m "$(cat <<'EOF'
P4a: e2e tests for handshake, restart, bad-yaml paths

Three new e2e scenarios:
- happy path: drone sends HELLO, GS ACKs, drone enters KEEPALIVE
- restart: kill+respawn dl-applier, expect fresh generation_id
- bad yaml: missing mlink → no HELLO sent → GS stays awaiting

Uses temp-file YAML fixtures via the existing _sandbox helper
(extended to take an extra_drone_conf dict).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 12: Update operator docs, sample config, README

**Files:**
- Modify: `conf/drone.conf.sample`
- Modify: `CLAUDE.md` (operator-prerequisites section)
- Modify: `README.md` (if it has a "configuration" section that lists keys)

- [ ] **Step 1: Document new drone.conf keys**

Append to `conf/drone.conf.sample`:
```ini
# --- P4a: drone→GS config handshake ----------------------------------
# Authoritative sources for MTU and FPS, read once at startup.
# MTU comes from /etc/wfb.yaml `wireless.mlink` (typically 1400 or
# 3994 depending on wfb-ng's radio_mtu setting). FPS comes from
# /etc/majestic.yaml `video0.fps`.
hello_wfb_yaml_path = /etc/wfb.yaml
hello_majestic_yaml_path = /etc/majestic.yaml

# Cadence of the DLHE announce. The first
# `hello_announce_initial_count` retries use the "initial" delay;
# after that the "steady" delay is used until an ACK arrives.
hello_announce_initial_ms    = 500
hello_announce_steady_ms     = 5000
hello_announce_initial_count = 60
# Once ACKed, DLHE is sent every `hello_keepalive_ms` to keep the
# GS aware. Three missed keepalive ACKs drop back to ANNOUNCING.
hello_keepalive_ms = 10000
```

- [ ] **Step 2: Add operator-prerequisites note to CLAUDE.md**

In the existing "Operational prerequisites" section of `CLAUDE.md`, append:

```markdown
3. `/etc/wfb.yaml` must contain `wireless.mlink: <integer>` — the
   radio MTU. dl-applier reads this at boot and reports it to the
   GS via the DLHE handshake; missing key means the applier refuses
   to send HELLO and the GS stays in safe_defaults.
4. `/etc/majestic.yaml` must contain `video0.fps: <integer>` —
   same constraint as above, on the FPS side.
```

- [ ] **Step 3: Verify no other docs reference the removed `fec.mtu_bytes`**

Run: `grep -rn "fec.mtu_bytes\|mtu_bytes" docs/ README.md conf/ 2>/dev/null`
- Anything pointing at the old GS-side mtu_bytes key should now mention "the drone-reported MTU via DLHE." Update inline.

Note: `gs.yaml.sample`'s `fec.mtu_bytes` key is left in for P4a (since the GS predictor still uses it). P4b removes it. Don't touch in this phase.

- [ ] **Step 4: Commit**

```bash
git add conf/drone.conf.sample CLAUDE.md README.md
git commit -m "$(cat <<'EOF'
P4a: docs for the new drone config handshake

Sample drone.conf entries, CLAUDE.md operator-prerequisites
addition for /etc/wfb.yaml and /etc/majestic.yaml.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 13: Full-suite verification

- [ ] **Step 1: Run both test suites end-to-end**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py
make -C drone test
```

Both must pass clean. Expected new test counts:
- Python: +6 in `test_drone_config.py`, +1 in `test_policy_leading.py`, +4 in `test_wire_contract.py`, +3 in `test_drone_e2e.py` ≈ +14.
- C: +8 in `test_dl_yaml.c`, +6 in `test_wire.c`, +8 in `test_dl_hello.c` ≈ +22.

- [ ] **Step 2: Smoke the applier startup on a workstation**

```bash
mkdir -p /tmp/dl-p4a-smoke
cat > /tmp/dl-p4a-smoke/wfb.yaml <<EOF
wireless:
  mlink: 1400
EOF
cat > /tmp/dl-p4a-smoke/majestic.yaml <<EOF
video0:
  fps: 60
EOF
cat > /tmp/dl-p4a-smoke/drone.conf <<EOF
listen_addr = 127.0.0.1
listen_port = 5800
gs_tunnel_addr = 127.0.0.1
gs_tunnel_port = 5801
hello_wfb_yaml_path = /tmp/dl-p4a-smoke/wfb.yaml
hello_majestic_yaml_path = /tmp/dl-p4a-smoke/majestic.yaml
hello_announce_initial_ms = 500
EOF
./drone/build/dl-applier --config /tmp/dl-p4a-smoke/drone.conf &
APPLIER_PID=$!
sleep 1
# Listen on 5801 with a UDP socket; should see a DLHE arrive.
python3 -c "
import socket, struct
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('127.0.0.1', 5801))
s.settimeout(2.0)
data, addr = s.recvfrom(64)
assert data[:4] == b'DLHE', data[:4]
gen = struct.unpack('>I', data[8:12])[0]
mtu = struct.unpack('>H', data[12:14])[0]
fps = struct.unpack('>H', data[14:16])[0]
print(f'OK: gen=0x{gen:08x} mtu={mtu} fps={fps}')
"
kill $APPLIER_PID
```

Expected: prints `OK: gen=0x... mtu=1400 fps=60`. The applier exits cleanly on SIGTERM.

- [ ] **Step 3: Final commit if any drift**

If anything in the smoke surfaced an issue, fix and commit. Otherwise no commit needed.

---

## Spec Coverage Check

| Spec section / requirement | Implementing task |
|---|---|
| `DLHE` wire format | Task 2 (C), Task 4 (Python) |
| `DLHA` wire format | Task 2 (C), Task 4 (Python) |
| Wire contract test | Task 5 |
| MTU from `/etc/wfb.yaml wireless.mlink` | Task 1, Task 6 |
| FPS from `/etc/majestic.yaml video0.fps` | Task 1, Task 6 |
| Drone state machine (ANNOUNCING/KEEPALIVE) | Task 6 |
| `generation_id` per boot | Task 6 (`random_u32` in `dl_hello_init`) |
| Drone-side poll-loop integration | Task 7 |
| GS state machine (AWAITING/SYNCED) | Task 8 |
| GS-side TunnelListener integration | Task 9 |
| GS-side ACK on every received HELLO | Task 9 |
| Reboot detection via gen_id change | Task 8 (state machine), Task 11 (e2e test) |
| Safe-defaults until ACKed | Task 10 |
| Parse-failure → no HELLO | Task 6 (`hello_init` returns -1), Task 11 (e2e test) |
| Operator docs (drone.conf, CLAUDE.md) | Task 12 |
| Full-suite verification | Task 13 |

No spec requirements left without an implementing task.
