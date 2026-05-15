# Waybeam encoder support — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let `encoder_kind = waybeam` source `video0.fps` from `/etc/waybeam.json` at applier boot instead of `/etc/majestic.yaml`, while leaving the rest of the encoder pipeline untouched.

**Architecture:** A small inline JSON helper (`dl_json_get.{h,c}`) mirrors `dl_yaml_get` in shape and scope ("robust enough for one specific JSON shape, not a general parser"). `dl_hello_init` dispatches on `cfg->encoder_kind` to pick the YAML or JSON path. One new config field (`hello_waybeam_json_path`) carries the file location. No GS-side or wire-format changes.

**Tech Stack:** C11, drone-side only. POSIX-only. Tests via the existing `make -C drone test` harness (DL_TEST macros).

**Spec:** `docs/superpowers/specs/2026-05-14-waybeam-encoder-support-design.md`

---

## File Structure

**New files**
- `drone/src/dl_json_get.h` — header for the JSON helper (mirrors `dl_yaml_get.h`).
- `drone/src/dl_json_get.c` — implementation (~120 LOC including helpers).
- `tests/drone/test_dl_json.c` — unit tests for the helper.
- `tests/drone/fixtures/waybeam_basic.json` — full waybeam config fixture.
- `tests/drone/fixtures/waybeam_compact.json` — compact (no-whitespace) variant.
- `tests/drone/fixtures/waybeam_no_fps.json` — `video0` block without `fps` key.

**Modified files**
- `drone/Makefile` — wire `dl_json_get.c` into APPLIER_SRCS, TEST_SRCS, and add `test_dl_json.c`.
- `drone/src/dl_config.h` — add `hello_waybeam_json_path` field.
- `drone/src/dl_config.c` — default + parser line.
- `drone/src/dl_hello.c` — dispatch on `encoder_kind`.
- `tests/drone/test_config.c` — assert new default + parsing.
- `tests/drone/test_dl_hello.c` — assert waybeam path + unknown-kind rejection.
- `conf/drone.conf.sample` — document `hello_waybeam_json_path`.
- `README.md` — operator prerequisite note.
- `CLAUDE.md` — update Operational Prerequisites #4.

---

## Task 1: JSON helper module (`dl_json_get`)

**Files:**
- Create: `drone/src/dl_json_get.h`
- Create: `drone/src/dl_json_get.c`
- Create: `tests/drone/test_dl_json.c`
- Create: `tests/drone/fixtures/waybeam_basic.json`
- Create: `tests/drone/fixtures/waybeam_compact.json`
- Create: `tests/drone/fixtures/waybeam_no_fps.json`
- Modify: `drone/Makefile`

- [ ] **Step 1: Create fixture `waybeam_basic.json`**

Write `tests/drone/fixtures/waybeam_basic.json` with the full sample shape (trimmed to what tests need — keep `video0` and `record` blocks so the collision test has both):

```json
{
  "system": {
    "webPort": 80,
    "overclockLevel": 1,
    "verbose": false
  },
  "video0": {
    "codec": "h265",
    "rcMode": "cbr",
    "fps": 60,
    "size": "1920x1080",
    "bitrate": 8192
  },
  "record": {
    "enabled": false,
    "dir": "/mnt/mmcblk0p1",
    "fps": 0,
    "bitrate": 0
  },
  "fpv": {
    "roiEnabled": true,
    "roiQp": 0
  }
}
```

- [ ] **Step 2: Create fixture `waybeam_compact.json`**

Write `tests/drone/fixtures/waybeam_compact.json` (same content, no whitespace, no newlines):

```json
{"system":{"webPort":80},"video0":{"codec":"h265","fps":60,"size":"1920x1080"},"record":{"fps":0}}
```

- [ ] **Step 3: Create fixture `waybeam_no_fps.json`**

Write `tests/drone/fixtures/waybeam_no_fps.json` (video0 block exists but lacks `fps`):

```json
{
  "video0": {
    "codec": "h265",
    "size": "1920x1080",
    "bitrate": 8192
  }
}
```

- [ ] **Step 4: Create the header `drone/src/dl_json_get.h`**

```c
/* dl_json_get.h — tiny scanner that finds `"<key>":<int>` inside a
 * top-level `"<block>":{ ... }` object. Robust enough for the one
 * specific JSON shape we read (/etc/waybeam.json). Not a general
 * JSON parser. */
#pragma once

/* Find `"<key>":<int>` inside a top-level `"<block>":{ ... }` object
 * in `path`. On success, writes the parsed integer into *out and
 * returns 0. On failure returns a negative errno-ish code: negative
 * errno on open failure (typically -ENOENT), -EINVAL for any
 * structural / parsing problem (block not found, key not found
 * inside block, non-integer value, file too large, etc.).
 * *out is only written on success. */
int dl_json_get_int(const char *path,
                    const char *block,
                    const char *key,
                    int *out);
```

- [ ] **Step 5: Write the failing test file `tests/drone/test_dl_json.c`**

```c
#include "dl_json_get.h"
#include "test_main.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#define FIX(name) "../tests/drone/fixtures/" name

DL_TEST(json_reads_video0_fps_from_basic) {
    int v = -1;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 60);
}

DL_TEST(json_reads_record_fps_distinctly_from_video0) {
    /* The block-anchoring invariant: record.fps = 0 must NOT be
     * returned when caller asks for video0.fps, and vice versa. */
    int v = -1;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "record", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 0);

    v = -1;
    rc = dl_json_get_int(FIX("waybeam_basic.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 60);
}

DL_TEST(json_returns_enoent_when_file_missing) {
    int v = 99;
    int rc = dl_json_get_int(FIX("does_not_exist.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, -ENOENT);
    DL_ASSERT_EQ(v, 99);  /* unchanged on failure */
}

DL_TEST(json_returns_einval_when_block_missing) {
    int v = 0;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "bogus", "fps", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(json_returns_einval_when_key_missing) {
    int v = 0;
    int rc = dl_json_get_int(FIX("waybeam_no_fps.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(json_returns_einval_for_non_integer_value) {
    int v = 0;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "video0", "size", &v);
    DL_ASSERT_EQ(rc, -EINVAL);
}

DL_TEST(json_handles_compact_no_whitespace) {
    int v = -1;
    int rc = dl_json_get_int(FIX("waybeam_compact.json"), "video0", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 60);
}

DL_TEST(json_accepts_zero_value) {
    int v = -1;
    int rc = dl_json_get_int(FIX("waybeam_basic.json"), "record", "fps", &v);
    DL_ASSERT_EQ(rc, 0);
    DL_ASSERT_EQ(v, 0);
}
```

- [ ] **Step 6: Wire test + (eventually) implementation into the Makefile**

Modify `drone/Makefile`. Add `$(SRCDIR)/dl_json_get.c` to `APPLIER_SRCS` (sorted alphabetically — between `dl_inject` if present and `dl_latency.c`, but `dl_inject.c` is in INJECT_SRCS only, so it goes between `dl_hello.c` and `dl_latency.c`). Add the same to `TEST_SRCS`, and add `$(TESTDIR)/test_dl_json.c` next to `test_dl_yaml.c` in TEST_SRCS.

Diff of `drone/Makefile`:

```diff
 APPLIER_SRCS := \
     $(SRCDIR)/dl_applier.c \
     $(SRCDIR)/dl_backend_enc.c \
     $(SRCDIR)/dl_backend_radio.c \
     $(SRCDIR)/dl_backend_tx.c \
     $(SRCDIR)/dl_config.c \
     $(SRCDIR)/dl_dbg.c \
     $(SRCDIR)/dl_dedup.c \
     $(SRCDIR)/dl_hello.c \
+    $(SRCDIR)/dl_json_get.c \
     $(SRCDIR)/dl_latency.c \
     $(SRCDIR)/dl_log.c \
     $(SRCDIR)/dl_mavlink.c \
     $(SRCDIR)/dl_osd.c \
     $(SRCDIR)/dl_watchdog.c \
     $(SRCDIR)/dl_wire.c \
     $(SRCDIR)/dl_yaml_get.c
```

```diff
 TEST_SRCS := \
     $(TESTDIR)/test_main.c \
     $(TESTDIR)/test_apply_stagger.c \
     $(TESTDIR)/test_wire.c \
     $(TESTDIR)/test_config.c \
     $(TESTDIR)/test_dbg.c \
     $(TESTDIR)/test_dedup.c \
     $(TESTDIR)/test_dl_latency.c \
     $(TESTDIR)/test_mavlink.c \
     $(TESTDIR)/test_watchdog.c \
+    $(TESTDIR)/test_dl_json.c \
     $(TESTDIR)/test_dl_yaml.c \
     $(TESTDIR)/test_dl_hello.c \
     $(SRCDIR)/dl_wire.c \
     $(SRCDIR)/dl_config.c \
     $(SRCDIR)/dl_dbg.c \
     $(SRCDIR)/dl_dedup.c \
     $(SRCDIR)/dl_hello.c \
+    $(SRCDIR)/dl_json_get.c \
     $(SRCDIR)/dl_latency.c \
     $(SRCDIR)/dl_log.c \
     $(SRCDIR)/dl_mavlink.c \
     $(SRCDIR)/dl_watchdog.c \
     $(SRCDIR)/dl_yaml_get.c
```

- [ ] **Step 7: Build to verify the test fails at link time**

Run: `make -C drone test 2>&1 | tail -20`

Expected: build fails with linker error `undefined reference to 'dl_json_get_int'` (or similar). The header is included but the .c file doesn't exist yet.

- [ ] **Step 8: Implement `drone/src/dl_json_get.c`**

```c
/* dl_json_get.c — see dl_json_get.h. */
#include "dl_json_get.h"

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FILE_BYTES (64 * 1024)

static int slurp(const char *path, char **out_buf, size_t *out_len) {
    FILE *fp = fopen(path, "rb");
    if (!fp) return -errno;
    if (fseek(fp, 0, SEEK_END) != 0) { fclose(fp); return -EIO; }
    long sz = ftell(fp);
    if (sz < 0 || sz > MAX_FILE_BYTES) { fclose(fp); return -EINVAL; }
    if (fseek(fp, 0, SEEK_SET) != 0) { fclose(fp); return -EIO; }
    char *buf = malloc((size_t)sz + 1);
    if (!buf) { fclose(fp); return -ENOMEM; }
    size_t got = fread(buf, 1, (size_t)sz, fp);
    fclose(fp);
    if (got != (size_t)sz) { free(buf); return -EIO; }
    buf[sz] = '\0';
    *out_buf = buf;
    *out_len = (size_t)sz;
    return 0;
}

/* Advance *pp past a JSON string. On entry *pp must point at the
 * opening `"`. On success *pp points just past the closing `"`.
 * Returns 0 on success, -1 if EOF arrives before the closing quote. */
static int skip_string(const char **pp, const char *end) {
    const char *p = *pp;
    if (p >= end || *p != '"') return -1;
    p++;
    while (p < end) {
        if (*p == '\\') {
            if (p + 1 >= end) return -1;
            p += 2;
            continue;
        }
        if (*p == '"') { *pp = p + 1; return 0; }
        p++;
    }
    return -1;
}

static const char *skip_ws(const char *p, const char *end) {
    while (p < end && (*p == ' ' || *p == '\t' ||
                       *p == '\n' || *p == '\r')) p++;
    return p;
}

/* Return pointer to the byte just past the opening `{` of the
 * top-level `"<block>":{...}` value, or NULL if not found. Only
 * matches a key whose string contents equal `block` exactly (no
 * escape sequences considered) and whose value starts with `{`. */
static const char *find_block_body(const char *buf, size_t len,
                                   const char *block) {
    size_t blen = strlen(block);
    const char *end = buf + len;
    const char *p = buf;
    int depth = 0;
    while (p < end) {
        if (*p == '"') {
            const char *q = p;
            if (skip_string(&q, end) != 0) return NULL;
            /* depth==1: we're a top-level key inside the outer object. */
            if (depth == 1 && (size_t)(q - p) == blen + 2 &&
                memcmp(p + 1, block, blen) == 0) {
                const char *r = skip_ws(q, end);
                if (r < end && *r == ':') {
                    r = skip_ws(r + 1, end);
                    if (r < end && *r == '{') return r + 1;
                }
            }
            p = q;
            continue;
        }
        if (*p == '{') depth++;
        else if (*p == '}') depth--;
        p++;
    }
    return NULL;
}

/* Within the block whose opening `{` has just been consumed, scan
 * for `"<key>":<int>` at depth 1 (immediate child of the block).
 * Returns 0 on success, -EINVAL on missing key, non-integer value,
 * or malformed content. */
static int find_key_int(const char *body, const char *end,
                        const char *key, int *out) {
    size_t klen = strlen(key);
    const char *p = body;
    int depth = 1;
    while (p < end && depth > 0) {
        if (*p == '"') {
            const char *q = p;
            if (skip_string(&q, end) != 0) return -EINVAL;
            if (depth == 1 && (size_t)(q - p) == klen + 2 &&
                memcmp(p + 1, key, klen) == 0) {
                const char *r = skip_ws(q, end);
                if (r >= end || *r != ':') return -EINVAL;
                r = skip_ws(r + 1, end);
                char *endp = NULL;
                errno = 0;
                long v = strtol(r, &endp, 10);
                if (errno != 0 || endp == r) return -EINVAL;
                if (v < INT_MIN || v > INT_MAX) return -EINVAL;
                const char *s = skip_ws(endp, end);
                if (s < end && *s != ',' && *s != '}') return -EINVAL;
                *out = (int)v;
                return 0;
            }
            p = q;
            continue;
        }
        if (*p == '{') depth++;
        else if (*p == '}') {
            depth--;
            if (depth == 0) break;
        }
        p++;
    }
    return -EINVAL;
}

int dl_json_get_int(const char *path, const char *block,
                    const char *key, int *out) {
    char *buf = NULL;
    size_t len = 0;
    int rc = slurp(path, &buf, &len);
    if (rc != 0) return rc;

    const char *body = find_block_body(buf, len, block);
    if (!body) { free(buf); return -EINVAL; }

    rc = find_key_int(body, buf + len, key, out);
    free(buf);
    return rc;
}
```

- [ ] **Step 9: Build and run the tests**

Run: `make -C drone test 2>&1 | tail -30`

Expected: build succeeds and the eight `json_*` tests all PASS. Existing tests still pass. Final summary line should report 0 failed.

- [ ] **Step 10: Commit**

```bash
git add drone/src/dl_json_get.h drone/src/dl_json_get.c \
        tests/drone/test_dl_json.c tests/drone/fixtures/waybeam_*.json \
        drone/Makefile
git commit -m "$(cat <<'EOF'
feat(drone): add dl_json_get helper

Mirror of dl_yaml_get for the one shape we'll read at boot
(/etc/waybeam.json). Pure top-level "<block>":{...} navigation
plus integer-value parse, with the same return convention
(-errno on open failure, -EINVAL on parse). Block-anchoring is
pinned by a unit test that distinguishes video0.fps from
record.fps in the same file.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Add `hello_waybeam_json_path` to `dl_config`

**Files:**
- Modify: `drone/src/dl_config.h:103` (add field after `hello_majestic_yaml_path`)
- Modify: `drone/src/dl_config.c:64` (default value) and `:215` (parser branch)
- Modify: `tests/drone/test_config.c` (assert default + parsing)

- [ ] **Step 1: Write the failing test**

Append to `tests/drone/test_config.c`:

```c
DL_TEST(config_hello_waybeam_json_path_default) {
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_STR_EQ(c.hello_waybeam_json_path, "/etc/waybeam.json");
}

DL_TEST(config_hello_waybeam_json_path_parses) {
    const char *body = "hello_waybeam_json_path = /tmp/wb.json\n";
    char path[64];
    DL_ASSERT_EQ(write_tmp(body, path, sizeof(path)), 0);
    dl_config_t c;
    dl_config_defaults(&c);
    DL_ASSERT_EQ(dl_config_load(path, &c), 0);
    DL_ASSERT_STR_EQ(c.hello_waybeam_json_path, "/tmp/wb.json");
    unlink(path);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `make -C drone test 2>&1 | tail -10`

Expected: compile fails with `'dl_config_t' has no member named 'hello_waybeam_json_path'` (or both tests FAIL after a successful compile, depending on which happens first).

- [ ] **Step 3: Add the field to `dl_config_t`**

In `drone/src/dl_config.h`, after the existing `hello_majestic_yaml_path` line (around line 103):

```c
    char     hello_wfb_yaml_path[DL_CONF_MAX_STR];
    char     hello_majestic_yaml_path[DL_CONF_MAX_STR];
    char     hello_waybeam_json_path[DL_CONF_MAX_STR];
} dl_config_t;
```

- [ ] **Step 4: Set the default**

In `drone/src/dl_config.c`, in `dl_config_defaults()` immediately after the existing `hello_majestic_yaml_path` line (around line 64):

```c
    strncpy(cfg->hello_wfb_yaml_path, "/etc/wfb.yaml", DL_CONF_MAX_STR - 1);
    strncpy(cfg->hello_majestic_yaml_path, "/etc/majestic.yaml", DL_CONF_MAX_STR - 1);
    strncpy(cfg->hello_waybeam_json_path, "/etc/waybeam.json", DL_CONF_MAX_STR - 1);
}
```

- [ ] **Step 5: Add the parser branch**

In `drone/src/dl_config.c`, in the parser if/else chain (after the `hello_majestic_yaml_path` branch around line 214-215):

```c
        else if (strcmp(key, "hello_majestic_yaml_path") == 0)
            SET_STR(hello_majestic_yaml_path);
        else if (strcmp(key, "hello_waybeam_json_path") == 0)
            SET_STR(hello_waybeam_json_path);
        else {
```

- [ ] **Step 6: Run tests**

Run: `make -C drone test 2>&1 | tail -10`

Expected: all tests PASS, including the two new `config_hello_waybeam_json_path_*` tests.

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_config.h drone/src/dl_config.c tests/drone/test_config.c
git commit -m "$(cat <<'EOF'
feat(drone): add hello_waybeam_json_path config field

Defaults to /etc/waybeam.json. Parsed via the same SET_STR
pattern as hello_majestic_yaml_path. Not yet wired into
dl_hello — that's the next commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Branch `dl_hello_init` on `encoder_kind`

**Files:**
- Modify: `drone/src/dl_hello.c:49-56` (the existing fps-read block)
- Modify: `tests/drone/test_dl_hello.c` (new waybeam-path test cases)
- Create: `tests/drone/fixtures/majestic_basic.yaml` is already present — reuse it.

- [ ] **Step 1: Write the failing tests**

Append to `tests/drone/test_dl_hello.c`. Note: `setup_cfg` points `hello_majestic_yaml_path` at the valid YAML fixture (fps=60), which means a buggy dispatch that ignores `encoder_kind` would still produce fps=60. To pin the dispatch branch, the positive-waybeam test explicitly clobbers the majestic path to a missing file so only the JSON read can succeed.

```c
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `make -C drone test 2>&1 | tail -30`

Expected: all three new tests FAIL — `hello_init_reads_fps_from_waybeam_json` because the current code tries the (now-missing) majestic YAML and bails; `hello_init_fails_when_waybeam_json_unreadable` because the current code tries the (still-valid) majestic YAML and *succeeds* — so the test fails with `rc != -1`; `hello_init_fails_for_unknown_encoder_kind` because the current code ignores `encoder_kind` and reads majestic YAML successfully, returning rc=0.

- [ ] **Step 3: Update `drone/src/dl_hello.c`**

Replace the existing fps-read block at lines 49-56:

```c
    int fps = 0;
    rc = dl_yaml_get_int(cfg->hello_majestic_yaml_path,
                         "video0", "fps", &fps);
    if (rc != 0) {
        dl_log_err("dl_hello: failed to read fps (%s video0.fps): %d",
                   cfg->hello_majestic_yaml_path, rc);
        return -1;
    }
```

with the dispatch on `encoder_kind`:

```c
    int fps = 0;
    if (strcmp(cfg->encoder_kind, "majestic") == 0) {
        rc = dl_yaml_get_int(cfg->hello_majestic_yaml_path,
                             "video0", "fps", &fps);
        if (rc != 0) {
            dl_log_err("dl_hello: failed to read fps (%s video0.fps): %d",
                       cfg->hello_majestic_yaml_path, rc);
            return -1;
        }
    } else if (strcmp(cfg->encoder_kind, "waybeam") == 0) {
        rc = dl_json_get_int(cfg->hello_waybeam_json_path,
                             "video0", "fps", &fps);
        if (rc != 0) {
            dl_log_err("dl_hello: failed to read fps (%s video0.fps): %d",
                       cfg->hello_waybeam_json_path, rc);
            return -1;
        }
    } else {
        dl_log_err("dl_hello: unknown encoder_kind for fps source: %s",
                   cfg->encoder_kind);
        return -1;
    }
```

Add the include near the top of the file (next to `#include "dl_yaml_get.h"`):

```c
#include "dl_json_get.h"
```

- [ ] **Step 4: Run tests**

Run: `make -C drone test 2>&1 | tail -10`

Expected: all tests PASS — including the three new `hello_init_*` tests and all pre-existing ones.

- [ ] **Step 5: Sanity-build the applier itself**

Run: `make -C drone 2>&1 | tail -10`

Expected: clean build of `drone/build/dl-applier` and `drone/build/dl-inject` with no warnings (`-Wall -Wextra -Wformat=2 -std=c11`).

- [ ] **Step 6: Commit**

```bash
git add drone/src/dl_hello.c tests/drone/test_dl_hello.c
git commit -m "$(cat <<'EOF'
feat(drone): dispatch dl_hello fps source on encoder_kind

majestic → /etc/majestic.yaml (existing path).
waybeam  → /etc/waybeam.json via dl_json_get.
Any other value (incl. "none") fails dl_hello_init loudly,
which keeps the applier running but suppresses DLHE — same
soft-fail semantics as a missing YAML.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Docs and config sample

**Files:**
- Modify: `conf/drone.conf.sample` (P4a block + encoder_kind comment)
- Modify: `README.md` (operator prerequisite)
- Modify: `CLAUDE.md` (Operational Prerequisites #4)

- [ ] **Step 1: Update `conf/drone.conf.sample`**

Replace the P4a block (around lines 133-146) so it documents both file paths and which `encoder_kind` selects which:

```
# --- P4a: drone→GS config handshake ----------------------------------
# Authoritative sources for MTU and FPS, read once at startup.
# MTU comes from /etc/wfb.yaml `wireless.mlink` (typically 1400 or
# 3994 depending on wfb-ng's radio_mtu setting).
#
# FPS source depends on `encoder_kind` above:
#   - majestic → hello_majestic_yaml_path (YAML, key `video0.fps`)
#   - waybeam  → hello_waybeam_json_path  (JSON, key `video0.fps`)
# Anything else (e.g. `none`) refuses to send HELLO; GS stays in
# safe_defaults until the operator picks a real encoder.
hello_wfb_yaml_path      = /etc/wfb.yaml
hello_majestic_yaml_path = /etc/majestic.yaml
hello_waybeam_json_path  = /etc/waybeam.json
```

- [ ] **Step 2: Update `README.md`**

Locate the operator-prerequisites / setup section (search for `majestic.yaml` or `video0.fps`) and add the waybeam variant. If the section reads "`/etc/majestic.yaml` must contain `video0.fps`", change it to read (preserving surrounding context):

> When `encoder_kind = majestic`: `/etc/majestic.yaml` must contain `video0.fps: <integer>`.
> When `encoder_kind = waybeam`: `/etc/waybeam.json` must contain `video0.fps` as an integer at the top-level `video0` object.

If no such section exists in README, add a one-paragraph note under the existing "Prerequisites" or "Operator setup" heading.

Run `grep -n "majestic" /workspace/README.md` first to locate the exact insertion point; if `majestic` is not mentioned in README, the spec's documentation requirement is satisfied by CLAUDE.md and the conf sample alone — skip the README edit.

- [ ] **Step 3: Update `CLAUDE.md`**

Locate the "Operational prerequisites" section (the numbered list around `### Operational prerequisites (wfb-ng master.cfg changes)`). Find item 4 (the `/etc/majestic.yaml` requirement):

> 4. `/etc/majestic.yaml` must contain `video0.fps: <integer>` —
>    same constraint as above, on the FPS side.

Replace with:

> 4. The fps source must be readable and contain an integer
>    `video0.fps`. Source depends on `encoder_kind` in drone.conf:
>    `majestic` → `/etc/majestic.yaml` (`video0.fps:` YAML key);
>    `waybeam` → `/etc/waybeam.json` (top-level `video0.fps` JSON
>    key). Missing or unparseable means the applier refuses to send
>    HELLO and the GS stays in safe_defaults.

- [ ] **Step 4: Verify the docs changes don't break anything**

Run: `make -C drone test 2>&1 | tail -5`

Expected: still 0 failed (docs are not compiled, but it's worth sanity-checking nothing regressed).

Also run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -q 2>&1 | tail -5` (per CLAUDE.md's documented test command).

Expected: all Python tests still pass — no GS-side change in this whole feature.

- [ ] **Step 5: Commit**

```bash
git add conf/drone.conf.sample CLAUDE.md
# Include README.md only if Step 2 actually modified it.
git status --porcelain README.md | grep -q . && git add README.md
git commit -m "$(cat <<'EOF'
docs: waybeam encoder fps source

Document the encoder_kind → fps-source dispatch in
drone.conf.sample, CLAUDE.md operational prerequisites, and
(if present) the README. No behavior change.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Final verification

- [ ] **Step 1: Full test sweep**

Run: `make -C drone test && python3 -m pytest --ignore=tests/test_mavlink_status.py -q`

Expected: both suites green. The C suite should report all original tests plus the new `json_*`, `config_hello_waybeam_json_path_*`, and `hello_init_*_waybeam_*` / `hello_init_fails_for_unknown_encoder_kind` tests passing.

- [ ] **Step 2: Clean build**

Run: `make -C drone clean && make -C drone 2>&1 | tail -10`

Expected: zero warnings under `-Wall -Wextra -Wformat=2 -std=c11`.

- [ ] **Step 3: Smoke `dl-inject --dry-run` for regression sanity**

Run: `drone/build/dl-inject --dry-run --mcs 5 --bandwidth 20 --tx-power 18 --k 8 --n 14 --depth 2 --bitrate 12000 --fps 60`

Expected: hex bytes printed; the existing `test_wire_contract.py` invariant is untouched by this work.

- [ ] **Step 4: Review log on the way out**

Run: `git log --oneline master ^origin/master`

Expected: four commits, in this order:
1. `feat(drone): add dl_json_get helper`
2. `feat(drone): add hello_waybeam_json_path config field`
3. `feat(drone): dispatch dl_hello fps source on encoder_kind`
4. `docs: waybeam encoder fps source`
