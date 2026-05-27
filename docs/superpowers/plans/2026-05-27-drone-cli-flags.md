# Drone applier per-field CLI flags — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose every relevant field in `drone.conf` as a `--kebab-case` CLI flag on `dl-applier`, with precedence built-in defaults → conf file → CLI overrides. `--config` becomes optional.

**Architecture:** Introduce three private field tables (`int`, `bool`, `string`) in `drone/src/dl_config.c` that act as the single source of truth for field names, offsets, types, and ranges. Both the conf-file parser (`dl_config_load`) and the new CLI parser (in `dl_applier.c`) consult these tables. The CLI parser builds its `getopt_long` option array dynamically from the tables and patches `dl_config_t` after the conf file has been loaded.

**Tech Stack:** C11, POSIX `getopt_long`, existing `DL_TEST(...)` C unit-test harness, pytest for Python e2e.

**Spec:** [`docs/superpowers/specs/2026-05-27-drone-cli-flags-design.md`](../specs/2026-05-27-drone-cli-flags-design.md).

**In-scope fields** (everything in `dl_config_t` except the Phase-3 debug-suite block):

- **Strings (13):** `listen_addr`, `wfb_tx_ctrl_addr`, `idr_listen_addr`, `osd_msg_path`, `wlan_dev`, `encoder_kind`, `encoder_host`, `mavlink_addr`, `gs_tunnel_addr`, `hello_wfb_yaml_path`, `hello_majestic_yaml_path`, `hello_waybeam_json_path`. (12 — `dbg_log_dir` is excluded.)
- **Bools (3 in scope):** `osd_enable`, `osd_debug_latency`, `interleaving_supported`, `mavlink_enable`. (4 — `debug_enable` and `dbg_fsync_each` excluded.)
- **Ints (~26 in scope):** `listen_port`, `wfb_tx_ctrl_port`, `min_idr_interval_ms`, `idr_listen_port`, `apply_stagger_ms`, `apply_sub_pace_ms`, `osd_update_interval_ms`, `health_timeout_ms`, `safe_k`, `safe_n`, `safe_depth`, `safe_mcs`, `safe_bandwidth`, `safe_tx_power_dBm`, `safe_bitrate_kbps`, `encoder_port`, `roi_qp_threshold_kbps`, `roi_qp_low_anchor_kbps`, `roi_qp_floor`, `roi_qp_step`, `mavlink_port`, `mavlink_sysid`, `mavlink_compid`, `gs_tunnel_port`, `hello_announce_initial_ms`, `hello_announce_steady_ms`, `hello_keepalive_ms`, `hello_announce_initial_count`. (28 — `dbg_log_enable` and `dbg_max_bytes` excluded.)

**Out of scope for CLI** (conf-file only): `debug_enable`, `dbg_log_enable`, `dbg_log_dir`, `dbg_max_bytes`, `dbg_fsync_each`.

---

## File Structure

**Files modified:**
- `drone/src/dl_config.h` — add new exported types (`dl_field_type_t`, `dl_int_field_t`, `dl_bool_field_t`, `dl_str_field_t`), iteration accessors, by-name setters, `dl_config_validate`.
- `drone/src/dl_config.c` — add field tables, by-name setters, validate function, refactor `dl_config_load` to dispatch via tables, extract `dl_parse_long_ranged` / `dl_parse_bool` helpers.
- `drone/src/dl_applier.c` — extend `cli_args_t` with override storage and "was-set" bit arrays, build dynamic `getopt_long` table from field iterators, generate `--help` text from tables, make `--config` optional, apply overrides before `dl_config_validate`.
- `drone/Makefile` — add `tests/drone/test_config_cli.c` to `TEST_SRCS`.
- `conf/drone.conf.sample` — no functional change; add a one-line note at the top pointing at `--help`.
- `README.md` — new "CLI overrides" subsection.

**Files created:**
- `tests/drone/test_config_cli.c` — C unit tests for the field tables, by-name setters, and `dl_config_validate`.
- `tests/test_drone_cli_overrides.py` — Python e2e tests for CLI overrides end-to-end against `dl-applier`.

---

## Task 1: Extract shared parsing helpers

**Files:**
- Modify: `drone/src/dl_config.c`

This is a no-behavior-change refactor. Today `dl_config.c` has `static int parse_int(...)` and `static int parse_bool(...)` plus the macros `SET_INT_RANGED` / `SET_BOOL`. Rename and stabilize the helper signatures so they can later be called from outside the `dl_config_load` loop without touching `path`/`lineno`.

- [ ] **Step 1: Rename `parse_int` → `dl_parse_long_ranged` and add range params**

Edit `drone/src/dl_config.c`. Replace the existing `static int parse_int(const char *val, long *out)` (around line 86) with:

```c
/* Parse a base-10 long out of `s`; reject empty, trailing garbage, or
 * out-of-range [lo..hi]. Returns 0 on success, -1 on failure. */
static int dl_parse_long_ranged(const char *s, long lo, long hi, long *out) {
    char *end;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0') return -1;
    if (v < lo || v > hi) return -1;
    *out = v;
    return 0;
}
```

Rename `parse_bool` to `dl_parse_bool` (signature unchanged):

```c
static int dl_parse_bool(const char *val, bool *out) {
    if (strcasecmp(val, "1") == 0 || strcasecmp(val, "true") == 0 ||
        strcasecmp(val, "yes") == 0 || strcasecmp(val, "on") == 0) {
        *out = true;  return 0;
    }
    if (strcasecmp(val, "0") == 0 || strcasecmp(val, "false") == 0 ||
        strcasecmp(val, "no") == 0 || strcasecmp(val, "off") == 0) {
        *out = false; return 0;
    }
    return -1;
}
```

Rewrite the `SET_INT_RANGED` macro to call the new helper:

```c
#define SET_INT_RANGED(field, type, lo, hi) do {                       \
    long _v;                                                           \
    if (dl_parse_long_ranged(val, (long)(lo), (long)(hi), &_v) != 0) { \
        dl_log_err("%s:%d: bad value for %s: %s",                      \
                   path, lineno, key, val);                            \
        rc = -1; continue;                                             \
    }                                                                  \
    cfg->field = (type)_v;                                             \
} while(0)
```

And `SET_BOOL`:

```c
#define SET_BOOL(field) do {                                           \
    bool _v;                                                           \
    if (dl_parse_bool(val, &_v) != 0) {                                \
        dl_log_err("%s:%d: bad bool for %s: %s",                       \
                   path, lineno, key, val);                            \
        rc = -1; continue;                                             \
    }                                                                  \
    cfg->field = _v;                                                   \
} while(0)
```

- [ ] **Step 2: Build and run existing tests**

Run: `make -C drone test`
Expected: all current tests pass (no behavior change).

- [ ] **Step 3: Commit**

```bash
git add drone/src/dl_config.c
git commit -m "refactor(dl_config): rename parse_* helpers and inline range check

Extract the range bounds into dl_parse_long_ranged() so it can be
reused by the upcoming CLI parser. Pure refactor; no behavior change.
"
```

---

## Task 2: Add the field-table types and tables

**Files:**
- Modify: `drone/src/dl_config.h`
- Modify: `drone/src/dl_config.c`

Introduce the data tables but do not yet rewire `dl_config_load`. The new tables and accessor functions exist alongside the existing if/else dispatch; Task 4 swaps the loader over.

- [ ] **Step 1: Add the types and accessor declarations to the header**

In `drone/src/dl_config.h`, after the `dl_config_t` definition and before `dl_config_defaults`, add:

```c
/* ---- Field table API ----------------------------------------------
 * The CLI parser in dl_applier walks these tables to build its
 * getopt_long option list, and the conf parser uses them as the
 * authoritative source for ranges and offsets. */

typedef enum {
    DL_F_U8, DL_F_I8, DL_F_U16, DL_F_U32,
} dl_field_type_t;

typedef struct {
    const char     *name;       /* exact conf key; CLI flag is name with _ → - */
    size_t          offset;     /* offsetof(dl_config_t, …) */
    dl_field_type_t type;
    long            lo, hi;     /* inclusive */
} dl_int_field_t;

typedef struct { const char *name; size_t offset; } dl_bool_field_t;
typedef struct { const char *name; size_t offset; } dl_str_field_t;

const dl_int_field_t  *dl_config_int_fields (size_t *n_out);
const dl_bool_field_t *dl_config_bool_fields(size_t *n_out);
const dl_str_field_t  *dl_config_str_fields (size_t *n_out);
```

- [ ] **Step 2: Add the tables to `dl_config.c`**

In `drone/src/dl_config.c`, add at file scope just below the includes:

```c
#include <stddef.h>   /* offsetof */

#define F_INT(name_, type_, lo_, hi_) \
    { #name_, offsetof(dl_config_t, name_), type_, (long)(lo_), (long)(hi_) }
#define F_BOOL(name_) { #name_, offsetof(dl_config_t, name_) }
#define F_STR(name_)  { #name_, offsetof(dl_config_t, name_) }

static const dl_int_field_t DL_INT_FIELDS[] = {
    F_INT(listen_port,                   DL_F_U16, 1,      65535),
    F_INT(wfb_tx_ctrl_port,              DL_F_U16, 1,      65535),
    F_INT(min_idr_interval_ms,           DL_F_U32, 0,      60000),
    F_INT(idr_listen_port,               DL_F_U16, 0,      65535),
    F_INT(apply_stagger_ms,              DL_F_U32, 0,      500),
    F_INT(apply_sub_pace_ms,             DL_F_U32, 0,      50),
    F_INT(osd_update_interval_ms,        DL_F_U32, 100,    60000),
    F_INT(health_timeout_ms,             DL_F_U32, 500,    120000),
    F_INT(safe_k,                        DL_F_U8,  1,      32),
    F_INT(safe_n,                        DL_F_U8,  2,      255),
    F_INT(safe_depth,                    DL_F_U8,  1,      8),
    F_INT(safe_mcs,                      DL_F_U8,  0,      7),
    F_INT(safe_bandwidth,                DL_F_U8,  20,     40),
    F_INT(safe_tx_power_dBm,             DL_F_I8,  -10,    30),
    F_INT(safe_bitrate_kbps,             DL_F_U16, 100,    65535),
    F_INT(encoder_port,                  DL_F_U16, 1,      65535),
    F_INT(roi_qp_threshold_kbps,         DL_F_U16, 100,    65535),
    F_INT(roi_qp_low_anchor_kbps,        DL_F_U16, 100,    65535),
    F_INT(roi_qp_floor,                  DL_F_I8,  -30,    0),
    F_INT(roi_qp_step,                   DL_F_U8,  1,      10),
    F_INT(mavlink_port,                  DL_F_U16, 1,      65535),
    F_INT(mavlink_sysid,                 DL_F_U8,  0,      255),
    F_INT(mavlink_compid,                DL_F_U8,  0,      255),
    F_INT(gs_tunnel_port,                DL_F_U16, 1,      65535),
    F_INT(hello_announce_initial_ms,     DL_F_U32, 1,      60000),
    F_INT(hello_announce_steady_ms,      DL_F_U32, 1,      300000),
    F_INT(hello_keepalive_ms,            DL_F_U32, 1,      300000),
    F_INT(hello_announce_initial_count,  DL_F_U32, 0,      100000),
};

static const dl_bool_field_t DL_BOOL_FIELDS[] = {
    F_BOOL(osd_enable),
    F_BOOL(osd_debug_latency),
    F_BOOL(interleaving_supported),
    F_BOOL(mavlink_enable),
};

static const dl_str_field_t DL_STR_FIELDS[] = {
    F_STR(listen_addr),
    F_STR(wfb_tx_ctrl_addr),
    F_STR(idr_listen_addr),
    F_STR(osd_msg_path),
    F_STR(wlan_dev),
    F_STR(encoder_kind),
    F_STR(encoder_host),
    F_STR(mavlink_addr),
    F_STR(gs_tunnel_addr),
    F_STR(hello_wfb_yaml_path),
    F_STR(hello_majestic_yaml_path),
    F_STR(hello_waybeam_json_path),
};

const dl_int_field_t *dl_config_int_fields(size_t *n_out) {
    if (n_out) *n_out = sizeof(DL_INT_FIELDS) / sizeof(DL_INT_FIELDS[0]);
    return DL_INT_FIELDS;
}
const dl_bool_field_t *dl_config_bool_fields(size_t *n_out) {
    if (n_out) *n_out = sizeof(DL_BOOL_FIELDS) / sizeof(DL_BOOL_FIELDS[0]);
    return DL_BOOL_FIELDS;
}
const dl_str_field_t *dl_config_str_fields(size_t *n_out) {
    if (n_out) *n_out = sizeof(DL_STR_FIELDS) / sizeof(DL_STR_FIELDS[0]);
    return DL_STR_FIELDS;
}
```

The Phase-3 debug-suite fields (`debug_enable`, `dbg_log_enable`, `dbg_log_dir`, `dbg_max_bytes`, `dbg_fsync_each`) are deliberately absent from these tables — they remain handled by the legacy if/else chain in `dl_config_load` (see Task 4).

- [ ] **Step 2.5: Decide on a kebab-case helper**

Both the loader (Task 4) and the CLI parser (Task 6) need to turn a CLI flag string (`mavlink-enable`) back into a conf key string (`mavlink_enable`) for lookup. Add this helper to `dl_config.c` as `static`:

```c
/* Copy `src` into `dst` (size `dstlen`) replacing '-' with '_'.
 * Truncates safely. Used to convert CLI flag names back to conf
 * keys for table lookup. */
static void dl_kebab_to_snake(char *dst, size_t dstlen, const char *src) {
    size_t i = 0;
    for (; src[i] && i + 1 < dstlen; i++)
        dst[i] = (src[i] == '-') ? '_' : src[i];
    dst[i] = '\0';
}
```

It is `static` for now; Task 6 will move it to the header if the CLI parser needs it (currently the CLI parser will keep an exact-copy of the kebab name in the `struct option` table, so this is only used by the by-name setters in Task 3).

- [ ] **Step 3: Build to verify nothing breaks**

Run: `make -C drone test`
Expected: all current tests pass. The new tables are unused for now — no test should reference them yet.

- [ ] **Step 4: Commit**

```bash
git add drone/src/dl_config.h drone/src/dl_config.c
git commit -m "feat(dl_config): introduce field-table accessors

Add private DL_INT_FIELDS / DL_BOOL_FIELDS / DL_STR_FIELDS tables and
public iteration accessors. The tables will replace the if/else chain
in dl_config_load (next task) and drive the upcoming CLI parser.
Behavior unchanged in this commit; tables are exported but unused.
"
```

---

## Task 3: Add by-name setter API + unit tests

**Files:**
- Modify: `drone/src/dl_config.h`
- Modify: `drone/src/dl_config.c`
- Create: `tests/drone/test_config_cli.c`
- Modify: `drone/Makefile`

- [ ] **Step 1: Write the failing test file**

Create `tests/drone/test_config_cli.c`:

```c
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
```

- [ ] **Step 2: Wire the new test file into the Makefile**

Modify `drone/Makefile`: in `TEST_SRCS`, add `$(TESTDIR)/test_config_cli.c` immediately after `$(TESTDIR)/test_config.c`. The block becomes:

```make
TEST_SRCS := \
    $(TESTDIR)/test_main.c \
    $(TESTDIR)/test_apply_stagger.c \
    $(TESTDIR)/test_wire.c \
    $(TESTDIR)/test_config.c \
    $(TESTDIR)/test_config_cli.c \
    $(TESTDIR)/test_dbg.c \
    ...
```

- [ ] **Step 3: Verify tests fail to link**

Run: `make -C drone test`
Expected: compile/link failure — `dl_config_set_int_by_name` / `dl_config_set_bool_by_name` / `dl_config_set_str_by_name` are undeclared.

- [ ] **Step 4: Declare the setters in `dl_config.h`**

Add to `drone/src/dl_config.h`, near the iteration accessors added in Task 2:

```c
/* Set a single int/bool/str field by its conf-key name (or its
 * kebab equivalent). Returns 0 on success, -1 on unknown name,
 * range violation, parse failure, or string overflow. Callers
 * provide their own logging context. */
int dl_config_set_int_by_name (dl_config_t *cfg, const char *name, const char *val);
int dl_config_set_bool_by_name(dl_config_t *cfg, const char *name, bool        val);
int dl_config_set_str_by_name (dl_config_t *cfg, const char *name, const char *val);
```

- [ ] **Step 5: Implement the setters in `dl_config.c`**

Add to `drone/src/dl_config.c`, below the field tables and accessor functions added in Task 2:

```c
/* Write the parsed value into the int field referenced by `f`. */
static void write_int_field(dl_config_t *cfg, const dl_int_field_t *f, long v) {
    void *p = (char *)cfg + f->offset;
    switch (f->type) {
    case DL_F_U8:  *(uint8_t  *)p = (uint8_t )v; break;
    case DL_F_I8:  *(int8_t   *)p = (int8_t  )v; break;
    case DL_F_U16: *(uint16_t *)p = (uint16_t)v; break;
    case DL_F_U32: *(uint32_t *)p = (uint32_t)v; break;
    }
}

int dl_config_set_int_by_name(dl_config_t *cfg, const char *name, const char *val) {
    char norm[64];
    dl_kebab_to_snake(norm, sizeof(norm), name);
    size_t n = 0;
    const dl_int_field_t *t = dl_config_int_fields(&n);
    for (size_t i = 0; i < n; i++) {
        if (strcmp(t[i].name, norm) != 0) continue;
        long v;
        if (dl_parse_long_ranged(val, t[i].lo, t[i].hi, &v) != 0) return -1;
        write_int_field(cfg, &t[i], v);
        return 0;
    }
    return -1;
}

int dl_config_set_bool_by_name(dl_config_t *cfg, const char *name, bool val) {
    char norm[64];
    dl_kebab_to_snake(norm, sizeof(norm), name);
    size_t n = 0;
    const dl_bool_field_t *t = dl_config_bool_fields(&n);
    for (size_t i = 0; i < n; i++) {
        if (strcmp(t[i].name, norm) != 0) continue;
        *(bool *)((char *)cfg + t[i].offset) = val;
        return 0;
    }
    return -1;
}

int dl_config_set_str_by_name(dl_config_t *cfg, const char *name, const char *val) {
    if (strlen(val) >= DL_CONF_MAX_STR) return -1;
    char norm[64];
    dl_kebab_to_snake(norm, sizeof(norm), name);
    size_t n = 0;
    const dl_str_field_t *t = dl_config_str_fields(&n);
    for (size_t i = 0; i < n; i++) {
        if (strcmp(t[i].name, norm) != 0) continue;
        char *dst = (char *)cfg + t[i].offset;
        snprintf(dst, DL_CONF_MAX_STR, "%s", val);
        return 0;
    }
    return -1;
}
```

- [ ] **Step 6: Run tests**

Run: `make -C drone test`
Expected: all tests pass, including the new ones in `test_config_cli.c`.

- [ ] **Step 7: Commit**

```bash
git add drone/src/dl_config.h drone/src/dl_config.c drone/Makefile tests/drone/test_config_cli.c
git commit -m "feat(dl_config): by-name setters for int/bool/str fields

dl_config_set_{int,bool,str}_by_name dispatch through the field
tables. Accept both conf-key (snake) and CLI-flag (kebab) names.
Tests cover hits, misses, ranges, overflow, and the Phase-3
debug-suite carve-out.
"
```

---

## Task 4: Rewrite `dl_config_load` to dispatch via the tables

**Files:**
- Modify: `drone/src/dl_config.c`

Replace the if/else chain in `dl_config_load` with three table lookups, keeping the legacy-removed-key stanza and the Phase-3 debug-suite block as the only hand-written keys. After this, every in-scope field is in exactly one place: the tables.

- [ ] **Step 1: Update `dl_config_load`**

In `drone/src/dl_config.c`, the giant `if/else if` chain in `dl_config_load` (currently around lines 163–238) gets replaced. The new dispatch order is: legacy-removed-key error → table-driven int → table-driven bool → table-driven str → hand-written Phase-3 debug-suite keys → unknown-key warning.

Replace the block starting `if (strcmp(key, "listen_addr") == 0) ...` down to (but **not** including) the `if (cfg->roi_qp_threshold_kbps <= ...)` postamble. New body:

```c
        /* Removed-key error: keys explicitly retired in 2026-05-11
         * still exist in old configs; surface them loudly. */
        if (strcmp(key, "video_k_min")       == 0 ||
            strcmp(key, "video_k_max")       == 0 ||
            strcmp(key, "video_n_max")       == 0 ||
            strcmp(key, "depth_max")         == 0 ||
            strcmp(key, "mcs_max")           == 0 ||
            strcmp(key, "tx_power_min_dBm")  == 0 ||
            strcmp(key, "tx_power_max_dBm")  == 0) {
            dl_log_err("%s:%d: %s is no longer supported "
                       "(removed 2026-05-11); the drone applies whatever "
                       "the GS sends", path, lineno, key);
            rc = -1;
            continue;
        }

        /* Table-driven int dispatch. */
        {
            size_t n = 0;
            const dl_int_field_t *t = dl_config_int_fields(&n);
            int matched = 0;
            for (size_t i = 0; i < n; i++) {
                if (strcmp(t[i].name, key) != 0) continue;
                long v;
                if (dl_parse_long_ranged(val, t[i].lo, t[i].hi, &v) != 0) {
                    dl_log_err("%s:%d: bad value for %s: %s",
                               path, lineno, key, val);
                    rc = -1;
                } else {
                    write_int_field(cfg, &t[i], v);
                }
                matched = 1; break;
            }
            if (matched) continue;
        }

        /* Table-driven bool dispatch. */
        {
            size_t n = 0;
            const dl_bool_field_t *t = dl_config_bool_fields(&n);
            int matched = 0;
            for (size_t i = 0; i < n; i++) {
                if (strcmp(t[i].name, key) != 0) continue;
                bool v;
                if (dl_parse_bool(val, &v) != 0) {
                    dl_log_err("%s:%d: bad bool for %s: %s",
                               path, lineno, key, val);
                    rc = -1;
                } else {
                    *(bool *)((char *)cfg + t[i].offset) = v;
                }
                matched = 1; break;
            }
            if (matched) continue;
        }

        /* Table-driven str dispatch. */
        {
            size_t n = 0;
            const dl_str_field_t *t = dl_config_str_fields(&n);
            int matched = 0;
            for (size_t i = 0; i < n; i++) {
                if (strcmp(t[i].name, key) != 0) continue;
                if (strlen(val) >= DL_CONF_MAX_STR) {
                    dl_log_err("%s:%d: value too long for %s",
                               path, lineno, key);
                    rc = -1;
                } else {
                    char *dst = (char *)cfg + t[i].offset;
                    snprintf(dst, DL_CONF_MAX_STR, "%s", val);
                }
                matched = 1; break;
            }
            if (matched) continue;
        }

        /* Phase-3 debug-suite keys (not in CLI scope; hand-written). */
        if      (strcmp(key, "debug_enable")   == 0) SET_BOOL(debug_enable);
        else if (strcmp(key, "dbg_log_enable") == 0) SET_INT_RANGED(dbg_log_enable, int8_t, -1, 1);
        else if (strcmp(key, "dbg_log_dir")    == 0) SET_STR(dbg_log_dir);
        else if (strcmp(key, "dbg_max_bytes")  == 0) SET_INT_RANGED(dbg_max_bytes, uint32_t, 4096, 1 << 30);
        else if (strcmp(key, "dbg_fsync_each") == 0) SET_BOOL(dbg_fsync_each);
        else {
            dl_log_warn("%s:%d: unknown key: %s", path, lineno, key);
        }
```

Leave `SET_STR` / `SET_BOOL` / `SET_INT_RANGED` macros defined (they're still used by the Phase-3 fallback). The `dl_kebab_to_snake` helper is left alone — it's only used by the by-name setters added in Task 3.

- [ ] **Step 2: Run all tests**

Run: `make -C drone test`
Expected: every existing config test still passes (the dispatch behavior is unchanged at the semantic level).

Run: `python3 -m pytest tests/ --ignore=tests/test_mavlink_status.py`
Expected: still green.

- [ ] **Step 3: Sanity-check against the sample conf**

Run: `make -C drone && drone/build/dl-applier --config conf/drone.conf.sample --help 2>&1 | head -3 || true`

Note this command exits with usage; we're checking the conf parses without complaints. Also exercise the parser via tests:

Run: `make -C drone test 2>&1 | grep -E 'FAIL|PASS' | head -20`
Expected: no FAIL lines.

- [ ] **Step 4: Commit**

```bash
git add drone/src/dl_config.c
git commit -m "refactor(dl_config): drive dl_config_load from field tables

Replace the per-key if/else chain with three table-driven dispatch
blocks (int / bool / str). The Phase-3 debug-suite fields stay as a
hand-written fallback since they're intentionally absent from the
public field tables. Behavior is unchanged.
"
```

---

## Task 5: Add `dl_config_validate` and move cross-field check into it

**Files:**
- Modify: `drone/src/dl_config.h`
- Modify: `drone/src/dl_config.c`
- Modify: `tests/drone/test_config_cli.c`

Today the `roi_qp_threshold_kbps > roi_qp_low_anchor_kbps` invariant is checked inline at the end of `dl_config_load`. The CLI parser needs to re-run that check after applying overrides, so extract it.

- [ ] **Step 1: Write a failing test**

Append to `tests/drone/test_config_cli.c`:

```c
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
```

- [ ] **Step 2: Verify they fail to link**

Run: `make -C drone test`
Expected: link failure — `dl_config_validate` is undeclared.

- [ ] **Step 3: Declare and implement `dl_config_validate`**

In `drone/src/dl_config.h`, after the by-name setters:

```c
/* Check cross-field invariants. Returns 0 if all invariants hold,
 * -1 otherwise (each violation is logged via dl_log_err). Callers
 * should run this after any path that mutates `cfg` (conf-file
 * load, CLI override). */
int dl_config_validate(const dl_config_t *cfg);
```

In `drone/src/dl_config.c`, add (just before `dl_config_dbg_log_resolved` is fine):

```c
int dl_config_validate(const dl_config_t *cfg) {
    int rc = 0;
    if (cfg->roi_qp_threshold_kbps <= cfg->roi_qp_low_anchor_kbps) {
        dl_log_err("config: roi_qp_threshold_kbps (%u) must be > "
                   "roi_qp_low_anchor_kbps (%u)",
                   (unsigned)cfg->roi_qp_threshold_kbps,
                   (unsigned)cfg->roi_qp_low_anchor_kbps);
        rc = -1;
    }
    return rc;
}
```

Then in the body of `dl_config_load`, replace the existing post-loop check:

```c
    if (cfg->roi_qp_threshold_kbps <= cfg->roi_qp_low_anchor_kbps) {
        dl_log_err("%s: roi_qp_threshold_kbps (%u) must be > "
                   "roi_qp_low_anchor_kbps (%u)",
                   path,
                   (unsigned)cfg->roi_qp_threshold_kbps,
                   (unsigned)cfg->roi_qp_low_anchor_kbps);
        rc = -1;
    }
```

with:

```c
    if (dl_config_validate(cfg) != 0) rc = -1;
```

- [ ] **Step 4: Run tests**

Run: `make -C drone test`
Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add drone/src/dl_config.h drone/src/dl_config.c tests/drone/test_config_cli.c
git commit -m "feat(dl_config): extract cross-field invariants into dl_config_validate

Currently inline in dl_config_load; needs to run again after the
CLI override layer lands. No behavior change for the conf-file
path.
"
```

---

## Task 6: Make `--config` optional and add CLI overrides in `dl_applier`

**Files:**
- Modify: `drone/src/dl_applier.c`

This is the main user-visible change. The plan keeps the existing `--config`, `--debug`, `--help` semantics and bolts the per-field flags on alongside.

- [ ] **Step 1: Replace `cli_args_t` and `parse_args`**

In `drone/src/dl_applier.c`, replace the `cli_args_t` struct, the `usage()` function, and `parse_args()` (currently lines ~168–206) with the implementation below. The `main()` body that follows is updated in Step 2.

```c
/* ---- CLI parsing -------------------------------------------------
 * Layered config: defaults → conf file (if --config) → CLI overrides.
 * Per-field flags are generated from the dl_config field tables; bit
 * arrays remember which fields the user passed so unset CLI flags
 * never clobber conf-file values.
 */
typedef struct {
    const char  *config_path;     /* may be NULL */
    bool         log_debug;
    bool         help_requested;
    dl_config_t  overrides;       /* only fields with set_*[i]=true are valid */
    /* one bit per index into DL_*_FIELDS tables */
    uint8_t      set_int [(64 + 7) / 8];   /* sized generously; assert below */
    uint8_t      set_bool[(16 + 7) / 8];
    uint8_t      set_str [(16 + 7) / 8];
} cli_args_t;

static inline void bit_set (uint8_t *a, size_t i) { a[i >> 3] |= (uint8_t)(1u << (i & 7)); }
static inline bool bit_test(const uint8_t *a, size_t i) { return (a[i >> 3] >> (i & 7)) & 1u; }

/* Long-option `val` encoding: high byte = category, low byte = index.
 * 0x10xx = int field, 0x20xx = bool field, 0x30xx = str field. */
enum {
    OPT_INT_BASE  = 0x1000,
    OPT_BOOL_BASE = 0x2000,
    OPT_STR_BASE  = 0x3000,
};

/* Convert a field name (snake_case) to a heap-allocated kebab-case
 * copy for use as a long-option name. Caller owns the storage; never
 * freed (lives until process exit). */
static char *xstrdup_kebab(const char *snake) {
    size_t n = strlen(snake);
    char *out = (char *)malloc(n + 1);
    if (!out) { perror("malloc"); exit(2); }
    for (size_t i = 0; i < n; i++) out[i] = (snake[i] == '_') ? '-' : snake[i];
    out[n] = '\0';
    return out;
}

static const char *type_label(dl_field_type_t t) {
    switch (t) {
    case DL_F_U8:  return "u8";
    case DL_F_I8:  return "i8";
    case DL_F_U16: return "u16";
    case DL_F_U32: return "u32";
    }
    return "?";
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [--config <drone.conf>] [--debug] [field-overrides...]\n"
        "\n"
        "  --config PATH      path to drone.conf (optional; defaults used if omitted)\n"
        "  --debug            enable DEBUG-level logging\n"
        "  --help             print this help and exit\n"
        "\n"
        "Field overrides (CLI > conf file > defaults):\n",
        prog);

    size_t n;
    const dl_int_field_t *ti = dl_config_int_fields(&n);
    fprintf(stderr, "\n  Integer fields (--name VALUE):\n");
    for (size_t i = 0; i < n; i++) {
        char kebab[64];
        for (size_t k = 0; ti[i].name[k] && k + 1 < sizeof(kebab); k++)
            kebab[k] = (ti[i].name[k] == '_') ? '-' : ti[i].name[k];
        kebab[strlen(ti[i].name)] = '\0';
        fprintf(stderr, "    --%-32s %s, range [%ld..%ld]\n",
                kebab, type_label(ti[i].type), ti[i].lo, ti[i].hi);
    }

    const dl_bool_field_t *tb = dl_config_bool_fields(&n);
    fprintf(stderr, "\n  Boolean switches (set to true when passed):\n");
    for (size_t i = 0; i < n; i++) {
        char kebab[64];
        for (size_t k = 0; tb[i].name[k] && k + 1 < sizeof(kebab); k++)
            kebab[k] = (tb[i].name[k] == '_') ? '-' : tb[i].name[k];
        kebab[strlen(tb[i].name)] = '\0';
        fprintf(stderr, "    --%s\n", kebab);
    }

    const dl_str_field_t *ts = dl_config_str_fields(&n);
    fprintf(stderr, "\n  String fields (--name VALUE):\n");
    for (size_t i = 0; i < n; i++) {
        char kebab[64];
        for (size_t k = 0; ts[i].name[k] && k + 1 < sizeof(kebab); k++)
            kebab[k] = (ts[i].name[k] == '_') ? '-' : ts[i].name[k];
        kebab[strlen(ts[i].name)] = '\0';
        fprintf(stderr, "    --%s\n", kebab);
    }
    fprintf(stderr, "\n");
}

static int parse_args(int argc, char **argv, cli_args_t *out) {
    memset(out, 0, sizeof(*out));

    size_t n_int = 0, n_bool = 0, n_str = 0;
    const dl_int_field_t  *ti = dl_config_int_fields (&n_int);
    const dl_bool_field_t *tb = dl_config_bool_fields(&n_bool);
    const dl_str_field_t  *ts = dl_config_str_fields (&n_str);

    /* Bounds-check against the set_* bit arrays. */
    if (n_int  > sizeof(out->set_int)  * 8 ||
        n_bool > sizeof(out->set_bool) * 8 ||
        n_str  > sizeof(out->set_str)  * 8) {
        fprintf(stderr, "internal error: field table outgrew set_* bit arrays\n");
        return -1;
    }

    /* Build the option table: 3 fixed entries + per-field + terminator. */
    size_t total = 3 + n_int + n_bool + n_str + 1;
    struct option *opts = (struct option *)calloc(total, sizeof(struct option));
    if (!opts) { perror("calloc"); return -1; }

    size_t k = 0;
    opts[k++] = (struct option){ "config", required_argument, 0, 'c' };
    opts[k++] = (struct option){ "debug",  no_argument,       0, 'd' };
    opts[k++] = (struct option){ "help",   no_argument,       0, 'h' };
    for (size_t i = 0; i < n_int; i++)
        opts[k++] = (struct option){ xstrdup_kebab(ti[i].name), required_argument, 0, (int)(OPT_INT_BASE  + i) };
    for (size_t i = 0; i < n_bool; i++)
        opts[k++] = (struct option){ xstrdup_kebab(tb[i].name), no_argument,       0, (int)(OPT_BOOL_BASE + i) };
    for (size_t i = 0; i < n_str; i++)
        opts[k++] = (struct option){ xstrdup_kebab(ts[i].name), required_argument, 0, (int)(OPT_STR_BASE  + i) };
    opts[k] = (struct option){ 0 };

    /* Seed overrides with defaults so write_*_by_name has a valid
     * target type to copy into; only fields with set_*[i]=true are
     * later copied out. */
    dl_config_defaults(&out->overrides);

    int c;
    while ((c = getopt_long(argc, argv, "c:dh", opts, NULL)) != -1) {
        if (c == 'c')      out->config_path = optarg;
        else if (c == 'd') out->log_debug = true;
        else if (c == 'h') { out->help_requested = true; usage(argv[0]); free(opts); return 1; }
        else if (c >= OPT_INT_BASE && c < OPT_INT_BASE + (int)n_int) {
            size_t i = (size_t)(c - OPT_INT_BASE);
            if (dl_config_set_int_by_name(&out->overrides, ti[i].name, optarg) != 0) {
                fprintf(stderr, "--%s: bad value %s (expected %s in [%ld..%ld])\n",
                        opts[3 + i].name, optarg, type_label(ti[i].type), ti[i].lo, ti[i].hi);
                free(opts); return -1;
            }
            bit_set(out->set_int, i);
        }
        else if (c >= OPT_BOOL_BASE && c < OPT_BOOL_BASE + (int)n_bool) {
            size_t i = (size_t)(c - OPT_BOOL_BASE);
            if (dl_config_set_bool_by_name(&out->overrides, tb[i].name, true) != 0) {
                fprintf(stderr, "--%s: internal error setting bool\n", tb[i].name);
                free(opts); return -1;
            }
            bit_set(out->set_bool, i);
        }
        else if (c >= OPT_STR_BASE && c < OPT_STR_BASE + (int)n_str) {
            size_t i = (size_t)(c - OPT_STR_BASE);
            if (dl_config_set_str_by_name(&out->overrides, ts[i].name, optarg) != 0) {
                fprintf(stderr, "--%s: value rejected (too long? max %d)\n",
                        ts[i].name, DL_CONF_MAX_STR - 1);
                free(opts); return -1;
            }
            bit_set(out->set_str, i);
        }
        else {
            usage(argv[0]);
            free(opts);
            return -1;
        }
    }
    free(opts);
    /* `out->config_path` may legitimately be NULL → caller skips load. */
    return 0;
}

/* Copy fields the user passed from `args->overrides` into `cfg`. */
static void apply_cli_overrides(dl_config_t *cfg, const cli_args_t *args) {
    size_t n;
    const dl_int_field_t  *ti = dl_config_int_fields (&n);
    for (size_t i = 0; i < n; i++) {
        if (!bit_test(args->set_int, i)) continue;
        void       *dst = (char *)cfg + ti[i].offset;
        const void *src = (const char *)&args->overrides + ti[i].offset;
        switch (ti[i].type) {
        case DL_F_U8:  *(uint8_t  *)dst = *(const uint8_t  *)src; break;
        case DL_F_I8:  *(int8_t   *)dst = *(const int8_t   *)src; break;
        case DL_F_U16: *(uint16_t *)dst = *(const uint16_t *)src; break;
        case DL_F_U32: *(uint32_t *)dst = *(const uint32_t *)src; break;
        }
    }
    const dl_bool_field_t *tb = dl_config_bool_fields(&n);
    for (size_t i = 0; i < n; i++) {
        if (!bit_test(args->set_bool, i)) continue;
        bool       *dst = (bool *)((char *)cfg + tb[i].offset);
        const bool *src = (const bool *)((const char *)&args->overrides + tb[i].offset);
        *dst = *src;
    }
    const dl_str_field_t  *ts = dl_config_str_fields (&n);
    for (size_t i = 0; i < n; i++) {
        if (!bit_test(args->set_str, i)) continue;
        char       *dst = (char *)cfg + ts[i].offset;
        const char *src = (const char *)&args->overrides + ts[i].offset;
        snprintf(dst, DL_CONF_MAX_STR, "%s", src);
    }
}
```

- [ ] **Step 2: Wire the new pieces into `main`**

In `drone/src/dl_applier.c`'s `main()`, replace the early section (currently around lines 208–222) that reads:

```c
int main(int argc, char **argv) {
    cli_args_t args;
    int pa = parse_args(argc, argv, &args);
    if (pa != 0) return pa < 0 ? 2 : 0;

    dl_log_init("dl-applier", args.log_debug ? DL_LOG_DEBUG : DL_LOG_INFO);
    dl_log_info("dynamic-link applier starting");

    dl_config_t cfg;
    dl_config_defaults(&cfg);
    if (dl_config_load(args.config_path, &cfg) < 0) {
        dl_log_fatal("config load failed");
        return 3;
    }

    dl_dbg_init(&cfg);
    ...
```

with:

```c
int main(int argc, char **argv) {
    cli_args_t args;
    int pa = parse_args(argc, argv, &args);
    if (pa != 0) return pa < 0 ? 2 : 0;

    dl_log_init("dl-applier", args.log_debug ? DL_LOG_DEBUG : DL_LOG_INFO);
    dl_log_info("dynamic-link applier starting");

    dl_config_t cfg;
    dl_config_defaults(&cfg);
    if (args.config_path && dl_config_load(args.config_path, &cfg) < 0) {
        dl_log_fatal("config load failed");
        return 3;
    }
    apply_cli_overrides(&cfg, &args);
    if (dl_config_validate(&cfg) != 0) {
        dl_log_fatal("config validation failed");
        return 3;
    }

    dl_dbg_init(&cfg);
    ...
```

- [ ] **Step 3: Build**

Run: `make -C drone`
Expected: clean build, no warnings.

- [ ] **Step 4: Quick smoke (defaults only)**

Run: `drone/build/dl-applier --help 2>&1 | head -30`
Expected: usage text listing `--config`, `--debug`, `--help`, then sections for integer / boolean / string fields. Phase-3 keys (`debug-enable`, `dbg-log-enable`, `dbg-log-dir`, `dbg-max-bytes`, `dbg-fsync-each`) must NOT appear anywhere in the output.

Run: `drone/build/dl-applier --safe-mcs 99 --config conf/drone.conf.sample 2>&1 | head -5`
Expected: error line like `--safe-mcs: bad value 99 (expected u8 in [0..7])`, exit 2.

- [ ] **Step 5: Full test sweep**

Run: `make -C drone test`
Expected: green.

Run: `python3 -m pytest tests/ --ignore=tests/test_mavlink_status.py`
Expected: green. The existing e2e tests still pass `--config conf-path`; the option survives unchanged.

- [ ] **Step 6: Commit**

```bash
git add drone/src/dl_applier.c
git commit -m "feat(dl-applier): per-field CLI overrides for every in-scope conf field

Build getopt_long's option table dynamically from the dl_config
field accessors. CLI values override conf-file values, which
override built-in defaults. --config becomes optional (defaults +
CLI alone are usable). --help enumerates every flag with its type
and range.

Phase-3 debug-suite fields (debug_enable, dbg_log_enable,
dbg_log_dir, dbg_max_bytes, dbg_fsync_each) remain conf-only as
designed.
"
```

---

## Task 7: Python e2e tests for CLI overrides

**Files:**
- Modify: `tests/test_drone_e2e.py` — `_sandbox` gains an optional `cli_args` kwarg that's threaded into the `dl-applier` invocation.
- Create: `tests/test_drone_cli_overrides.py` — new test file.

`_sandbox` (in `tests/test_drone_e2e.py:301`) currently spawns `dl-applier --config <path> --debug` with no way to append extra CLI flags. We add a single passthrough kwarg `cli_args: list[str] | None` to the context manager so the new tests can stamp CLI overrides on top of the sandbox conf. No other behavior changes.

- [ ] **Step 1: Extend `_sandbox` with `cli_args` passthrough**

Modify `tests/test_drone_e2e.py`. Update the signature at line 301:

```python
@contextlib.contextmanager
def _sandbox(tmp_path: Path, *, extra_drone_conf: dict | None = None,
             cli_args: list[str] | None = None,
             **overrides):
```

In the body of `_sandbox`, find the spawn (around line 374):

```python
proc = subprocess.Popen(
    [str(APPLIER), "--config", str(cfg), "--debug"],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
)
```

Replace with:

```python
spawn_argv = [str(APPLIER), "--config", str(cfg), "--debug"]
if cli_args:
    spawn_argv.extend(cli_args)
proc = subprocess.Popen(
    spawn_argv,
    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
)
```

Also update `_Sandbox.restart_drone_applier` so a respawn uses the same passthrough. Add the field to `_Sandbox.__init__` and the restart:

```python
def __init__(self, state: dict, gs_tunnel_sock: socket.socket,
             cfg_path: Path, cfg_dict: dict,
             cli_args: list[str] | None = None):
    self._state = state
    self._gs_tunnel_sock = gs_tunnel_sock
    self._cfg_path = cfg_path
    self._cfg_dict = cfg_dict
    self._cli_args = list(cli_args or [])
```

In `restart_drone_applier`, change the `subprocess.Popen` argv to:

```python
spawn_argv = [str(APPLIER), "--config", str(self._cfg_path), "--debug"]
spawn_argv.extend(self._cli_args)
new_proc = subprocess.Popen(
    spawn_argv,
    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
)
```

And at the `_sandbox` yield (line 409), pass `cli_args` through:

```python
yield _Sandbox(state, gs_tunnel_sock, cfg, defaults, cli_args=cli_args)
```

- [ ] **Step 2: Run the existing e2e tests to confirm zero regression**

Run: `python3 -m pytest tests/test_drone_e2e.py -q`
Expected: all existing tests pass — `_sandbox` callers that don't pass `cli_args` are unaffected.

- [ ] **Step 3: Create the new test file**

Create `tests/test_drone_cli_overrides.py`:

```python
"""End-to-end tests for dl-applier per-field CLI overrides.

Layered config: built-in defaults -> --config drone.conf -> CLI flags.
These tests exercise the CLI override layer end-to-end against the
real dl-applier binary, the mock wfb_tx control sink, and the rest
of the _sandbox plumbing from test_drone_e2e.
"""
from __future__ import annotations

import socket
import subprocess
import time
from pathlib import Path

import pytest

from tests.test_drone_e2e import (
    APPLIER,
    CMD_SET_RADIO,
    _sandbox,
    build_drone,  # noqa: F401 — autouse session fixture
)


def _wait_for_safe_radio(wfb, timeout: float = 10.0) -> dict:
    """Block until the mock wfb_tx sees a CMD_SET_RADIO from the
    watchdog's safe_defaults push. Returns the decoded request dict."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for m in wfb.received:
            if m.get("cmd") == CMD_SET_RADIO:
                return m
        time.sleep(0.05)
    raise TimeoutError(
        f"no safe_defaults CMD_SET_RADIO in {timeout}s; got {wfb.received!r}"
    )


# --------------------------------------------------------------------
# --help surface: in-scope flags listed, Phase-3 debug-suite excluded.
# --------------------------------------------------------------------

def test_help_lists_in_scope_fields():
    out = subprocess.run(
        [str(APPLIER), "--help"],
        capture_output=True, text=True, timeout=5,
    )
    # parse_args returns 1 on --help, which main maps to exit 0.
    # We don't assert on returncode here; only on the printed text.
    text = out.stderr + out.stdout

    for flag in (
        "--safe-mcs", "--safe-bandwidth", "--safe-tx-power-dBm",
        "--mavlink-port", "--mavlink-enable", "--osd-debug-latency",
        "--encoder-host", "--gs-tunnel-port", "--hello-keepalive-ms",
        "--roi-qp-threshold-kbps", "--listen-port",
    ):
        assert flag in text, f"expected {flag} in --help output"


def test_help_excludes_phase3_debug_suite():
    out = subprocess.run(
        [str(APPLIER), "--help"],
        capture_output=True, text=True, timeout=5,
    )
    text = out.stderr + out.stdout
    for flag in (
        "--debug-enable", "--dbg-log-enable", "--dbg-log-dir",
        "--dbg-max-bytes", "--dbg-fsync-each",
    ):
        assert flag not in text, f"{flag} should NOT be in --help output"


# --------------------------------------------------------------------
# Validation: out-of-range and unknown flags exit non-zero.
# --------------------------------------------------------------------

def test_bad_value_exits_nonzero():
    """safe-mcs is u8 in [0..7]; 99 must be rejected."""
    out = subprocess.run(
        [str(APPLIER), "--safe-mcs", "99"],
        capture_output=True, text=True, timeout=5,
    )
    assert out.returncode != 0
    assert "safe-mcs" in (out.stderr + out.stdout).lower()


def test_unknown_flag_exits_nonzero():
    """Phase-3 debug-suite flags are intentionally unknown to the
    CLI parser; passing one must exit non-zero."""
    out = subprocess.run(
        [str(APPLIER), "--debug-enable"],
        capture_output=True, text=True, timeout=5,
    )
    assert out.returncode != 0


# --------------------------------------------------------------------
# End-to-end: CLI override beats conf-file value.
# --------------------------------------------------------------------

def test_cli_safe_mcs_overrides_conf(tmp_path):
    """The sandbox conf sets safe_mcs=1. We override it to 4 on the
    CLI, let the watchdog trip (health_timeout_ms is 2000 in the
    sandbox defaults), and verify the safe-push uses MCS=4."""
    with _sandbox(tmp_path, cli_args=["--safe-mcs", "4"]) as s:
        radio = _wait_for_safe_radio(s["wfb"])
        assert radio["mcs"] == 4, (
            f"expected MCS=4 from CLI override, got {radio['mcs']}; "
            f"all wfb_tx requests: {s['wfb'].received!r}"
        )


def test_cli_safe_bitrate_overrides_conf(tmp_path):
    """Same idea for safe_bitrate_kbps to confirm the override is
    field-specific (not just safe_mcs)."""
    with _sandbox(tmp_path,
                  cli_args=["--safe-bitrate-kbps", "3000"]) as s:
        # Wait for the safe push so we know the watchdog tripped, then
        # check the encoder mock saw bitrate=3000 in the resulting POST.
        _wait_for_safe_radio(s["wfb"])
        # The encoder mock's recorded requests live on s["encoder"].
        # The applier hits /api/v1/set on majestic; look for the
        # bitrate path argument. Wait briefly to let the encoder POST
        # arrive after the radio command.
        deadline = time.monotonic() + 2.0
        bitrate_seen = None
        while time.monotonic() < deadline and bitrate_seen is None:
            for req in getattr(s["encoder"], "requests", []):
                path = req.get("path", "")
                if "video0.bitrate" in path or "bitrate" in path:
                    # Path format: /api/v1/set?video0.bitrate=3000
                    if "=3000" in path or "3000" in path:
                        bitrate_seen = 3000
                        break
            time.sleep(0.05)
        assert bitrate_seen == 3000, (
            f"expected encoder to be set to bitrate=3000; saw "
            f"{getattr(s['encoder'], 'requests', None)!r}"
        )


# --------------------------------------------------------------------
# End-to-end: --config is optional. Boot from defaults + CLI alone.
# --------------------------------------------------------------------

def _wait_for_bind(addr: str, port: int, timeout: float = 3.0) -> bool:
    """Return True once `addr:port` is bound (a fresh bind to it
    fails with EADDRINUSE)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind((addr, port))
            probe.close()
            time.sleep(0.05)
        except OSError:
            probe.close()
            return True
    return False


def test_dl_applier_boots_without_config():
    """No --config flag. CLI provides a free listen-port and disables
    the IDR socket; everything else uses built-in defaults. We only
    verify the applier binds its listen socket — defaults like
    wlan_dev=wlan0, encoder_host=127.0.0.1:80 will fail downstream,
    but those failures are non-fatal at boot."""
    # Reserve a free UDP port.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()

    proc = subprocess.Popen(
        [str(APPLIER),
         "--listen-addr", "127.0.0.1",
         "--listen-port", str(port),
         "--idr-listen-port", "0",
         "--mavlink-enable",   # set to true; default is also true, no-op
         "--encoder-port", "1",   # connect-refused fast on encoder
         "--debug"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    try:
        assert _wait_for_bind("127.0.0.1", port, timeout=3.0), (
            "dl-applier did not bind --listen-port without --config"
        )
        # Verify the process is still alive (didn't crash on boot).
        assert proc.poll() is None, (
            f"dl-applier exited early: rc={proc.poll()}, "
            f"stderr={proc.stderr.read(1024)!r}"
        )
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
```

- [ ] **Step 4: Run the new tests**

Run: `python3 -m pytest tests/test_drone_cli_overrides.py -v`
Expected: all six tests pass.

If `test_cli_safe_bitrate_overrides_conf` fails because the encoder mock's recorded-requests attribute is named differently (e.g. `s["encoder"].log` instead of `.requests`), grep `tests/test_drone_e2e.py` for `class.*Encoder` and update the attribute access. The test's intent is to confirm the bitrate POST reflects the CLI override.

- [ ] **Step 5: Run the full Python suite**

Run: `python3 -m pytest tests/ --ignore=tests/test_mavlink_status.py -q`
Expected: green.

- [ ] **Step 6: Commit**

```bash
git add tests/test_drone_e2e.py tests/test_drone_cli_overrides.py
git commit -m "test: dl-applier per-field CLI overrides (e2e)

Cover --help surface, range rejection, unknown-flag rejection,
CLI override beating conf-file value (safe_mcs, safe_bitrate_kbps),
and no-config boot.

Extends _sandbox with a cli_args= passthrough kwarg so callers can
stamp CLI overrides on top of the generated conf. No behavior
change for existing callers.
"
```

---

## Task 8: Update README and conf sample comment

**Files:**
- Modify: `README.md`
- Modify: `conf/drone.conf.sample`

- [ ] **Step 1: Add a CLI-overrides subsection to README**

Find the README section that documents how to run `dl-applier` (search for `dl-applier --config` in `README.md`). Append the following subsection beneath it:

```markdown
### CLI overrides

`dl-applier` accepts a `--<kebab-case>` flag for every in-scope
field in `drone.conf`. Precedence is **built-in defaults → conf
file → CLI overrides**, so a CLI flag wins over the file. Listing:

    dl-applier --help

The conf file is optional — `dl-applier --safe-mcs 3` boots from
built-in defaults with `safe_mcs=3`. With both layers:

    dl-applier --config /etc/dynamic-link/drone.conf \
               --safe-mcs 3 --mavlink-port 14570

Boolean fields are on-only switches (`--mavlink-enable` flips it
to `true`; there is no `--no-mavlink-enable`). To force a
default-true field off, set it in the conf file.

The Phase-3 debug-suite fields (`debug_enable`, `dbg_log_enable`,
`dbg_log_dir`, `dbg_max_bytes`, `dbg_fsync_each`) intentionally
have no CLI flag — they live in the conf file only.
```

- [ ] **Step 2: Add a one-line pointer at the top of the sample conf**

Modify `conf/drone.conf.sample`. In the header comment block (currently lines 1–4), append a line:

```
# Every key below can also be set on the CLI as --<kebab-case>;
# run `dl-applier --help` for the full list. CLI overrides win.
```

- [ ] **Step 3: Commit**

```bash
git add README.md conf/drone.conf.sample
git commit -m "docs: dl-applier per-field CLI overrides

Document the new precedence (defaults -> conf -> CLI), the
optional --config, and the on-only boolean caveat.
"
```

---

## Final verification

- [ ] **Step 1: Full C test suite**

Run: `make -C drone clean && make -C drone test`
Expected: every `DL_TEST` passes; no FAIL lines.

- [ ] **Step 2: Full Python test suite**

Run: `python3 -m pytest tests/ --ignore=tests/test_mavlink_status.py -q`
Expected: all green.

- [ ] **Step 3: Manual --help spot check**

Run: `drone/build/dl-applier --help 2>&1`

Verify:
- `--config`, `--debug`, `--help` listed at the top.
- Integer fields section lists `--safe-mcs (u8, range [0..7])` and similar lines.
- Boolean section lists `--mavlink-enable`, `--osd-enable`, `--osd-debug-latency`, `--interleaving-supported`.
- String section lists `--encoder-host`, `--listen-addr`, `--hello-wfb-yaml-path`, etc.
- No `--debug-enable`, `--dbg-log-enable`, `--dbg-log-dir`, `--dbg-max-bytes`, `--dbg-fsync-each` anywhere.

- [ ] **Step 4: Manual override smoke test**

Run: `drone/build/dl-applier --safe-mcs 99 2>&1 | head -3`
Expected: an error message naming `safe-mcs` and the range, exit 2.

Run: `drone/build/dl-applier --safe-mcs 3 --config /dev/null --debug 2>&1 | head -10`
Expected: applier starts (it will fail later on socket binding but that's fine — we're checking arg parsing).

---

## Out of scope (for reference)

The following were considered and explicitly excluded from this plan:

- Env-var layer (`DRONE_*` ↔ conf key mapping).
- Alternative config formats (JSON / TOML / YAML).
- Renaming `--debug` or moving it into the conf.
- Touching `dl-inject` (it has its own CLI surface and a different lifetime).
- Phase-3 debug-suite CLI exposure (`debug_enable`, `dbg_log_enable`, `dbg_log_dir`, `dbg_max_bytes`, `dbg_fsync_each`) — staying conf-only as designed.
