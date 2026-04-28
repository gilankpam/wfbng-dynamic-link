/* dl_dbg.c — see dl_dbg.h. */
#include "dl_dbg.h"
#include "dl_log.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define DL_DBG_DETAIL_MAX 1024
#define DL_DBG_LINE_MAX   2048

static struct {
    FILE        *fp;
    char         path[DL_CONF_MAX_STR + 32];
    uint32_t     max_bytes;
    bool         fsync_each;
    uint32_t     seq;
} g;

static const char *sev_str(dl_dbg_sev_t s) {
    switch (s) {
        case DL_DBG_SEV_DEBUG: return "debug";
        case DL_DBG_SEV_INFO:  return "info";
        case DL_DBG_SEV_WARN:  return "warn";
        case DL_DBG_SEV_ERROR: return "error";
    }
    return "info";
}

static uint64_t mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

/* Best-effort: try to open the active path. On failure, leave g.fp NULL
 * and return -1 — dl_dbg becomes a no-op. */
static int open_active(void) {
    g.fp = fopen(g.path, "a");
    if (!g.fp) return -1;
    /* Line-buffered so each emit hits the kernel without fsync(). */
    setvbuf(g.fp, NULL, _IOLBF, 0);
    return 0;
}

void dl_dbg_init(const dl_config_t *cfg) {
    memset(&g, 0, sizeof(g));
    if (!dl_config_dbg_log_resolved(cfg)) {
        return;
    }

    /* Make the directory if it doesn't exist (parent must already
     * exist — we're not going to create the SD mountpoint). */
    if (mkdir(cfg->dbg_log_dir, 0755) < 0 && errno != EEXIST) {
        dl_log_warn("dbg: mkdir %s: %s; failure log disabled",
                    cfg->dbg_log_dir, strerror(errno));
        return;
    }

    snprintf(g.path, sizeof(g.path), "%s/dl-events.jsonl",
             cfg->dbg_log_dir);
    g.max_bytes = cfg->dbg_max_bytes;
    g.fsync_each = cfg->dbg_fsync_each;

    if (open_active() < 0) {
        dl_log_warn("dbg: open %s: %s; failure log disabled",
                    g.path, strerror(errno));
        return;
    }

    dl_log_info("dbg: failure log → %s (cap %u bytes%s)",
                g.path, g.max_bytes,
                g.fsync_each ? ", fsync each" : "");
}

void dl_dbg_close(void) {
    if (g.fp) {
        fflush(g.fp);
        fclose(g.fp);
        g.fp = NULL;
    }
}

bool dl_dbg_enabled(void) {
    return g.fp != NULL;
}

static void rotate_if_needed(void) {
    if (!g.fp || g.max_bytes == 0) return;
    long pos = ftell(g.fp);
    if (pos < 0 || (uint32_t)pos < g.max_bytes) return;

    fclose(g.fp);
    g.fp = NULL;

    char prev[sizeof(g.path) + 4];
    char cur[sizeof(g.path) + 4];
    /* Two-deep ring: dl-events.jsonl → dl-events.jsonl.1 →
     * dl-events.jsonl.2 (overwritten on next rotate). */
    snprintf(prev, sizeof(prev), "%s.2", g.path);
    snprintf(cur,  sizeof(cur),  "%s.1", g.path);
    /* Best-effort; ignore errors — we just want to keep going. */
    rename(cur, prev);
    rename(g.path, cur);

    if (open_active() < 0) {
        dl_log_warn("dbg: re-open after rotate failed: %s",
                    strerror(errno));
    }
}

/* Tiny JSON-string escape for the body field. Writes at most
 * out_size bytes including NUL. */
static void json_escape(const char *in, size_t in_len,
                        char *out, size_t out_size) {
    size_t oi = 0;
    for (size_t i = 0; i < in_len && oi + 2 < out_size; ++i) {
        unsigned char c = (unsigned char)in[i];
        if (c == '\\' || c == '"') {
            if (oi + 3 >= out_size) break;
            out[oi++] = '\\'; out[oi++] = (char)c;
        } else if (c == '\n') {
            if (oi + 3 >= out_size) break;
            out[oi++] = '\\'; out[oi++] = 'n';
        } else if (c == '\r') {
            if (oi + 3 >= out_size) break;
            out[oi++] = '\\'; out[oi++] = 'r';
        } else if (c == '\t') {
            if (oi + 3 >= out_size) break;
            out[oi++] = '\\'; out[oi++] = 't';
        } else if (c < 0x20 || c == 0x7F) {
            if (oi + 7 >= out_size) break;
            oi += (size_t)snprintf(out + oi, out_size - oi,
                                   "\\u%04x", (unsigned)c);
        } else {
            out[oi++] = (char)c;
        }
    }
    out[oi] = '\0';
}

void dl_dbg_emit(const char *reason,
                 dl_dbg_sev_t sev,
                 const char *detail_json) {
    if (!g.fp) return;
    if (!detail_json || detail_json[0] == '\0') detail_json = "{}";

    /* Defensive truncation. We don't try to keep the JSON valid here —
     * upstream callers must pass valid JSON in the first place; this
     * cap just stops a runaway detail string from blowing the line
     * budget. */
    char detail[DL_DBG_DETAIL_MAX];
    size_t dlen = strlen(detail_json);
    if (dlen >= sizeof(detail)) dlen = sizeof(detail) - 1;
    memcpy(detail, detail_json, dlen);
    detail[dlen] = '\0';

    char line[DL_DBG_LINE_MAX];
    int n = snprintf(line, sizeof(line),
                     "{\"t\":%llu,\"seq\":%u,\"sev\":\"%s\","
                     "\"reason\":\"%s\",\"detail\":%s}\n",
                     (unsigned long long)mono_us(),
                     (unsigned)g.seq++,
                     sev_str(sev),
                     reason ? reason : "",
                     detail);
    if (n <= 0) return;
    size_t to_write = (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1;
    /* If snprintf truncated, ensure the line still ends with `}\n` so
     * the file stays jq-parsable. Cheap workaround for pathological
     * input — normal traffic never hits this branch. */
    if ((size_t)n >= sizeof(line)) {
        line[sizeof(line) - 3] = '}';
        line[sizeof(line) - 2] = '\n';
        line[sizeof(line) - 1] = '\0';
        to_write = sizeof(line) - 1;
    }
    fwrite(line, 1, to_write, g.fp);
    if (g.fsync_each) {
        fflush(g.fp);
        fsync(fileno(g.fp));
    }
    rotate_if_needed();
}

void dl_dbg_emit_errno(const char *reason, int err) {
    char buf[128];
    char esc[64];
    json_escape(strerror(err), strlen(strerror(err)), esc, sizeof(esc));
    snprintf(buf, sizeof(buf),
             "{\"errno\":%d,\"strerror\":\"%s\"}", err, esc);
    dl_dbg_emit(reason, DL_DBG_SEV_WARN, buf);
}

void dl_dbg_emit_http(const char *reason,
                      dl_dbg_sev_t sev,
                      int http_status,
                      const char *body,
                      size_t body_len) {
    char escbody[256];
    if (body_len > 64) body_len = 64;
    json_escape(body ? body : "", body_len, escbody, sizeof(escbody));

    char buf[512];
    snprintf(buf, sizeof(buf),
             "{\"http\":%d,\"body\":\"%s\"}", http_status, escbody);
    dl_dbg_emit(reason, sev, buf);
}
