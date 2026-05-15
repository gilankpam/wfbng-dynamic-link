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
