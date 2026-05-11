/* dl_yaml_get.c — see dl_yaml_get.h. */
#include "dl_yaml_get.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
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
