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
