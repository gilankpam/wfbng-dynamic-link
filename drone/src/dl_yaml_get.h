/* dl_yaml_get.h — tiny line-scan helper for finding `<key>: <integer>`
 * inside a top-level `<block>:` section. Robust enough for the two
 * specific YAML shapes we read (/etc/wfb.yaml, /etc/majestic.yaml).
 * Not a general YAML parser. */
#pragma once

/* Find `key: <int>` indented under top-level block `block:` in `path`.
 * On success, writes the parsed integer into *out and returns 0.
 * On failure returns a negative errno-ish code: negative errno on
 * open failure (typically -ENOENT), -EINVAL for any structural /
 * parsing problem (block not found, key not found inside block,
 * non-integer value, etc.).
 * `*out` is only written on success. */
int dl_yaml_get_int(const char *path,
                    const char *block,
                    const char *key,
                    int *out);
