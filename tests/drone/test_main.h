/* test_main.h — tiny assert-based test registry. */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DL_TEST_MAX 128

typedef void (*dl_test_fn)(void);

typedef struct {
    const char *name;
    dl_test_fn  fn;
} dl_test_entry_t;

extern dl_test_entry_t dl_tests[DL_TEST_MAX];
extern int              dl_test_count;
extern int              dl_test_failed;

/* Register a test. Call via DL_TEST(fn). */
void dl_test_register(const char *name, dl_test_fn fn);

#define DL_TEST(fn) \
    static void fn(void); \
    __attribute__((constructor)) static void register_##fn(void) { \
        dl_test_register(#fn, fn); \
    } \
    static void fn(void)

/* Fail the current test (continues the suite). */
#define DL_FAIL(fmt, ...) do {                                              \
    fprintf(stderr, "  FAIL %s:%d: " fmt "\n", __FILE__, __LINE__,          \
            ##__VA_ARGS__);                                                 \
    dl_test_failed++;                                                       \
    return;                                                                 \
} while (0)

#define DL_ASSERT(cond) do {                                                \
    if (!(cond)) DL_FAIL("assertion failed: %s", #cond);                    \
} while (0)

#define DL_ASSERT_EQ(a, b) do {                                             \
    long long _a = (long long)(a);                                          \
    long long _b = (long long)(b);                                          \
    if (_a != _b) DL_FAIL("expected %s == %s (got %lld vs %lld)",           \
                          #a, #b, _a, _b);                                  \
} while (0)

#define DL_ASSERT_STR_EQ(a, b) do {                                         \
    const char *_a = (a);                                                   \
    const char *_b = (b);                                                   \
    if (strcmp(_a, _b) != 0)                                                \
        DL_FAIL("expected \"%s\" == \"%s\"", _a, _b);                       \
} while (0)
