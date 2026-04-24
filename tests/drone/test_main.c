/* test_main.c — runs every test_* registered via DL_TEST. */
#include "test_main.h"

#include <stdio.h>

dl_test_entry_t dl_tests[DL_TEST_MAX];
int             dl_test_count = 0;
int             dl_test_failed = 0;

void dl_test_register(const char *name, dl_test_fn fn) {
    if (dl_test_count >= DL_TEST_MAX) {
        fprintf(stderr, "DL_TEST_MAX exceeded\n");
        return;
    }
    dl_tests[dl_test_count].name = name;
    dl_tests[dl_test_count].fn   = fn;
    dl_test_count++;
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    int passed = 0;
    int failed_start;
    for (int i = 0; i < dl_test_count; ++i) {
        failed_start = dl_test_failed;
        printf("RUN  %s\n", dl_tests[i].name);
        dl_tests[i].fn();
        if (dl_test_failed == failed_start) {
            printf("PASS %s\n", dl_tests[i].name);
            passed++;
        } else {
            printf("FAIL %s\n", dl_tests[i].name);
        }
    }
    int ran = dl_test_count;
    int failed = ran - passed;
    printf("\n%d tests, %d passed, %d failed\n", ran, passed, failed);
    return failed == 0 ? 0 : 1;
}
