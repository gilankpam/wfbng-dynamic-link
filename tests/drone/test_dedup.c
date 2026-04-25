/* test_dedup.c — sequence dedup + reset semantics. */
#include "test_main.h"
#include "dl_dedup.h"

DL_TEST(test_dedup_first_packet_always_accepted) {
    dl_dedup_t d;
    dl_dedup_init(&d);
    DL_ASSERT(!dl_dedup_check(&d, 42));    /* fresh seed */
    DL_ASSERT( dl_dedup_check(&d, 42));    /* exact dup */
}

DL_TEST(test_dedup_monotonic_accepted) {
    dl_dedup_t d;
    dl_dedup_init(&d);
    dl_dedup_check(&d, 10);
    DL_ASSERT(!dl_dedup_check(&d, 11));
    DL_ASSERT(!dl_dedup_check(&d, 12));
    DL_ASSERT(!dl_dedup_check(&d, 100));
}

DL_TEST(test_dedup_older_rejected) {
    dl_dedup_t d;
    dl_dedup_init(&d);
    dl_dedup_check(&d, 100);
    DL_ASSERT(dl_dedup_check(&d, 99));
    DL_ASSERT(dl_dedup_check(&d, 50));
    DL_ASSERT(dl_dedup_check(&d, 1));
}

DL_TEST(test_dedup_reset_accepts_lower_seq) {
    /* Real-world trigger: drone learns seq=100200 from an injected
     * test, then GS restarts and emits seq=1884. Without reset, the
     * applier never recovers. */
    dl_dedup_t d;
    dl_dedup_init(&d);
    dl_dedup_check(&d, 100200);
    DL_ASSERT(dl_dedup_check(&d, 1884));   /* would-be stale: rejected */
    dl_dedup_reset(&d);
    DL_ASSERT(!dl_dedup_check(&d, 1884));  /* post-reset: fresh seed */
    DL_ASSERT(!dl_dedup_check(&d, 1885));
    DL_ASSERT( dl_dedup_check(&d, 1884));  /* and dup logic resumes */
}

DL_TEST(test_dedup_wrap_handled_via_signed_delta) {
    /* uint32_t wrap: seq 2^32-5 → seq 10 should be treated as forward
     * progress (signed delta ≈ +15). */
    dl_dedup_t d;
    dl_dedup_init(&d);
    dl_dedup_check(&d, 0xFFFFFFFBu);       /* near-max */
    DL_ASSERT(!dl_dedup_check(&d, 10));    /* delta = +15, accept */
    DL_ASSERT( dl_dedup_check(&d, 0xFFFFFFFBu));  /* now genuinely stale */
}
