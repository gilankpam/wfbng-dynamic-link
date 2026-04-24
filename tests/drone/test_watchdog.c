/* test_watchdog.c — failsafe 1 state machine. */
#include "test_main.h"
#include "dl_watchdog.h"

DL_TEST(test_watchdog_quiet_before_first_decision) {
    dl_watchdog_t w;
    dl_watchdog_init(&w, 1000);
    /* No decision yet — ticks never trip. */
    DL_ASSERT(!dl_watchdog_tick(&w, 2000));
    DL_ASSERT(!dl_watchdog_tick(&w, 5000));
    DL_ASSERT(!dl_watchdog_is_tripped(&w));
}

DL_TEST(test_watchdog_fires_after_timeout) {
    dl_watchdog_t w;
    dl_watchdog_init(&w, 1000);
    dl_watchdog_notify_decision(&w, 10000);
    DL_ASSERT(!dl_watchdog_tick(&w, 10500));  /* 500 < 1000 */
    DL_ASSERT(!dl_watchdog_tick(&w, 10999));  /* still inside */
    DL_ASSERT( dl_watchdog_tick(&w, 11000));  /* exactly at timeout */
    DL_ASSERT( dl_watchdog_is_tripped(&w));
}

DL_TEST(test_watchdog_fires_exactly_once_while_silent) {
    dl_watchdog_t w;
    dl_watchdog_init(&w, 500);
    dl_watchdog_notify_decision(&w, 0);
    DL_ASSERT(dl_watchdog_tick(&w, 500));
    /* No more pushes while still silent. */
    DL_ASSERT(!dl_watchdog_tick(&w, 1000));
    DL_ASSERT(!dl_watchdog_tick(&w, 2000));
    DL_ASSERT(!dl_watchdog_tick(&w, 60000));
}

DL_TEST(test_watchdog_reset_clears_latch) {
    dl_watchdog_t w;
    dl_watchdog_init(&w, 500);
    dl_watchdog_notify_decision(&w, 0);
    DL_ASSERT(dl_watchdog_tick(&w, 500));
    /* Fresh decision clears tripped flag. */
    dl_watchdog_notify_decision(&w, 600);
    DL_ASSERT(!dl_watchdog_is_tripped(&w));
    /* And the next silent window will fire again. */
    DL_ASSERT(dl_watchdog_tick(&w, 1200));
}
