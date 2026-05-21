/* test_idr_listen.c — unit tests for dl_idr_listen. */
#include "dl_idr_listen.h"
#include "test_main.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

DL_TEST(test_idr_listen_port_zero_disabled) {
    dl_idr_listen_t *l = dl_idr_listen_open("127.0.0.1", 0);
    DL_ASSERT(l == NULL);
}

DL_TEST(test_idr_listen_bind_drain_three) {
    /* Fixed high port; the test harness is single-threaded so
     * collisions with a real wfb-ng IDR helper on the dev host are
     * vanishingly rare. */
    const uint16_t PORT = 51123;
    dl_idr_listen_t *l = dl_idr_listen_open("127.0.0.1", PORT);
    DL_ASSERT(l != NULL);
    DL_ASSERT(dl_idr_listen_fd(l) >= 0);

    DL_ASSERT_EQ(dl_idr_listen_drain(l), 0);

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    DL_ASSERT(s >= 0);
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(PORT);
    inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
    const char msg[] = "abc\n";
    for (int i = 0; i < 3; i++) {
        ssize_t r = sendto(s, msg, sizeof(msg) - 1, 0,
                           (struct sockaddr *)&dst, sizeof(dst));
        DL_ASSERT_EQ(r, (ssize_t)(sizeof(msg) - 1));
    }
    struct pollfd pf = { .fd = dl_idr_listen_fd(l), .events = POLLIN };
    DL_ASSERT(poll(&pf, 1, 500) > 0);
    DL_ASSERT_EQ(dl_idr_listen_drain(l), 3);
    DL_ASSERT_EQ(dl_idr_listen_drain(l), 0);

    close(s);
    dl_idr_listen_close(l);
}

DL_TEST(test_idr_listen_close_null_safe) {
    dl_idr_listen_close(NULL);  /* must not crash */
    DL_ASSERT(dl_idr_listen_fd(NULL) == -1);
    DL_ASSERT(dl_idr_listen_drain(NULL) == 0);
}
