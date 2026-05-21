/* dl_idr_listen.c */
#include "dl_idr_listen.h"
#include "dl_log.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

struct dl_idr_listen {
    int fd;
};

dl_idr_listen_t *dl_idr_listen_open(const char *bind_addr, uint16_t port) {
    if (port == 0) return NULL;

    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        dl_log_err("idr_listen: socket: %s", strerror(errno));
        return NULL;
    }
    int one = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in sa = {0};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port);
    if (!bind_addr || !*bind_addr) {
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, bind_addr, &sa.sin_addr) != 1) {
        dl_log_err("idr_listen: bad bind_addr '%s'", bind_addr);
        close(fd);
        return NULL;
    }
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        dl_log_err("idr_listen: bind %s:%u: %s",
                   bind_addr && *bind_addr ? bind_addr : "0.0.0.0",
                   port, strerror(errno));
        close(fd);
        return NULL;
    }

    dl_idr_listen_t *l = calloc(1, sizeof(*l));
    if (!l) { close(fd); return NULL; }
    l->fd = fd;
    dl_log_info("idr_listen: bound %s:%u",
                bind_addr && *bind_addr ? bind_addr : "0.0.0.0", port);
    return l;
}

int dl_idr_listen_fd(const dl_idr_listen_t *l) {
    return l ? l->fd : -1;
}

size_t dl_idr_listen_drain(dl_idr_listen_t *l) {
    if (!l) return 0;
    size_t count = 0;
    uint8_t buf[64];
    while (1) {
        ssize_t n = recvfrom(l->fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            dl_log_debug("idr_listen: recvfrom: %s", strerror(errno));
            break;
        }
        count++;
    }
    return count;
}

void dl_idr_listen_close(dl_idr_listen_t *l) {
    if (!l) return;
    if (l->fd >= 0) close(l->fd);
    free(l);
}
