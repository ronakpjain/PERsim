/**
 * @file vcan_bridge.c
 * @brief Linux SocketCAN to PERsim PCAN/TCP gateway for the Yoctocoupler guest.
 * @details One TCP connection multiplexes up to three explicitly mapped classic
 * CAN interfaces. SocketCAN already supplies the kernel interface; no custom
 * kernel driver is involved. See docs/cm5.md for the bridge protocol.
 */
/** @brief Request POSIX.1-2008 libc interfaces. */
#define _POSIX_C_SOURCE 200809L
/** @brief Retain Linux interface-name helpers in glibc feature selection. */
#define _DEFAULT_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

/** @brief A bounded outgoing CAN queue for one guest interface. */
struct mapping {
    int fd;                         /**< Bound PF_CAN socket. */
    unsigned bus;                   /**< PER bus ID: 0, 1, or 3. */
    unsigned ifindex;               /**< Unique Linux CAN interface index. */
    struct can_frame pending[256];  /**< TCP-to-CAN queue, no silent dropping. */
    size_t count;                   /**< Number of queued CAN frames. */
};
/** @brief Report an external error and terminate; systemd may restart the client. */
static void fail(const char *what) { perror(what); exit(1); }
/** @brief Parse an unsigned numeric argument without accepting suffixes. */
static unsigned number(const char *text, unsigned max) {
    char *end = NULL;
    errno = 0;
    unsigned long n = strtoul(text, &end, 10);
    if (errno || !text[0] || *end || n > max) {
        fprintf(stderr, "Invalid numeric argument: %s\n", text); exit(2);
    }
    return (unsigned)n;
}
/** @brief Enable nonblocking I/O; the poll loop owns all backpressure. */
static void nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) fail("fcntl");
}
/** @brief Bind an existing Linux CAN interface, never creating or configuring it. */
static struct mapping open_can(const char *spec) {
    char name[IFNAMSIZ];
    const char *colon = strchr(spec, ':');
    if (!colon || colon == spec || (size_t)(colon - spec) >= sizeof(name)) {
        fprintf(stderr, "Expected INTERFACE:BUS, e.g. vcan0:0\n"); exit(2);
    }
    memcpy(name, spec, (size_t)(colon - spec)); name[colon - spec] = 0;
    unsigned bus = number(colon + 1, 3);
    if (bus == 2) { fprintf(stderr, "Bus 2 is CCAN, not supported by this bridge\n"); exit(2); }
    unsigned index = if_nametoindex(name);
    if (!index) fail("if_nametoindex (bring up vcan first)");
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) fail("socket CAN_RAW");
    struct sockaddr_can address = {.can_family = AF_CAN, .can_ifindex = (int)index};
    if (bind(fd, (struct sockaddr *)&address, sizeof(address)) < 0) fail("bind CAN");
    nonblocking(fd);
    fprintf(stderr, "%s <-> PER bus %u\n", name, bus);
    struct mapping result = {.fd = fd, .bus = bus, .ifindex = index, .count = 0};
    return result;
}
/** @brief Encode one standard data frame into the explicit 20-byte PCAN format. */
static void encode(uint8_t out[20], unsigned bus, const struct can_frame *frame) {
    memset(out, 0, 20); memcpy(out, "PCAN", 4);
    out[4] = 1; out[5] = (uint8_t)bus; out[6] = frame->can_dlc;
    for (int i = 0; i < 4; ++i) out[8 + i] = (uint8_t)(frame->can_id >> (8 * i));
    memcpy(out + 12, frame->data, frame->can_dlc);
}
/** @brief Validate a host record and queue it only on its explicitly mapped bus. */
static void route(const uint8_t in[20], struct mapping *maps, int count) {
    uint32_t id = 0;
    for (int i = 0; i < 4; ++i) id |= (uint32_t)in[8 + i] << (8 * i);
    if (memcmp(in, "PCAN", 4) || in[4] != 1 || in[7] || in[6] > 8 || id > CAN_SFF_MASK
        || (in[5] != 0 && in[5] != 1 && in[5] != 3)) {
        fprintf(stderr, "Invalid PCAN record; disconnecting\n"); exit(1);
    }
    for (int i = 0; i < count; ++i) if (maps[i].bus == in[5]) {
        struct mapping *m = &maps[i];
        if (m->count == 256) { fprintf(stderr, "CAN output backpressure; disconnecting\n"); exit(1); }
        struct can_frame *f = &m->pending[m->count++];
        memset(f, 0, sizeof(*f)); f->can_id = id; f->can_dlc = in[6];
        memcpy(f->data, in + 12, f->can_dlc);
    }
}
/** @brief Run a guest client: HOST PORT [vcan0:0 [vcan1:1 [vcan3:3]]]. */
int main(int argc, char **argv) {
    if (argc < 3 || argc > 6) {
        fprintf(stderr, "Usage: %s HOST_IPV4 PORT [INTERFACE:BUS ...]\n", argv[0]); return 2;
    }
    signal(SIGPIPE, SIG_IGN);
    unsigned port = number(argv[2], 65535);
    if (!port) { fprintf(stderr, "Port must be nonzero\n"); return 2; }
    struct mapping maps[3];
    int count = argc == 3 ? 1 : argc - 3;
    for (int i = 0; i < count; ++i) {
        maps[i] = open_can(argc == 3 ? "vcan0:0" : argv[3 + i]);
        for (int j = 0; j < i; ++j) if (maps[j].bus == maps[i].bus || maps[j].ifindex == maps[i].ifindex) {
            fprintf(stderr, "Duplicate bus or interface mapping\n"); return 2;
        }
    }
    int tcp = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp < 0) fail("socket TCP");
    struct sockaddr_in host = {.sin_family = AF_INET, .sin_port = htons((uint16_t)port)};
    if (inet_pton(AF_INET, argv[1], &host.sin_addr) != 1) {
        fprintf(stderr, "Expected numeric IPv4 host address\n"); return 2;
    }
    if (connect(tcp, (struct sockaddr *)&host, sizeof(host)) < 0) fail("connect PERsim");
    nonblocking(tcp);
    const int yes = 1;
    if (setsockopt(tcp, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)) < 0) fail("TCP_NODELAY");
    uint8_t tx[65536], rx[65536];
    size_t tx_size = 0, rx_size = 0;
    fprintf(stderr, "Connected to PERsim; classic standard CAN only, no VM lockstep\n");
    for (;;) {
        struct pollfd fds[4];
        fds[0] = (struct pollfd){.fd = tcp, .events = POLLIN | (tx_size ? POLLOUT : 0)};
        for (int i = 0; i < count; ++i)
            fds[i + 1] = (struct pollfd){.fd = maps[i].fd, .events = POLLIN | (maps[i].count ? POLLOUT : 0)};
        int ready = poll(fds, (nfds_t)(count + 1), -1);
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0) fail("poll");
        for (int i = 0; i < count + 1; ++i) if ((fds[i].revents & (POLLERR | POLLNVAL)) || (i > 0 && (fds[i].revents & POLLHUP))) {
            fprintf(stderr, "Socket error; disconnecting\n"); return 1;
        }
        if (fds[0].revents & POLLOUT) {
            ssize_t n = send(tcp, tx, tx_size, MSG_NOSIGNAL);
            if (n > 0) { memmove(tx, tx + n, tx_size - (size_t)n); tx_size -= (size_t)n; }
            else if (n < 0 && errno != EAGAIN && errno != EINTR) fail("send TCP");
        }
        if (fds[0].revents & (POLLIN | POLLHUP)) {
            ssize_t n = read(tcp, rx + rx_size, sizeof(rx) - rx_size);
            if (n == 0) {
                fprintf(stderr, "PERsim disconnected%s\n", rx_size ? " (truncated record)" : "");
                return rx_size ? 1 : 0;
            }
            if (n < 0 && errno != EAGAIN && errno != EINTR) fail("read TCP");
            if (n > 0) rx_size += (size_t)n;
            size_t used = 0;
            while (rx_size - used >= 20) { route(rx + used, maps, count); used += 20; }
            memmove(rx, rx + used, rx_size - used); rx_size -= used;
        }
        for (int i = 0; i < count; ++i) {
            struct mapping *m = &maps[i];
            if (fds[i + 1].revents & POLLOUT) {
                ssize_t n = write(m->fd, &m->pending[0], CAN_MTU);
                if (n == CAN_MTU) { memmove(m->pending, m->pending + 1, (--m->count) * sizeof(m->pending[0])); }
                else if (n < 0 && errno != EAGAIN && errno != ENOBUFS && errno != EINTR) fail("write CAN");
            }
            if (fds[i + 1].revents & POLLIN) {
                struct can_frame f;
                ssize_t n = read(m->fd, &f, CAN_MTU);
                if (n < 0 && (errno == EAGAIN || errno == EINTR)) continue;
                if (n != CAN_MTU) fail("read CAN");
                if ((f.can_id & ~CAN_SFF_MASK) || f.can_dlc > 8) {
                    fprintf(stderr, "Unsupported extended/RTR/error frame on guest CAN\n"); return 1;
                }
                if (tx_size + 20 > sizeof(tx)) { fprintf(stderr, "TCP output backpressure; disconnecting\n"); return 1; }
                encode(tx + tx_size, m->bus, &f); tx_size += 20;
            }
        }
    }
}
