// Temporary standalone test for the unix/NuttX serial backend, exercising
// only the raw open/read/write/close primitives (bypassing the
// dial<->listen INIT/ACK handshake, which requires a real peer that
// implements the ACK side -- e.g. a real zenohd router, not another
// zenoh-pico client dialing the same way).
//
// Not part of the permanent test suite -- for local verification only.

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "zenoh-pico/system/link/serial.h"
#include "zenoh-pico/system/platform.h"

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <dev-a> <dev-b>\n", argv[0]);
        return 1;
    }

    _z_sys_net_socket_t sock_a, sock_b;

    // Open both ends directly via the low-level primitive, skipping
    // _z_open_serial_from_dev's call into _z_connect_serial (the
    // INIT/ACK handshake) since that needs a real listener peer.
    sock_a._fd = open(argv[1], O_RDWR | O_NOCTTY);
    sock_b._fd = open(argv[2], O_RDWR | O_NOCTTY);

    if (sock_a._fd < 0 || sock_b._fd < 0) {
        fprintf(stderr, "FAIL: could not open test devices\n");
        return 1;
    }

    struct termios cfg;
    for (int i = 0; i < 2; i++) {
        _z_sys_net_socket_t *s = i == 0 ? &sock_a : &sock_b;
        tcgetattr(s->_fd, &cfg);
        cfmakeraw(&cfg);
        cfsetispeed(&cfg, B115200);
        cfsetospeed(&cfg, B115200);
        cfg.c_cc[VMIN] = 1;
        cfg.c_cc[VTIME] = 0;
        tcsetattr(s->_fd, TCSANOW, &cfg);
        tcflush(s->_fd, TCIOFLUSH);
    }

    const char *payload = "hello-over-real-posix-serial-link";
    size_t payload_len = strlen(payload);

    // Real call into MY code: _z_send_serial_internal / _z_read_serial_internal,
    // the two functions that actually do the COBS-framed I/O over the fd.
    size_t sent = _z_send_serial_internal(sock_a, 0x00, (const uint8_t *)payload, payload_len);
    if (sent != payload_len) {
        fprintf(stderr, "FAIL: _z_send_serial_internal returned %zu, expected %zu\n", sent, payload_len);
        return 1;
    }

    uint8_t header;
    uint8_t recv_buf[128] = {0};
    size_t recv_len = _z_read_serial_internal(sock_b, &header, recv_buf, sizeof(recv_buf));

    if (recv_len == SIZE_MAX) {
        fprintf(stderr, "FAIL: _z_read_serial_internal returned SIZE_MAX\n");
        return 1;
    }

    if (recv_len != payload_len || memcmp(recv_buf, payload, payload_len) != 0) {
        fprintf(stderr, "FAIL: round-trip mismatch (recv_len=%zu): '%.*s'\n", recv_len, (int)recv_len,
                (char *)recv_buf);
        return 1;
    }

    printf("PASS: sent %zu bytes, received %zu bytes, content matches: '%.*s'\n", sent, recv_len, (int)recv_len,
           (char *)recv_buf);

    _z_close_serial(&sock_a);
    _z_close_serial(&sock_b);

    return 0;
}
