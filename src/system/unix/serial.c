//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//
// unix/serial.c
//
// POSIX termios-based serial link for the "unix" system backend. Written for
// NuttX (PX4 flight controllers talking Zenoh over a UART/USB-CDC link to a
// companion computer), but uses nothing beyond plain POSIX termios, so it
// should work unmodified on desktop Linux/macOS/BSD as well.
//
// Only device-path based serial ("serial//dev/ttyS0#baudrate=...") is
// implemented. Pin-based serial (bit-banged GPIO UART, meaningful only on
// bare-metal MCU targets like ESP32/RP2040) and listen/accept-style serial
// are intentionally left unimplemented -- upstream's own most mature
// embedded backend (ESP-IDF) doesn't implement them either, since serial is
// a point-to-point link with no listen/accept concept: both ends simply
// open() their side of the wire.

#include "zenoh-pico/system/link/serial.h"

#include "zenoh-pico/config.h"

#if Z_FEATURE_LINK_SERIAL == 1

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "zenoh-pico/protocol/codec/serial.h"
#include "zenoh-pico/system/common/serial.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/logging.h"
#include "zenoh-pico/utils/result.h"

// Some Bxxx constants beyond the strict POSIX set (B921600 and up) are a
// widely supported Linux/BSD/NuttX extension, not guaranteed present on
// every libc -- fall back to the raw integer value if missing, matching the
// same defensive pattern PX4's own mavlink module uses for the exact same
// UART peripherals (src/modules/mavlink/mavlink_main.cpp).
#ifndef B230400
#define B230400 230400
#endif
#ifndef B460800
#define B460800 460800
#endif
#ifndef B500000
#define B500000 500000
#endif
#ifndef B921600
#define B921600 921600
#endif
#ifndef B1000000
#define B1000000 1000000
#endif

static speed_t _z_serial_baudrate_to_speed(uint32_t baudrate) {
    switch (baudrate) {
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        default:
            // Unrecognised baud rate -- caller logs and fails the open.
            return (speed_t)0xFFFFFFFF;
    }
}

z_result_t _z_open_serial_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    (void)sock;
    (void)txpin;
    (void)rxpin;
    (void)baudrate;
    _Z_ERROR("Pin-based serial (bit-banged GPIO UART) is not supported on the unix/NuttX backend -- "
              "use a device path (serial//dev/ttyS0#baudrate=...) instead");
    return _Z_ERR_GENERIC;
}

z_result_t _z_open_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    speed_t speed = _z_serial_baudrate_to_speed(baudrate);

    if (speed == (speed_t)0xFFFFFFFF) {
        _Z_ERROR("Unsupported serial baud rate: %u", (unsigned int)baudrate);
        return _Z_ERR_GENERIC;
    }

    // O_NOCTTY: this fd must never become the process's controlling
    // terminal -- matches PX4's own mavlink module opening this exact same
    // class of device (FC USB-CDC / UART peripherals).
    int fd = open(dev, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        _Z_ERROR("Failed to open serial device %s: %s", dev, strerror(errno));
        return _Z_ERR_GENERIC;
    }

    struct termios cfg;

    if (tcgetattr(fd, &cfg) < 0) {
        _Z_ERROR("tcgetattr failed on %s: %s", dev, strerror(errno));
        close(fd);
        return _Z_ERR_GENERIC;
    }

    cfmakeraw(&cfg);  // 8N1, no flow control, no line discipline processing

    if (cfsetispeed(&cfg, speed) < 0 || cfsetospeed(&cfg, speed) < 0) {
        _Z_ERROR("Failed to set baud rate %u on %s: %s", (unsigned int)baudrate, dev, strerror(errno));
        close(fd);
        return _Z_ERR_GENERIC;
    }

    // Blocking reads with no minimum byte count / inter-byte timeout: the
    // read-loop in _z_read_serial_internal below reads one byte at a time
    // and relies on the underlying fd blocking until data arrives.
    cfg.c_cc[VMIN] = 1;
    cfg.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &cfg) < 0) {
        _Z_ERROR("tcsetattr failed on %s: %s", dev, strerror(errno));
        close(fd);
        return _Z_ERR_GENERIC;
    }

    tcflush(fd, TCIOFLUSH);

    sock->_fd = fd;

    return _z_connect_serial(*sock);
}

z_result_t _z_listen_serial_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    (void)sock;
    (void)txpin;
    (void)rxpin;
    (void)baudrate;
    _Z_ERROR("Listen-mode serial is not implemented (serial is point-to-point: "
              "both ends open(), neither listens/accepts)");
    return _Z_ERR_GENERIC;
}

z_result_t _z_listen_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    (void)sock;
    (void)dev;
    (void)baudrate;
    _Z_ERROR("Listen-mode serial is not implemented (serial is point-to-point: "
              "both ends open(), neither listens/accepts)");
    return _Z_ERR_GENERIC;
}

void _z_close_serial(_z_sys_net_socket_t *sock) {
    if (sock->_fd >= 0) {
        close(sock->_fd);
        sock->_fd = -1;
    }
}

size_t _z_read_serial_internal(const _z_sys_net_socket_t sock, uint8_t *header, uint8_t *ptr, size_t len) {
    uint8_t *raw_buf = (uint8_t *)z_malloc(_Z_SERIAL_MAX_COBS_BUF_SIZE);

    if (raw_buf == NULL) {
        _Z_ERROR("Failed to allocate serial COBS buffer");
        return SIZE_MAX;
    }

    size_t rb = 0;

    while (rb < _Z_SERIAL_MAX_COBS_BUF_SIZE) {
        ssize_t r = read(sock._fd, &raw_buf[rb], 1);

        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }

            _Z_ERROR("Error reading from serial: %s", strerror(errno));
            z_free(raw_buf);
            return SIZE_MAX;

        } else if (r == 0) {
            // EOF -- peer closed / device gone.
            _Z_ERROR("EOF reading from serial");
            z_free(raw_buf);
            return SIZE_MAX;

        } else {
            rb += (size_t)r;

            if (raw_buf[rb - 1] == (uint8_t)0x00) {
                break;
            }
        }
    }

    uint8_t *tmp_buf = (uint8_t *)z_malloc(_Z_SERIAL_MFS_SIZE);

    if (tmp_buf == NULL) {
        _Z_ERROR("Failed to allocate serial MFS buffer");
        z_free(raw_buf);
        return SIZE_MAX;
    }

    size_t ret = _z_serial_msg_deserialize(raw_buf, rb, ptr, len, header, tmp_buf, _Z_SERIAL_MFS_SIZE);

    z_free(raw_buf);
    z_free(tmp_buf);

    return ret;
}

size_t _z_send_serial_internal(const _z_sys_net_socket_t sock, uint8_t header, const uint8_t *ptr, size_t len) {
    uint8_t *tmp_buf = (uint8_t *)z_malloc(_Z_SERIAL_MFS_SIZE);
    uint8_t *raw_buf = (uint8_t *)z_malloc(_Z_SERIAL_MAX_COBS_BUF_SIZE);

    if ((raw_buf == NULL) || (tmp_buf == NULL)) {
        _Z_ERROR("Failed to allocate serial COBS and/or MFS buffer");
        z_free(raw_buf);
        z_free(tmp_buf);
        return SIZE_MAX;
    }

    size_t ret =
        _z_serial_msg_serialize(raw_buf, _Z_SERIAL_MAX_COBS_BUF_SIZE, ptr, len, header, tmp_buf, _Z_SERIAL_MFS_SIZE);

    if (ret == SIZE_MAX) {
        z_free(raw_buf);
        z_free(tmp_buf);
        return ret;
    }

    size_t written = 0;

    while (written < ret) {
        ssize_t w = write(sock._fd, &raw_buf[written], ret - written);

        if (w < 0) {
            if (errno == EINTR) {
                continue;
            }

            _Z_ERROR("Error writing to serial: %s", strerror(errno));
            z_free(raw_buf);
            z_free(tmp_buf);
            return SIZE_MAX;
        }

        written += (size_t)w;
    }

    z_free(raw_buf);
    z_free(tmp_buf);

    return len;
}

#endif  // Z_FEATURE_LINK_SERIAL == 1
