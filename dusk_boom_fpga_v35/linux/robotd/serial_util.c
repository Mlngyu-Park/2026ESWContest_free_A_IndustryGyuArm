/* serial_util.c — see serial_util.h */
#include "serial_util.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/serial.h>
#include <sys/ioctl.h>
#endif

static speed_t baud_const(int baud)
{
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
#endif
    default: return 0;
    }
}

int serial_open(const char *dev, int baud)
{
    const speed_t sp = baud_const(baud);
    if (sp == 0) {
        errno = EINVAL;
        return -1;
    }
    const int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return -1;
    }
    struct termios t;
    if (tcgetattr(fd, &t) != 0) {
        const int e = errno;
        close(fd);
        errno = e;
        return -1;
    }
    cfmakeraw(&t);
    /* a line held low (ESP unpowered / in reset / IO5 not a UART yet) arrives as a stream of
     * break / framing-error 0x00 bytes (~100k/s at 1 Mbps, seen on the board) - never relay those */
    t.c_iflag |= IGNBRK | IGNPAR;
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    cfsetispeed(&t, sp);
    cfsetospeed(&t, sp);
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        const int e = errno;
        close(fd);
        errno = e;
        return -1;
    }
#ifdef TIOCGSERIAL
    /* usb-serial / xuartps: deliver received bytes without extra buffering delay */
    struct serial_struct ss;
    if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
        ss.flags |= ASYNC_LOW_LATENCY;
        (void)ioctl(fd, TIOCSSERIAL, &ss);
    }
#endif
    tcflush(fd, TCIOFLUSH);
    return fd;
}

int serial_write_all(int fd, const uint8_t *p, size_t n, int timeout_ms)
{
    while (n > 0) {
        const ssize_t w = write(fd, p, n);
        if (w > 0) {
            p += w;
            n -= (size_t)w;
            continue;
        }
        if (w < 0 && errno != EAGAIN && errno != EINTR) {
            return -1;
        }
        struct pollfd pf = {.fd = fd, .events = POLLOUT};
        if (poll(&pf, 1, timeout_ms) <= 0) {
            errno = ETIMEDOUT;
            return -1;
        }
    }
    return 0;
}

int robot_find(char *buf, size_t cap)
{
    static const char *const pat[] = {"/dev/ttyACM%d", "/dev/ttyUSB%d"};
    for (size_t k = 0; k < sizeof pat / sizeof pat[0]; k++) {
        for (int i = 0; i < 4; i++) {
            char p[32];
            snprintf(p, sizeof p, pat[k], i);
            if (access(p, R_OK | W_OK) == 0) {
                snprintf(buf, cap, "%s", p);
                return 0;
            }
        }
    }
    return -1;
}

static int read_hex4(const char *path, unsigned *v)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return -1;
    }
    const int ok = fscanf(f, "%x", v) == 1;
    fclose(f);
    return ok ? 0 : -1;
}

void usb_id_of_tty(const char *dev, char *buf, size_t cap)
{
    buf[0] = '\0';
    const char *name = strrchr(dev, '/');
    name = name ? name + 1 : dev;
    char link[PATH_MAX], real[PATH_MAX];
    snprintf(link, sizeof link, "/sys/class/tty/%s/device", name);
    if (realpath(link, real) == NULL) {
        return;
    }
    /* ttyACM: device = USB interface -> ../idVendor ; ttyUSB: device = port -> ../../idVendor */
    static const char *const up[] = {"/..", "/../.."};
    for (size_t i = 0; i < 2; i++) {
        char pv[PATH_MAX + 32], pp[PATH_MAX + 32];
        unsigned v, p;
        snprintf(pv, sizeof pv, "%s%s/idVendor", real, up[i]);
        snprintf(pp, sizeof pp, "%s%s/idProduct", real, up[i]);
        if (read_hex4(pv, &v) == 0 && read_hex4(pp, &p) == 0) {
            snprintf(buf, cap, "%04x:%04x", v, p);
            return;
        }
    }
}

void status_get(const char *key, char *buf, size_t cap)
{
    buf[0] = '\0';
    FILE *f = fopen(robotd_status_path(), "r");
    if (f == NULL) {
        return;
    }
    char line[256];
    const size_t kl = strlen(key);
    while (fgets(line, sizeof line, f)) {
        if (strncmp(line, key, kl) == 0 && line[kl] == '=') {
            char *v = line + kl + 1;
            v[strcspn(v, "\r\n")] = '\0';
            snprintf(buf, cap, "%s", v);
            break;
        }
    }
    fclose(f);
}

int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static const char *run_path(char *buf, size_t cap, const char *name)
{
    const char *d = getenv("DUSK_RUN_DIR");
    snprintf(buf, cap, "%s/%s", d && *d ? d : "/run", name);
    return buf;
}

const char *robotd_status_path(void)
{
    static char p[PATH_MAX];
    return run_path(p, sizeof p, "dusk_robotd.status");
}

const char *robotd_pause_path(void)
{
    static char p[PATH_MAX];
    return run_path(p, sizeof p, "dusk_robotd.pause");
}
