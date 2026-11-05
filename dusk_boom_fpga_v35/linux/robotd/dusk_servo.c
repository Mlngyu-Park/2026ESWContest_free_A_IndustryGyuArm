/* dusk_servo — Feetech bus servo test tool on the DUSK_BOOM v3.5 Zynq (Linux console)
 *
 * Talks directly to the robot USB serial adapter on the module USB-A. While it runs, dusk_robotd
 * releases the port (pause file) and takes it back afterwards.
 *
 *   dusk_servo scan [FIRST LAST]          ping IDs (default 1..20): model + present position
 *   dusk_servo ping ID
 *   dusk_servo pos ID                     present position (STS3215: 0..4095)
 *   dusk_servo read ID ADDR LEN           raw register read
 *   dusk_servo torque ID 0|1              torque off / on            (servo holds or goes limp)
 *   dusk_servo move ID POS [TIME [SPEED]] goal position            !! the servo MOVES !!
 *   options: -p DEV (default: dusk_robotd's port, else auto) -b BAUD (1000000) -B big-endian (SCS)
 *
 * Feetech protocol: FF FF ID LEN INSTR PARAM.. CHK, LEN = params + 2, CHK = ~(ID+LEN+INSTR+params).
 * STS registers: 3 model (2) · 40 torque enable · 42 goal position (2) · 44 goal time (2) ·
 *                46 goal speed (2) · 56 present position (2). STS = little-endian, SCS = big-endian.
 */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "serial_util.h"

#define INST_PING  0x01
#define INST_READ  0x02
#define INST_WRITE 0x03

#define REG_MODEL   3
#define REG_TORQUE  40
#define REG_GOAL    42
#define REG_POS     56

static int g_fd = -1;
static bool g_be = false;
static bool g_paused = false;
static int g_timeout_ms = 30;

static uint8_t checksum(const uint8_t *p, size_t n)   /* over ID..last param */
{
    unsigned s = 0;
    for (size_t i = 0; i < n; i++) {
        s += p[i];
    }
    return (uint8_t)~s;
}

/* build + send one instruction packet */
static int send_packet(uint8_t id, uint8_t inst, const uint8_t *par, size_t np, uint8_t *pkt, size_t *pktlen)
{
    size_t k = 0;
    pkt[k++] = 0xFF;
    pkt[k++] = 0xFF;
    pkt[k++] = id;
    pkt[k++] = (uint8_t)(np + 2);
    pkt[k++] = inst;
    memcpy(pkt + k, par, np);
    k += np;
    pkt[k] = checksum(pkt + 2, k - 2);
    k++;
    *pktlen = k;
    tcflush(g_fd, TCIFLUSH);
    return serial_write_all(g_fd, pkt, k, 100);
}

/* wait for a status packet from id; returns number of params (>= 0) or -1 on timeout.
 * Skips an echo of our own packet (half-duplex adapters may echo). */
static int recv_status(uint8_t id, const uint8_t *tx, size_t txlen, uint8_t *err, uint8_t *par, size_t cap)
{
    uint8_t rx[300];
    size_t n = 0;
    const int64_t end = now_ms() + g_timeout_ms;
    for (;;) {
        const int64_t left = end - now_ms();
        if (left <= 0) {
            return -1;
        }
        struct pollfd pf = {.fd = g_fd, .events = POLLIN};
        if (poll(&pf, 1, (int)left) <= 0) {
            continue;
        }
        const ssize_t got = read(g_fd, rx + n, sizeof rx - n);
        if (got <= 0) {
            continue;
        }
        n += (size_t)got;
        size_t s = 0;
        if (n >= txlen && memcmp(rx, tx, txlen) == 0) {
            s = txlen;                                   /* echo */
        }
        for (size_t i = s; i + 6 <= n; i++) {
            if (rx[i] != 0xFF || rx[i + 1] != 0xFF || rx[i + 2] != id) {
                continue;
            }
            const size_t len = rx[i + 3];
            if (len < 2 || i + 4 + len > n) {
                break;                                   /* wait for the rest */
            }
            if (checksum(rx + i + 2, len + 1) != rx[i + 3 + len]) {
                continue;
            }
            *err = rx[i + 4];
            const size_t np = len - 2;
            memcpy(par, rx + i + 5, np < cap ? np : cap);
            return (int)np;
        }
        if (n == sizeof rx) {
            n = 0;
        }
    }
}

static int txrx(uint8_t id, uint8_t inst, const uint8_t *par, size_t np, uint8_t *err, uint8_t *out, size_t cap)
{
    uint8_t pkt[300];
    size_t pl;
    if (send_packet(id, inst, par, np, pkt, &pl) != 0) {
        fprintf(stderr, "write failed: %s\n", strerror(errno));
        return -1;
    }
    return recv_status(id, pkt, pl, err, out, cap);
}

static int read_u16(uint8_t id, uint8_t addr, unsigned *v)
{
    uint8_t par[2] = {addr, 2}, err, d[4];
    if (txrx(id, INST_READ, par, 2, &err, d, sizeof d) != 2) {
        return -1;
    }
    *v = g_be ? (unsigned)(d[0] << 8 | d[1]) : (unsigned)(d[1] << 8 | d[0]);
    return err;
}

static void put_u16(uint8_t *p, unsigned v)
{
    if (g_be) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
    else      { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
}

/* ------------------------------------------------------------------ port handling */

static void pause_robotd(void)
{
    char pid[16];
    status_get("pid", pid, sizeof pid);
    if (pid[0] == '\0') {
        return;                                          /* dusk_robotd not running */
    }
    const int fd = open(robotd_pause_path(), O_WRONLY | O_CREAT | O_CLOEXEC, 0644);
    if (fd >= 0) {
        close(fd);
        g_paused = true;
    }
    /* wait until it has closed the port (status robot= empty), max 1 s */
    for (int i = 0; i < 50; i++) {
        char r[64];
        status_get("robot", r, sizeof r);
        if (r[0] == '\0') {
            break;
        }
        usleep(20000);
    }
}

static void resume_robotd(void)
{
    if (g_paused) {
        unlink(robotd_pause_path());
        g_paused = false;
    }
}

static void usage(void)
{
    fprintf(stderr,
            "usage: dusk_servo [-p DEV] [-b BAUD] [-B] CMD ...\n"
            "  scan [FIRST LAST]          ping IDs (default 1..20)\n"
            "  ping ID | pos ID | read ID ADDR LEN\n"
            "  torque ID 0|1\n"
            "  move ID POS [TIME_MS [SPEED]]   (the servo MOVES)\n");
}

static int cmd_scan(int first, int last)
{
    int found = 0;
    for (int id = first; id <= last; id++) {
        uint8_t err, d[8];
        if (txrx((uint8_t)id, INST_PING, NULL, 0, &err, d, sizeof d) < 0) {
            continue;
        }
        unsigned model = 0, pos = 0;
        const int em = read_u16((uint8_t)id, REG_MODEL, &model);
        const int ep = read_u16((uint8_t)id, REG_POS, &pos);
        printf("ID %3d  answers  err=0x%02X  model=%s%u  position=%s%u\n", id, err,
               em < 0 ? "?" : "", em < 0 ? 0 : model, ep < 0 ? "?" : "", ep < 0 ? 0 : pos);
        found++;
    }
    printf("%d servo(s) found in %d..%d\n", found, first, last);
    return found > 0 ? 0 : 1;
}

int main(int argc, char **argv)
{
    const char *dev = NULL;
    int baud = 1000000, opt;
    while ((opt = getopt(argc, argv, "p:b:Bh")) != -1) {
        switch (opt) {
        case 'p': dev = optarg; break;
        case 'b': baud = atoi(optarg); break;
        case 'B': g_be = true; break;
        default: usage(); return opt == 'h' ? 0 : 2;
        }
    }
    if (optind >= argc) {
        usage();
        return 2;
    }
    const char *cmd = argv[optind];
    char **a = argv + optind + 1;
    const int na = argc - optind - 1;

    char devbuf[64];
    if (dev == NULL) {
        status_get("robot", devbuf, sizeof devbuf);
        if (devbuf[0] == '\0' && robot_find(devbuf, sizeof devbuf) != 0) {
            fprintf(stderr, "no robot adapter (/dev/ttyACM*, /dev/ttyUSB*) — is it on the module USB-A, J5/J6 on?\n");
            return 1;
        }
        dev = devbuf;
    }
    pause_robotd();
    g_fd = serial_open(dev, baud);
    if (g_fd < 0) {
        fprintf(stderr, "%s: %s\n", dev, strerror(errno));
        resume_robotd();
        return 1;
    }
    int rc = 0;
    uint8_t err, d[64];
    if (strcmp(cmd, "scan") == 0) {
        rc = cmd_scan(na >= 1 ? atoi(a[0]) : 1, na >= 2 ? atoi(a[1]) : 20);
    } else if (strcmp(cmd, "ping") == 0 && na == 1) {
        const int r = txrx((uint8_t)atoi(a[0]), INST_PING, NULL, 0, &err, d, sizeof d);
        if (r < 0) { printf("ID %s: no answer\n", a[0]); rc = 1; }
        else       { printf("ID %s: answers, err=0x%02X\n", a[0], err); }
    } else if (strcmp(cmd, "pos") == 0 && na == 1) {
        unsigned v;
        const int r = read_u16((uint8_t)atoi(a[0]), REG_POS, &v);
        if (r < 0) { printf("ID %s: no answer\n", a[0]); rc = 1; }
        else       { printf("ID %s: position %u (err=0x%02X)\n", a[0], v, r); }
    } else if (strcmp(cmd, "read") == 0 && na == 3) {
        uint8_t par[2] = {(uint8_t)atoi(a[1]), (uint8_t)atoi(a[2])};
        const int r = txrx((uint8_t)atoi(a[0]), INST_READ, par, 2, &err, d, sizeof d);
        if (r < 0) { printf("ID %s: no answer\n", a[0]); rc = 1; }
        else {
            printf("ID %s addr %s:", a[0], a[1]);
            for (int i = 0; i < r && i < (int)sizeof d; i++) printf(" %02X", d[i]);
            printf("  (err=0x%02X)\n", err);
        }
    } else if (strcmp(cmd, "torque") == 0 && na == 2) {
        uint8_t par[2] = {REG_TORQUE, (uint8_t)(atoi(a[1]) ? 1 : 0)};
        const int r = txrx((uint8_t)atoi(a[0]), INST_WRITE, par, 2, &err, d, sizeof d);
        if (r < 0) { printf("ID %s: no answer\n", a[0]); rc = 1; }
        else       { printf("ID %s: torque %s (err=0x%02X)\n", a[0], par[1] ? "on" : "off", err); }
    } else if (strcmp(cmd, "move") == 0 && na >= 2) {
        uint8_t par[7];
        par[0] = REG_GOAL;
        put_u16(par + 1, (unsigned)atoi(a[1]));
        put_u16(par + 3, na >= 3 ? (unsigned)atoi(a[2]) : 0);
        put_u16(par + 5, na >= 4 ? (unsigned)atoi(a[3]) : 0);
        const int r = txrx((uint8_t)atoi(a[0]), INST_WRITE, par, 7, &err, d, sizeof d);
        if (r < 0) { printf("ID %s: no answer\n", a[0]); rc = 1; }
        else       { printf("ID %s: goal %s sent (err=0x%02X)\n", a[0], a[1], err); }
    } else {
        usage();
        rc = 2;
    }
    close(g_fd);
    resume_robotd();
    return rc;
}
