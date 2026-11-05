/* dusk_robotd — robot bridge on the DUSK_BOOM v3.5 Zynq (Linux, ARM side of the AC7010C)
 *
 *   ESP32-S3  <->  /dev/ttyPS1 (Zynq UART0 over EMIO -> PL -> ESP_IO0/1, 1 Mbps)
 *             <->  this program
 *             <->  USB serial adapter on the module USB-A (/dev/ttyACM* CH343, /dev/ttyUSB* CH340/CP210x/FTDI)
 *             <->  Feetech bus servos
 *
 * Transparent byte relay: the Feetech protocol stays in the ESP firmware (robot_link.c), exactly as
 * if the adapter were plugged into the ESP. Optionally a TCP port (-t) lets a PC use the same robot
 * (e.g. pyserial "socket://<board-ip>:<port>"); replies go to whoever sent last.
 *
 * EMIO GPIO (see rtl/dusk_boom_v35_top.v): 7 = bridge running (the FPGA drives ESP_IO0 only then),
 *                                           6 = robot adapter open (STATUS bit4, module LED steady on).
 * While robotd_pause_path() exists the robot port is released (dusk_servo takes it for a moment).
 */
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "serial_util.h"

#define EMIO_FIRST       54        /* zynq_gpio: MIO 0..53, then EMIO */
#define GPIO_ROBOT_OPEN  6
#define GPIO_BRIDGE_ON   7
#define RETRY_MS         500
#define STATUS_MS        2000

static const char *g_esp_dev = "/dev/ttyPS1";
static const char *g_robot_arg = "auto";
static int g_baud = 1000000;
static int g_tcp_port = 0;
static bool g_gpio = true;
static bool g_verbose = false;
static bool g_console = false;
static volatile sig_atomic_t g_stop = 0;

static int g_esp = -1, g_robot = -1, g_listen = -1, g_client = -1;
static char g_robot_dev[64] = "", g_usb_id[16] = "";
static unsigned long g_n_e2r, g_n_r2e, g_n_t2r, g_n_r2t, g_n_drop;
static enum { SRC_ESP, SRC_TCP } g_last_src = SRC_ESP;
static int g_gpio_base = -1;

static void logmsg(const char *fmt, ...)
{
    char msg[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    fprintf(stderr, "%02d:%02d:%02d dusk_robotd: %s\n", tm.tm_hour, tm.tm_min, tm.tm_sec, msg);
    if (g_console) {
        FILE *c = fopen("/dev/console", "w");
        if (c) {
            fprintf(c, "[dusk_robotd] %s\n", msg);
            fclose(c);
        }
    }
}

static void hexdump(const char *tag, const uint8_t *p, size_t n)
{
    if (!g_verbose) {
        return;
    }
    char line[3 * 48 + 1];
    size_t k = 0;
    for (size_t i = 0; i < n && i < 48; i++) {
        k += (size_t)snprintf(line + k, sizeof line - k, "%02X ", p[i]);
    }
    fprintf(stderr, "  %s %zu: %s%s\n", tag, n, line, n > 48 ? "..." : "");
}

/* ------------------------------------------------------------------ EMIO GPIO (sysfs) */

static int write_file(const char *path, const char *s)
{
    const int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        return -1;
    }
    const ssize_t w = write(fd, s, strlen(s));
    close(fd);
    return w == (ssize_t)strlen(s) ? 0 : -1;
}

static int find_zynq_gpio_base(void)
{
    DIR *dir = opendir("/sys/class/gpio");
    if (dir == NULL) {
        return -1;
    }
    int base = -1;
    struct dirent *de;
    while (base < 0 && (de = readdir(dir)) != NULL) {
        if (strncmp(de->d_name, "gpiochip", 8) != 0) {
            continue;
        }
        char p[300], label[64] = "";
        snprintf(p, sizeof p, "/sys/class/gpio/%s/label", de->d_name);
        FILE *f = fopen(p, "r");
        if (f == NULL) {
            continue;
        }
        const bool ok = fgets(label, sizeof label, f) != NULL;
        fclose(f);
        if (ok && strncmp(label, "zynq_gpio", 9) == 0) {
            base = atoi(de->d_name + 8);
        }
    }
    closedir(dir);
    return base;
}

static void emio_set(int n, int v)
{
    if (!g_gpio || g_gpio_base < 0) {
        return;
    }
    const int g = g_gpio_base + EMIO_FIRST + n;
    char p[64], num[16];
    snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/direction", g);
    if (access(p, F_OK) != 0) {
        snprintf(num, sizeof num, "%d", g);
        (void)write_file("/sys/class/gpio/export", num);
    }
    /* "high"/"low" sets direction and value in one step (no glitch) */
    if (write_file(p, v ? "high" : "low") != 0) {
        logmsg("GPIO %d (EMIO%d) not writable: %s", g, n, strerror(errno));
    }
}

/* ------------------------------------------------------------------ status file */

static void write_status(void)
{
    char tmp[300];
    snprintf(tmp, sizeof tmp, "%s.tmp", robotd_status_path());
    FILE *f = fopen(tmp, "w");
    if (f == NULL) {
        return;
    }
    fprintf(f, "pid=%d\nbridge=1\nesp=%s\nrobot=%s\nusb=%s\nbaud=%d\ntcp=%d\ntcp_client=%d\n"
               "esp_to_robot=%lu\nrobot_to_esp=%lu\ntcp_to_robot=%lu\nrobot_to_tcp=%lu\ndropped=%lu\n",
            (int)getpid(), g_esp >= 0 ? g_esp_dev : "", g_robot >= 0 ? g_robot_dev : "", g_usb_id,
            g_baud, g_tcp_port, g_client >= 0, g_n_e2r, g_n_r2e, g_n_t2r, g_n_r2t, g_n_drop);
    fclose(f);
    rename(tmp, robotd_status_path());
}

/* ------------------------------------------------------------------ robot port */

static void robot_close(const char *why)
{
    if (g_robot >= 0) {
        close(g_robot);
        g_robot = -1;
        logmsg("robot %s closed (%s)", g_robot_dev, why);
        emio_set(GPIO_ROBOT_OPEN, 0);
        write_status();
    }
}

static void robot_try_open(void)
{
    char dev[64];
    if (strcmp(g_robot_arg, "auto") == 0) {
        if (robot_find(dev, sizeof dev) != 0) {
            return;
        }
    } else {
        snprintf(dev, sizeof dev, "%s", g_robot_arg);
        if (access(dev, F_OK) != 0) {
            return;
        }
    }
    const int fd = serial_open(dev, g_baud);
    if (fd < 0) {
        static int64_t last_warn;
        if (now_ms() - last_warn > 10000) {
            logmsg("robot %s: open failed: %s", dev, strerror(errno));
            last_warn = now_ms();
        }
        return;
    }
    g_robot = fd;
    snprintf(g_robot_dev, sizeof g_robot_dev, "%s", dev);
    usb_id_of_tty(dev, g_usb_id, sizeof g_usb_id);
    logmsg("robot open: %s (USB %s) %d 8N1", g_robot_dev, g_usb_id[0] ? g_usb_id : "?", g_baud);
    emio_set(GPIO_ROBOT_OPEN, 1);
    write_status();
}

/* ------------------------------------------------------------------ TCP */

static void tcp_listen(void)
{
    g_listen = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (g_listen < 0) {
        logmsg("tcp socket: %s", strerror(errno));
        return;
    }
    const int one = 1;
    setsockopt(g_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons((uint16_t)g_tcp_port),
                            .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (bind(g_listen, (struct sockaddr *)&a, sizeof a) != 0 || listen(g_listen, 1) != 0) {
        logmsg("tcp port %d: %s", g_tcp_port, strerror(errno));
        close(g_listen);
        g_listen = -1;
        return;
    }
    logmsg("TCP robot port %d open (one client at a time)", g_tcp_port);
}

static void tcp_accept(void)
{
    const int c = accept(g_listen, NULL, NULL);
    if (c < 0) {
        return;
    }
    if (g_client >= 0) {
        close(g_client);
        logmsg("TCP client replaced");
    }
    const int one = 1;
    setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    fcntl(c, F_SETFL, fcntl(c, F_GETFL) | O_NONBLOCK);
    g_client = c;
    logmsg("TCP client connected");
    write_status();
}

/* ------------------------------------------------------------------ main loop */

static void on_signal(int s)
{
    (void)s;
    g_stop = 1;
}

static void usage(void)
{
    fprintf(stderr,
            "usage: dusk_robotd [-e ESPDEV|none] [-r ROBOTDEV|auto] [-b BAUD] [-t TCPPORT] [-n] [-v] [-c]\n"
            "  -e  ESP32 side tty (default /dev/ttyPS1 = Zynq UART0 over EMIO)\n"
            "  -r  robot USB serial adapter (default auto: /dev/ttyACM0..3, /dev/ttyUSB0..3)\n"
            "  -b  baud rate for both sides (default 1000000)\n"
            "  -t  also accept one TCP client on this port (default 0 = off)\n"
            "  -n  do not touch the EMIO GPIO (tests on a PC)\n"
            "  -v  hex dump of every chunk   -c  also print events on /dev/console\n");
}

int main(int argc, char **argv)
{
    int opt;
    while ((opt = getopt(argc, argv, "e:r:b:t:nvch")) != -1) {
        switch (opt) {
        case 'e': g_esp_dev = optarg; break;
        case 'r': g_robot_arg = optarg; break;
        case 'b': g_baud = atoi(optarg); break;
        case 't': g_tcp_port = atoi(optarg); break;
        case 'n': g_gpio = false; break;
        case 'v': g_verbose = true; break;
        case 'c': g_console = true; break;
        default: usage(); return opt == 'h' ? 0 : 2;
        }
    }
    signal(SIGPIPE, SIG_IGN);
    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);
    signal(SIGHUP, on_signal);

    if (g_gpio) {
        g_gpio_base = find_zynq_gpio_base();
        if (g_gpio_base < 0) {
            logmsg("zynq_gpio not found in /sys/class/gpio — status bits / ESP_IO0 driver stay off");
        }
    }
    if (strcmp(g_esp_dev, "none") != 0) {
        g_esp = serial_open(g_esp_dev, g_baud);
        if (g_esp < 0) {
            logmsg("ESP side %s: %s", g_esp_dev, strerror(errno));
            return 1;
        }
    }
    if (g_tcp_port > 0) {
        tcp_listen();
    }
    /* the FPGA starts driving ESP_IO0 (our UART TX, idle high) only now */
    emio_set(GPIO_ROBOT_OPEN, 0);
    emio_set(GPIO_BRIDGE_ON, 1);
    logmsg("bridge running: ESP %s <-> robot %s, %d baud", g_esp >= 0 ? g_esp_dev : "(none)",
           g_robot_arg, g_baud);
    write_status();

    int64_t next_try = 0, next_status = now_ms() + STATUS_MS;
    uint8_t buf[512];
    while (!g_stop) {
        const bool paused = access(robotd_pause_path(), F_OK) == 0;
        if (paused && g_robot >= 0) {
            robot_close("paused for dusk_servo");
        }
        if (!paused && g_robot < 0 && now_ms() >= next_try) {
            robot_try_open();
            next_try = now_ms() + RETRY_MS;
        }

        struct pollfd pf[4];
        int n = 0, i_esp = -1, i_rob = -1, i_lis = -1, i_cli = -1;
        if (g_esp >= 0)    { i_esp = n; pf[n++] = (struct pollfd){.fd = g_esp, .events = POLLIN}; }
        if (g_robot >= 0)  { i_rob = n; pf[n++] = (struct pollfd){.fd = g_robot, .events = POLLIN}; }
        if (g_listen >= 0) { i_lis = n; pf[n++] = (struct pollfd){.fd = g_listen, .events = POLLIN}; }
        if (g_client >= 0) { i_cli = n; pf[n++] = (struct pollfd){.fd = g_client, .events = POLLIN}; }
        const int r = poll(pf, (nfds_t)n, 200);
        if (r < 0 && errno != EINTR) {
            logmsg("poll: %s", strerror(errno));
            break;
        }

        /* ESP -> robot */
        if (i_esp >= 0 && (pf[i_esp].revents & POLLIN)) {
            const ssize_t got = read(g_esp, buf, sizeof buf);
            if (got > 0) {
                hexdump("esp>rob", buf, (size_t)got);
                g_last_src = SRC_ESP;
                if (g_robot >= 0 && serial_write_all(g_robot, buf, (size_t)got, 100) == 0) {
                    g_n_e2r += (unsigned long)got;
                } else {
                    g_n_drop += (unsigned long)got;
                    if (g_robot >= 0) {
                        robot_close(strerror(errno));
                    }
                }
            }
        }
        /* TCP -> robot */
        if (i_cli >= 0 && (pf[i_cli].revents & (POLLIN | POLLHUP | POLLERR))) {
            const ssize_t got = read(g_client, buf, sizeof buf);
            if (got <= 0 && !(got < 0 && errno == EAGAIN)) {
                close(g_client);
                g_client = -1;
                logmsg("TCP client gone");
                write_status();
            } else if (got > 0) {
                hexdump("tcp>rob", buf, (size_t)got);
                g_last_src = SRC_TCP;
                if (g_robot >= 0 && serial_write_all(g_robot, buf, (size_t)got, 100) == 0) {
                    g_n_t2r += (unsigned long)got;
                } else {
                    g_n_drop += (unsigned long)got;
                }
            }
        }
        /* robot -> ESP or TCP (whoever sent last) */
        if (i_rob >= 0 && g_robot >= 0 && (pf[i_rob].revents & (POLLIN | POLLHUP | POLLERR))) {
            const ssize_t got = read(g_robot, buf, sizeof buf);
            if (got > 0) {
                if (g_last_src == SRC_TCP && g_client >= 0) {
                    hexdump("rob>tcp", buf, (size_t)got);
                    if (serial_write_all(g_client, buf, (size_t)got, 100) == 0) {
                        g_n_r2t += (unsigned long)got;
                    }
                } else if (g_esp >= 0) {
                    hexdump("rob>esp", buf, (size_t)got);
                    if (serial_write_all(g_esp, buf, (size_t)got, 100) == 0) {
                        g_n_r2e += (unsigned long)got;
                    }
                }
            } else if (got == 0 || (errno != EAGAIN && errno != EINTR)) {
                robot_close(got == 0 ? "unplugged" : strerror(errno));   /* USB unplugged */
            }
        }
        if (i_lis >= 0 && (pf[i_lis].revents & POLLIN)) {
            tcp_accept();
        }
        if (now_ms() >= next_status) {
            write_status();
            next_status = now_ms() + STATUS_MS;
        }
    }

    logmsg("stopping");
    emio_set(GPIO_BRIDGE_ON, 0);       /* FPGA releases ESP_IO0 */
    robot_close("exit");
    emio_set(GPIO_ROBOT_OPEN, 0);
    unlink(robotd_status_path());
    return 0;
}
