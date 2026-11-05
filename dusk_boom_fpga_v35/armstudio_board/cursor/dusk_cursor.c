/* dusk_cursor.c — mouse cursor for the DUSK_BOOM ARM STUDIO screen (/dev/fb0, no GPU)
 *
 *   The screen program (armstudio_screen.py) redraws the whole picture only when something changes
 *   and takes 0.1–1 s for that on the 667 MHz ARM.  Drawing the cursor there makes the mouse jerky,
 *   so this small program owns the cursor: it reads the mice (/dev/input/event*, EV_REL / touch
 *   EV_ABS), keeps the position, and draws the arrow straight into the framebuffer with a saved
 *   background ("save-under").
 *
 *   Shared with the screen program through /dev/shm/dusk_cursor (struct shared below):
 *     x, y      cursor position (written here, read by the screen program for clicks)
 *     frame     the screen program adds 1 after every write to the framebuffer -> the saved
 *               background is stale: take a new one, do not restore the old one
 *
 *   build: arm-none-linux-gnueabihf-gcc -O2 -static -o dusk_cursor dusk_cursor.c
 */
#include <dirent.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define CW 12
#define CH 19
#define MAXDEV 16

struct shared {
    volatile int32_t x, y;
    volatile uint32_t frame;
    volatile uint32_t hidden;
};

/* arrow: 1 = white, 2 = black outline */
static const char *ARROW[CH] = {
    "2           ", "22          ", "212         ", "2112        ", "21112       ", "211112      ",
    "2111112     ", "21111112    ", "211111112   ", "2111111112  ", "21111111112 ", "211111222222",
    "2112112     ", "212 2112    ", "22  2112    ", "2    2112   ", "     2112   ", "      2112  ",
    "      222   "};

static uint32_t *fb;
static int fbw, fbh, stride_px;
static uint32_t under[CW * CH];
static int ux = -1, uy = -1;            /* where the saved background belongs (-1 = none) */

static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static void save_under(int x, int y)
{
    for (int r = 0; r < CH; r++)
        for (int c = 0; c < CW; c++) {
            int px = x + c, py = y + r;
            under[r * CW + c] = (px < fbw && py < fbh) ? fb[py * stride_px + px] : 0;
        }
    ux = x;
    uy = y;
}

static void restore_under(void)
{
    if (ux < 0)
        return;
    for (int r = 0; r < CH; r++)
        for (int c = 0; c < CW; c++) {
            int px = ux + c, py = uy + r;
            if (px < fbw && py < fbh && ARROW[r][c] != ' ')
                fb[py * stride_px + px] = under[r * CW + c];
        }
    ux = -1;
}

static void draw_arrow(int x, int y)
{
    for (int r = 0; r < CH; r++)
        for (int c = 0; c < CW; c++) {
            int px = x + c, py = y + r;
            char k = ARROW[r][c];
            if (px < fbw && py < fbh && k != ' ')
                fb[py * stride_px + px] = (k == '1') ? 0x00FFFFFF : 0x00000000;
        }
}

struct dev { int fd; char path[64]; int absx_min, absx_max, absy_min, absy_max; };
static struct dev devs[MAXDEV];
static int ndev;

static int test_bit(const unsigned long *a, int b) { return (a[b / (8 * sizeof(long))] >> (b % (8 * sizeof(long)))) & 1; }

static void scan_devices(void)
{
    DIR *d = opendir("/dev/input");
    if (!d)
        return;
    struct dirent *e;
    while ((e = readdir(d)) && ndev < MAXDEV) {
        if (strncmp(e->d_name, "event", 5))
            continue;
        char p[64];
        snprintf(p, sizeof p, "/dev/input/%s", e->d_name);
        int known = 0;
        for (int i = 0; i < ndev; i++)
            if (!strcmp(devs[i].path, p))
                known = 1;
        if (known)
            continue;
        int fd = open(p, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
            continue;
        unsigned long ev[1] = {0}, rel[1] = {0}, abs_[2] = {0};
        ioctl(fd, EVIOCGBIT(0, sizeof ev), ev);
        ioctl(fd, EVIOCGBIT(EV_REL, sizeof rel), rel);
        ioctl(fd, EVIOCGBIT(EV_ABS, sizeof abs_), abs_);
        int is_mouse = test_bit(ev, EV_REL) && test_bit(rel, REL_X);
        int is_touch = test_bit(ev, EV_ABS) && test_bit(abs_, ABS_X) && test_bit(abs_, ABS_Y);
        if (!is_mouse && !is_touch) {
            close(fd);
            continue;
        }
        struct dev *v = &devs[ndev++];
        memset(v, 0, sizeof *v);
        v->fd = fd;
        snprintf(v->path, sizeof v->path, "%s", p);
        if (is_touch) {
            struct input_absinfo a;
            if (!ioctl(fd, EVIOCGABS(ABS_X), &a)) { v->absx_min = a.minimum; v->absx_max = a.maximum; }
            if (!ioctl(fd, EVIOCGABS(ABS_Y), &a)) { v->absy_min = a.minimum; v->absy_max = a.maximum; }
        }
    }
    closedir(d);
}

int main(void)
{
    int ffd = open("/dev/fb0", O_RDWR);
    if (ffd < 0) { perror("fb0"); return 1; }
    struct fb_var_screeninfo vi;
    struct fb_fix_screeninfo fi;
    ioctl(ffd, FBIOGET_VSCREENINFO, &vi);
    ioctl(ffd, FBIOGET_FSCREENINFO, &fi);
    fbw = vi.xres; fbh = vi.yres; stride_px = fi.line_length / 4;
    fb = mmap(NULL, fi.line_length * fbh, PROT_READ | PROT_WRITE, MAP_SHARED, ffd, 0);
    if (fb == MAP_FAILED) { perror("mmap fb"); return 1; }

    int sfd = open("/dev/shm/dusk_cursor", O_RDWR | O_CREAT, 0666);
    if (sfd < 0 || ftruncate(sfd, sizeof(struct shared))) { perror("shm"); return 1; }
    struct shared *sh = mmap(NULL, sizeof *sh, PROT_READ | PROT_WRITE, MAP_SHARED, sfd, 0);
    if (sh == MAP_FAILED) { perror("mmap shm"); return 1; }
    if (sh->x <= 0 && sh->y <= 0) { sh->x = fbw / 2; sh->y = fbh / 2; }

    uint32_t seen_frame = sh->frame - 1;
    int drawn_x = -1, drawn_y = -1;
    double last_scan = 0;
    for (;;) {
        if (now() - last_scan > 2.0) { scan_devices(); last_scan = now(); }
        struct pollfd pf[MAXDEV];
        for (int i = 0; i < ndev; i++) { pf[i].fd = devs[i].fd; pf[i].events = POLLIN; }
        poll(pf, ndev, 8);                                /* ~120 Hz when idle */
        for (int i = 0; i < ndev; i++) {
            if (!(pf[i].revents & (POLLIN | POLLERR | POLLHUP)))
                continue;
            struct input_event ev[64];
            ssize_t n = read(devs[i].fd, ev, sizeof ev);
            if (n <= 0) {                                 /* unplugged */
                close(devs[i].fd);
                devs[i] = devs[--ndev];
                i--;
                continue;
            }
            for (int k = 0; k < (int)(n / sizeof ev[0]); k++) {
                if (ev[k].type == EV_REL && ev[k].code == REL_X) sh->x += ev[k].value;
                else if (ev[k].type == EV_REL && ev[k].code == REL_Y) sh->y += ev[k].value;
                else if (ev[k].type == EV_ABS && ev[k].code == ABS_X && devs[i].absx_max > devs[i].absx_min)
                    sh->x = (ev[k].value - devs[i].absx_min) * (fbw - 1) / (devs[i].absx_max - devs[i].absx_min);
                else if (ev[k].type == EV_ABS && ev[k].code == ABS_Y && devs[i].absy_max > devs[i].absy_min)
                    sh->y = (ev[k].value - devs[i].absy_min) * (fbh - 1) / (devs[i].absy_max - devs[i].absy_min);
            }
            if (sh->x < 0) sh->x = 0;
            if (sh->y < 0) sh->y = 0;
            if (sh->x > fbw - 1) sh->x = fbw - 1;
            if (sh->y > fbh - 1) sh->y = fbh - 1;
        }
        int x = sh->x, y = sh->y;
        uint32_t f = sh->frame;
        if (f != seen_frame) {                            /* new picture underneath: old save is stale */
            seen_frame = f;
            ux = -1;
            drawn_x = -1;
        }
        if (sh->hidden) {
            restore_under();
            drawn_x = -1;
            continue;
        }
        if (x != drawn_x || y != drawn_y) {
            restore_under();
            save_under(x, y);
            draw_arrow(x, y);
            drawn_x = x;
            drawn_y = y;
        }
    }
}
