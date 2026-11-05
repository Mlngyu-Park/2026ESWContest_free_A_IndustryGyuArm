/* serial_util.h — small helpers shared by dusk_robotd and dusk_servo (DUSK_BOOM v3.5 Zynq Linux) */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* run-time files, in $DUSK_RUN_DIR (default /run) */
const char *robotd_status_path(void);   /* dusk_robotd.status */
const char *robotd_pause_path(void);    /* dusk_robotd.pause: while it exists dusk_robotd frees the robot port */

/* open a tty raw 8N1 at baud (standard Linux rates only); -1 on error (errno set) */
int  serial_open(const char *dev, int baud);
/* write all bytes (blocking with timeout_ms per chunk); 0 = ok */
int  serial_write_all(int fd, const uint8_t *p, size_t n, int timeout_ms);
/* first robot adapter that exists: /dev/ttyACM0..3, /dev/ttyUSB0..3 -> buf; 0 = found */
int  robot_find(char *buf, size_t cap);
/* "vvvv:pppp" of the USB device behind /dev/ttyXXX, "" if unknown */
void usb_id_of_tty(const char *dev, char *buf, size_t cap);
/* value of "key=" in the dusk_robotd status file, "" if missing */
void status_get(const char *key, char *buf, size_t cap);
/* milliseconds, monotonic */
int64_t now_ms(void);
