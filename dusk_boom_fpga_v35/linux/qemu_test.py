#!/usr/bin/env python3
"""qemu_test.py — boot the DUSK_BOOM Zynq Linux (zImage + system.dtb) in QEMU xilinx-zynq-a9 and test
the robot bridge end to end, without hardware:

    fake ESP  --(unix socket = Zynq UART0 / ttyPS1)-->  dusk_robotd  --> /dev/ttyUSB0
    (QEMU usb-serial FTDI on the Zynq USB host)  --(unix socket)-->  fake Feetech servos (IDs 1..5)
    console = UART1 / ttyPS0 (unix socket): motd, dusk_servo scan, status

usage: python3 qemu_test.py SD_DIR      (SD_DIR has zImage and system.dtb)
QEMU limits: no PL (EMIO GPIO / ESP_IO pins are not emulated), the USB PHY is not a USB3320.
"""
import os
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time

SD = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "../sd")
fails = 0


def check(what, ok, info=""):
    global fails
    print(("PASS " if ok else "FAIL ") + what + (f"  ({info})" if info else ""), flush=True)
    if not ok:
        fails += 1


def chk(b):
    return (~sum(b)) & 0xFF


def packet(id_, inst, params=b""):
    body = bytes([id_, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([chk(body)])


def connect(path, timeout=10):
    end = time.time() + timeout
    while time.time() < end:
        try:
            s = socket.socket(socket.AF_UNIX)
            s.connect(path)
            return s
        except OSError:
            time.sleep(0.1)
    raise RuntimeError("cannot connect " + path)


class Servos(threading.Thread):
    def __init__(self, sock):
        super().__init__(daemon=True)
        self.s, self.rx, self.n = sock, bytearray(), 0

    def run(self):
        while True:
            try:
                d = self.s.recv(512)
            except OSError:
                return
            if not d:
                return
            self.rx += d
            while True:
                i = self.rx.find(b"\xff\xff")
                if i < 0 or len(self.rx) < i + 4 or len(self.rx) < i + 4 + self.rx[i + 3]:
                    break
                pkt = bytes(self.rx[i:i + 4 + self.rx[i + 3]])
                del self.rx[:i + len(pkt)]
                id_, inst, par = pkt[2], pkt[4], pkt[5:-1]
                if chk(pkt[2:-1]) != pkt[-1] or not 1 <= id_ <= 5:
                    continue
                rep = b""
                if inst == 0x02 and par[0] == 56:
                    rep = (2048 + id_).to_bytes(2, "little")[:par[1]]
                elif inst == 0x02 and par[0] == 3:
                    rep = (777).to_bytes(2, "little")[:par[1]]
                self.n += 1
                self.s.sendall(packet(id_, 0, rep))


class Console:
    def __init__(self, sock):
        self.s, self.buf = sock, b""

    def expect(self, text, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if text.encode() in self.buf:
                i = self.buf.index(text.encode()) + len(text)
                out, self.buf = self.buf[:i], self.buf[i:]
                return out.decode(errors="replace")
            r, _, _ = select.select([self.s], [], [], 0.2)
            if r:
                d = self.s.recv(4096)
                if not d:
                    break
                self.buf += d
        return None

    def run(self, cmd, timeout=20):
        mark = "__END%d__" % int(time.time() * 1000 % 100000)
        self.s.sendall((cmd + f"; echo {mark}\n").encode())
        out = self.expect(mark + "\r\n", timeout) or self.expect(mark, 1) or ""
        return out


def main():
    t = tempfile.mkdtemp(prefix="dusk_qemu_")
    socks = {k: os.path.join(t, k + ".sock") for k in ("esp", "con", "rob")}
    cmd = ["qemu-system-arm", "-M", "xilinx-zynq-a9", "-m", "512M", "-display", "none", "-monitor", "none",
           "-kernel", os.path.join(SD, "zImage"), "-dtb", os.path.join(SD, "system.dtb"),
           "-append", "console=ttyPS0,115200 earlycon",
           "-chardev", f"socket,id=esp,path={socks['esp']},server=on,wait=off", "-serial", "chardev:esp",
           "-chardev", f"socket,id=con,path={socks['con']},server=on,wait=off", "-serial", "chardev:con",
           "-chardev", f"socket,id=rob,path={socks['rob']},server=on,wait=off",
           "-device", "usb-serial,chardev=rob"]
    q = subprocess.Popen(["taskset", "-c", "0-3"] + cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        con = Console(connect(socks["con"]))
        esp = connect(socks["esp"])
        servos = Servos(connect(socks["rob"]))
        servos.start()

        boot = con.expect("DUSK_BOOM robot v3.5", 180)
        check("Linux boots to the motd", boot is not None)
        if boot is None:
            print(con.buf.decode(errors="replace")[-3000:])
            return 1
        con.expect("dusk_servo scan", 5)
        time.sleep(3)
        out = con.run("uname -a; cat /proc/device-tree/model; echo; ls /dev/ttyPS* /dev/ttyUSB* /dev/ttyACM* 2>&1")
        print(out)
        check("ttyPS0 + ttyPS1 present", "/dev/ttyPS0" in out and "/dev/ttyPS1" in out)
        check("model = DUSK_BOOM", "DUSK_BOOM robot v3.5" in out)
        usb = "/dev/ttyUSB0" in out
        check("USB serial adapter -> /dev/ttyUSB0", usb)
        out = con.run("sleep 2; cat /run/dusk_robotd.status; cat /var/log/dusk_robotd.log")
        print(out)
        check("dusk_robotd running", "bridge=1" in out)
        if usb:
            check("dusk_robotd opened the adapter", "robot=/dev/ttyUSB0" in out)
            esp.sendall(packet(3, 0x02, bytes([56, 2])))
            rep, end = b"", time.time() + 5
            while len(rep) < 8 and time.time() < end:
                r, _, _ = select.select([esp], [], [], 0.2)
                if r:
                    rep += esp.recv(64)
            check("ESP READ pos 3 through Linux + USB", rep == packet(3, 0, (2051).to_bytes(2, "little")), rep.hex())
            out = con.run("dusk_servo scan 1 6", 30)
            print(out)
            check("dusk_servo scan finds 5", "5 servo(s) found" in out)
        out = con.run("ls /sys/class/gpio; cat /sys/class/gpio/gpiochip*/label; ls /sys/class/leds")
        print(out)
        check("zynq_gpio present", "zynq_gpio" in out)
        check("PS LED (heartbeat)", "ps_led" in out)
    finally:
        q.kill()
        q.wait()
    print(f"==== {'ALL PASS' if fails == 0 else str(fails) + ' FAIL'} ====")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
