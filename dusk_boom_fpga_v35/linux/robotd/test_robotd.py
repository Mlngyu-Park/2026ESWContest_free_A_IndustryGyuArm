#!/usr/bin/env python3
"""test_robotd.py — PC test of dusk_robotd + dusk_servo with pseudo terminals (no hardware).

    fake ESP  <-pty->  dusk_robotd  <-pty->  fake Feetech servos (IDs 1..5, STS3215-like)
    TCP client ------->     ^
    dusk_servo (pause file) -> takes the robot port for a moment

usage: python3 test_robotd.py BUILD_DIR      (native build: make CC=gcc STATIC= all)
"""
import os
import select
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import tty

BUILD = sys.argv[1] if len(sys.argv) > 1 else "build"
fails = 0
PROCS = []


def check(what, ok, info=""):
    global fails
    print(("PASS " if ok else "FAIL ") + what + (f"  ({info})" if info else ""))
    if not ok:
        fails += 1


def chk(b):
    return (~sum(b)) & 0xFF


def packet(id_, inst, params=b""):
    body = bytes([id_, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([chk(body)])


class FakeServos(threading.Thread):
    """Feetech servos on the master side of a pty: PING, READ, WRITE."""

    def __init__(self, fd, ids):
        super().__init__(daemon=True)
        self.fd, self.ids, self.stop = fd, ids, False
        self.regs = {i: bytearray(80) for i in ids}
        for i in ids:
            self.regs[i][3:5] = (777).to_bytes(2, "little")          # model
            self.regs[i][56:58] = (2048 + i).to_bytes(2, "little")   # present position
        self.rx = bytearray()
        self.writes = []

    def run(self):
        while not self.stop:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if not r:
                continue
            try:
                self.rx += os.read(self.fd, 512)
            except OSError:
                continue
            while True:
                i = self.rx.find(b"\xff\xff")
                if i < 0 or len(self.rx) < i + 4:
                    break
                ln = self.rx[i + 3]
                if len(self.rx) < i + 4 + ln:
                    break
                pkt = bytes(self.rx[i:i + 4 + ln])
                del self.rx[:i + 4 + ln]
                id_, inst, par = pkt[2], pkt[4], pkt[5:-1]
                if chk(pkt[2:-1]) != pkt[-1] or id_ not in self.ids:
                    continue
                if inst == 0x01:
                    rep = b""
                elif inst == 0x02:
                    a, n = par[0], par[1]
                    rep = bytes(self.regs[id_][a:a + n])
                elif inst == 0x03:
                    a = par[0]
                    self.regs[id_][a:a + len(par) - 1] = par[1:]
                    self.writes.append((id_, a, bytes(par[1:])))
                    rep = b""
                else:
                    continue
                time.sleep(0.0005)                                    # servo return delay
                os.write(self.fd, packet(id_, 0x00, rep))


def read_for(fd, n, timeout=1.0):
    out = bytearray()
    end = time.time() + timeout
    while len(out) < n and time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            out += os.read(fd, 512)
    return bytes(out)


def status(run_dir):
    try:
        with open(os.path.join(run_dir, "dusk_robotd.status")) as f:
            return dict(l.strip().split("=", 1) for l in f if "=" in l)
    except FileNotFoundError:
        return {}


def main():
    run_dir = tempfile.mkdtemp(prefix="dusk_run_")
    env = dict(os.environ, DUSK_RUN_DIR=run_dir)
    esp_m, esp_s = os.openpty()
    rob_m, rob_s = os.openpty()
    for fd in (esp_m, esp_s, rob_m, rob_s):
        tty.setraw(fd)
    esp_dev, rob_dev = os.ttyname(esp_s), os.ttyname(rob_s)
    servos = FakeServos(rob_m, [1, 2, 3, 4, 5])
    servos.start()

    with socket.socket() as t:                     # a free TCP port
        t.bind(("127.0.0.1", 0))
        port = t.getsockname()[1]
    d = subprocess.Popen([os.path.join(BUILD, "dusk_robotd"), "-n", "-v", "-e", esp_dev, "-r", rob_dev,
                          "-t", str(port)], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    PROCS.append(d)
    time.sleep(0.8)
    st = status(run_dir)
    check("status file: robot open", st.get("robot") == rob_dev, str(st))
    check("status file: bridge=1", st.get("bridge") == "1")

    # ESP -> PING 1 -> reply back to the ESP
    os.write(esp_m, packet(1, 0x01))
    rep = read_for(esp_m, 6)
    check("ESP PING 1 -> reply", rep == packet(1, 0x00), rep.hex())

    # ESP -> READ position of 3
    os.write(esp_m, packet(3, 0x02, bytes([56, 2])))
    rep = read_for(esp_m, 8)
    check("ESP READ pos 3", rep == packet(3, 0x00, (2051).to_bytes(2, "little")), rep.hex())

    # unknown ID: no reply, nothing on the ESP
    os.write(esp_m, packet(9, 0x01))
    rep = read_for(esp_m, 6, 0.2)
    check("ESP PING 9 -> silence", rep == b"", rep.hex())

    # TCP client gets its own replies, the ESP gets nothing
    s = socket.create_connection(("127.0.0.1", port), timeout=2)
    time.sleep(0.1)
    s.sendall(packet(2, 0x02, bytes([56, 2])))
    s.settimeout(1.0)
    rep = b""
    try:
        while len(rep) < 8:
            rep += s.recv(64)
    except socket.timeout:
        pass
    check("TCP READ pos 2 -> TCP", rep == packet(2, 0x00, (2050).to_bytes(2, "little")), rep.hex())
    check("... and not to the ESP", read_for(esp_m, 1, 0.2) == b"")
    # ESP again -> back to the ESP
    os.write(esp_m, packet(4, 0x01))
    rep = read_for(esp_m, 6)
    check("ESP after TCP -> ESP", rep == packet(4, 0x00), rep.hex())
    s.close()

    # dusk_servo: pause file, scan, resume
    r = subprocess.run([os.path.join(BUILD, "dusk_servo"), "-p", rob_dev, "scan", "1", "8"], env=env,
                       capture_output=True, text=True, timeout=10)
    print(r.stdout.rstrip())
    check("dusk_servo scan finds 5", "5 servo(s) found" in r.stdout, r.stderr.strip())
    check("dusk_servo scan position", "ID   3  answers  err=0x00  model=777  position=2051" in r.stdout)
    check("pause file removed", not os.path.exists(os.path.join(run_dir, "dusk_robotd.pause")))
    r = subprocess.run([os.path.join(BUILD, "dusk_servo"), "-p", rob_dev, "move", "2", "1000", "500"],
                       env=env, capture_output=True, text=True, timeout=10)
    check("dusk_servo move -> WRITE 42", (2, 42, bytes([0xE8, 0x03, 0xF4, 0x01, 0, 0])) in servos.writes,
          r.stdout.strip())
    time.sleep(1.0)
    st = status(run_dir)
    check("robotd took the port back", st.get("robot") == rob_dev, str(st))
    os.write(esp_m, packet(5, 0x01))
    rep = read_for(esp_m, 6)
    check("ESP PING 5 after dusk_servo", rep == packet(5, 0x00), rep.hex())

    # many packets in a row (sync-write style burst)
    burst = b"".join(packet(i, 0x01) for i in (1, 2, 3, 4, 5))
    t0 = time.time()
    os.write(esp_m, burst)
    rep = read_for(esp_m, 30)
    check("5 PINGs in one burst -> 5 replies", rep == b"".join(packet(i, 0x00) for i in (1, 2, 3, 4, 5)),
          f"{len(rep)} bytes, {1000 * (time.time() - t0):.1f} ms")

    # adapter unplugged (pty master closed) -> robotd notices
    servos.stop = True
    time.sleep(0.1)
    os.close(rob_m)
    time.sleep(1.0)
    st = status(run_dir)
    check("unplug -> robot closed", st.get("robot", "") == "", str(st))

    d.send_signal(signal.SIGTERM)
    _, err = d.communicate(timeout=5)
    check("SIGTERM -> exit 0", d.returncode == 0, str(d.returncode))
    check("status file removed on exit", not os.path.exists(os.path.join(run_dir, "dusk_robotd.status")))
    print("--- dusk_robotd log ---")
    print("\n".join(l for l in err.splitlines() if "  " not in l[:3]))
    print(f"==== {'ALL PASS' if fails == 0 else str(fails) + ' FAIL'} ====")
    return 1 if fails else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        for p in list(PROCS):
            if p.poll() is None:
                p.kill()
