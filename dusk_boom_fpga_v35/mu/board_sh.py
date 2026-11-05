#!/usr/bin/env python3
"""board_sh.py — 뮤(라떼판다)에서 FPGA 보드 시리얼 셸(ttyPS0, CP2102N)에 명령 하나 보내고 출력 받기.
   사용: board_sh.py '명령' [기다릴초=10]      (보드 셸은 root, 표시줄 'dusk-boom:...#')
   출력은 시작·끝 표시 사이만 잘라서 보여준다. 포트를 다른 프로그램(tpu_bridge)이 쓰고 있으면
   그 프로그램의 /run/user 소켓으로 대신 보낸다."""
import glob, os, sys, time, uuid
import serial

def port():
    p = glob.glob('/dev/serial/by-id/*CP2102N*')
    return p[0] if p else '/dev/ttyUSB0'

def run(cmd, wait=10.0, baud=115200):
    tag = uuid.uuid4().hex[:8]
    s = serial.Serial(port(), baud, timeout=0.1)
    s.reset_input_buffer()
    s.write(b'\x03\r\n')            # 혹시 앞 명령이 멈춰 있으면 끊는다
    time.sleep(0.3); s.reset_input_buffer()
    line = f"echo B{tag}; ( {cmd} ) 2>&1; echo E{tag}\r\n"
    s.write(line.encode())
    buf = b''; t0 = time.time()
    while time.time() - t0 < wait:
        buf += s.read(4096)
        if f'\nE{tag}'.encode() in buf.replace(b'\r', b''):
            break
    s.close()
    t = buf.replace(b'\r', b'').decode('utf-8', 'replace')
    a = t.find(f'\nB{tag}\n'); b = t.find(f'\nE{tag}')
    if a < 0: return 'NO_START_MARK:\n' + t[-2000:]
    return t[a + len(tag) + 3: b if b > 0 else None]

if __name__ == '__main__':
    print(run(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 10.0), end='')
