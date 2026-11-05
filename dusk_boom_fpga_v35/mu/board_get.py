#!/usr/bin/env python3
"""board_get.py — FPGA 보드 파일 하나를 뮤로 가져오기 (USB 시리얼 셸, base64). 사용: board_get.py 보드경로 로컬파일"""
import base64, glob, hashlib, re, sys, time
import serial

def port():
    p = glob.glob('/dev/serial/by-id/*CP2102N*')
    return p[0] if p else '/dev/ttyUSB0'

def main(src, dst, baud=921600):
    s = serial.Serial(port(), 115200, timeout=0.05)
    s.write(b'\r\n'); time.sleep(0.2); s.reset_input_buffer()
    s.write(f'stty -F /dev/ttyPS0 {baud}\r\n'.encode()); time.sleep(0.5); s.baudrate = baud
    time.sleep(0.2); s.reset_input_buffer()
    s.write(f"stty -echo; echo B_$((1+1)); base64 '{src}'; echo E_$((1+1)); sha256sum '{src}' | cut -c1-64; echo F_$((1+1)); stty echo\r\n".encode())
    buf = b''; t0 = time.time()
    while time.time() - t0 < 120 and b'F_2' not in buf:
        buf += s.read(65536)
    s.write(b'stty -F /dev/ttyPS0 115200\r\n'); time.sleep(0.4); s.baudrate = 115200; s.close()
    t = buf.replace(b'\r', b'').decode('ascii', 'replace')
    body = t[t.index('B_2\n') + 4: t.index('E_2')]
    data = base64.b64decode(re.sub(r'[^A-Za-z0-9+/=]', '', body))
    sha = t[t.index('E_2') + 4:].strip()[:64]
    open(dst, 'wb').write(data)
    ok = hashlib.sha256(data).hexdigest() == sha
    print(f'{src} -> {dst} {len(data)} bytes {"OK" if ok else "SHA MISMATCH"}')

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
