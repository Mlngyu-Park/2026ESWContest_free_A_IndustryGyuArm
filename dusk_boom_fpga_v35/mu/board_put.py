#!/usr/bin/env python3
"""board_put.py — 뮤에서 FPGA 보드로 파일 하나 보내기 (USB 시리얼 셸, 랜 없이).
   사용: board_put.py 로컬파일 보드경로      예) board_put.py rachel_tpu /mnt/debian/opt/dusk/rachel/rachel_tpu
   xz 압축 → base64 → 보드 셸의 'base64 -d' 로 받고 → 데비안 python3 lzma 로 풀고 → sha256 비교 → 제자리로.
   보내는 동안만 921600bps, 끝나면 115200 으로 되돌린다."""
import base64, glob, hashlib, lzma, sys, time
import os
import serial
BAUD = int(os.environ.get('PUT_BAUD', '921600'))
SLOW = float(os.environ.get('PUT_SLOW', '1.6'))

def port():
    p = glob.glob('/dev/serial/by-id/*CP2102N*')
    return p[0] if p else '/dev/ttyUSB0'

def until(s, mark, wait):
    buf = b''; t0 = time.time()
    while time.time() - t0 < wait:
        buf += s.read(4096)
        if mark.encode() in buf: return buf.decode('utf-8', 'replace')
    raise SystemExit('timeout waiting for ' + mark + '\n' + buf[-800:].decode('utf-8', 'replace'))

def main(src, dst):
    raw = open(src, 'rb').read()
    sha = hashlib.sha256(raw).hexdigest()
    b64 = base64.encodebytes(lzma.compress(raw, preset=9))      # 76 글자 줄
    s = serial.Serial(port(), 115200, timeout=0.05)
    s.write(b'\x03\r\n'); time.sleep(0.3); s.reset_input_buffer()
    s.write(f'stty -F /dev/ttyPS0 {BAUD}\r\n'.encode()); time.sleep(0.5)
    s.baudrate = BAUD; time.sleep(0.2); s.reset_input_buffer()
    s.write(b'echo READY_$((40+2))\r\n'); until(s, 'READY_42', 5)
    s.write(b'stty -echo; base64 -d > /tmp/_put.xz; stty echo; echo GOT_$((40+2))\r\n'); time.sleep(0.3)
    t0 = time.time(); n = len(b64); CH = 2048
    for i in range(0, n, CH):
        s.write(b64[i:i + CH]); s.flush()
        time.sleep(CH * 10 / BAUD * SLOW)          # 보드 쪽이 따라오게 조금 느리게
        if i % (CH * 64) == 0:
            sys.stderr.write(f'\r  {i * 100 // n}%  {time.time() - t0:.0f}s'); sys.stderr.flush()
    s.write(b'\x04'); until(s, 'GOT_42', 30)
    sys.stderr.write(f'\r  100%  {time.time() - t0:.0f}s ({n} bytes base64)\n')
    cmd = (f"ls -l /tmp/_put.xz; chroot /mnt/debian python3 -c 'import lzma,sys;sys.stdout.buffer.write(lzma.decompress(sys.stdin.buffer.read()))' < /tmp/_put.xz > '{dst}.part' && sha256sum '{dst}.part' | cut -c1-64; "
           f"rm -f /tmp/_put.xz; echo SUM_$((40+2))\r\n")
    s.write(cmd.encode()); out = until(s, 'SUM_42', 60)
    ok = sha in out
    if not ok: sys.stderr.write(f'  expect xz {len(lzma.compress(raw, preset=9))} bytes\n  board: {out[-600:]}\n')
    # 원래 파일의 권한을 따른다 (새 파일이면 실행파일처럼 보이는 것만 +x)
    mode = '755' if raw[:4] == b'\x7fELF' or raw[:2] == b'#!' else '644'
    keep = f"[ -e '{dst}' ] && chmod --reference='{dst}' '{dst}.part' 2>/dev/null || chmod {mode} '{dst}.part'; "
    s.write(((keep if ok else '') + f"{'mv' if ok else 'rm -f'} '{dst}.part' {chr(39) + dst + chr(39) if ok else ''}; "
             f"stty -F /dev/ttyPS0 115200\r\n").encode()); time.sleep(0.5)
    s.baudrate = 115200; s.close()
    print(f'{src} -> {dst} : {"OK" if ok else "SHA MISMATCH"} {sha[:16]}')
    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
