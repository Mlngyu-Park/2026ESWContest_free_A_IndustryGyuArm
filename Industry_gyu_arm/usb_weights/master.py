#!/usr/bin/env python3
"""
노트북 = 마스터, Due = 계산 노드

가중치 9.4MB 를 노트북이 들고 있고, Due 에게 행렬곱만 시킨다.
DUSK 설계 그대로다 — "CPU 는 계산하지 않는다. 명령만 큐에 넣는다."

사용법:
    python3 master.py COM7 --ping          연결 확인
    python3 master.py COM7 --bench         MAC 커널 속도 측정
    python3 master.py COM7 --matvec        실제 가중치로 행렬곱 1회
    python3 master.py COM7 --usbspeed      USB 순수 속도 (계산 제외)
    python3 master.py COM7 --throughput    행렬곱 전체 속도
    python3 master.py COM7 --all           전부

    (리눅스면 /dev/ttyACM1 같은 것. Native USB 포트는 번호가 다르다)
"""
import sys, os, struct, time

HERE = os.path.dirname(os.path.abspath(__file__))

def find_weights():
    """가중치를 여러 곳에서 찾는다 (Windows/WSL 양쪽 대응)"""
    cands = [
        os.path.join(HERE, 'rachel_v9.bin'),                        # 같은 폴더
        os.path.join(HERE, '..', 'rachel_cpp', 'rachel_v9.bin'),    # 프로젝트 구조
        r'\\wsl$\Ubuntu\home\drboom\2026_project\Industry_gyu_arm\rachel_cpp\rachel_v9.bin',
    ]
    for c in cands:
        if os.path.exists(c):
            return c
    print("가중치 rachel_v9.bin 을 못 찾았습니다. 찾아본 곳:")
    for c in cands:
        print(f"   {c}")
    print("\nmaster.py 와 같은 폴더에 rachel_v9.bin 을 두세요.")
    sys.exit(1)

WEIGHTS = None   # main 에서 정한다


def open_port(name):
    try:
        import serial
    except ImportError:
        print("pyserial 이 필요합니다:  pip install pyserial")
        sys.exit(1)
    # Native USB(CDC)는 baudrate 를 무시하고 전속력으로 간다
    ser = serial.Serial(name, 115200, timeout=5)
    time.sleep(0.3)
    ser.reset_input_buffer()
    return ser



# ── 진행 표시 ─────────────────────────────────────────────
def bar(done, total, width=28):
    """[=====>    ] 45%  형태"""
    if total <= 0:
        return ''
    f = done / total
    n = int(f * width)
    return '[' + '=' * n + ('>' if n < width else '') + ' ' * (width - n - (1 if n < width else 0)) + f'] {f*100:3.0f}%'


def prog(msg, done, total, extra=''):
    """같은 줄에 덮어쓴다"""
    sys.stdout.write(f'\r  {msg} {bar(done, total)} {extra}   ')
    sys.stdout.flush()


def prog_end():
    sys.stdout.write('\n')
    sys.stdout.flush()


def send_rows_acked(ser, data, rows, cols, msg):
    """행 하나 보내고 Due 의 ACK('A') 를 기다린다.

    ★ 왜 필요한가 — 실측으로 확인한 것:
      160KB 를 한 번에 밀어넣으니 Due 가 받는 속도를 앞질러
      USB 버퍼가 넘치고 결과를 못 받았다 (128행에서 타임아웃).
      행마다 주고받으면 흐름이 맞는다."""
    t0 = time.time()
    for r in range(rows):
        ser.write(data[r * cols:(r + 1) * cols])
        ser.flush()
        ack = ser.read(1)
        if ack != b'A':
            prog_end()
            raise TimeoutError(f"{r}행에서 ACK 없음 (받은 것: {ack})")
        if (r & 7) == 7 or r == rows - 1:
            el = time.time() - t0
            sent = (r + 1) * cols
            rate = sent / 1024 / el if el > 0 else 0
            prog(msg, r + 1, rows, f'{sent/1024:6.0f} KB  {rate:6.0f} KB/s')
    prog_end()
    return time.time() - t0


def read_exact_prog(ser, n, msg):
    """받으면서 진행 표시"""
    buf = b''
    t0 = time.time()
    last = 0
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            prog_end()
            raise TimeoutError(f"{n}바이트 중 {len(buf)}만 받음")
        buf += chunk
        if time.time() - last > 0.1 or len(buf) == n:
            el = time.time() - t0
            rate = len(buf) / 1024 / el if el > 0 else 0
            prog(msg, len(buf), n, f'{len(buf)/1024:6.1f}/{n/1024:.1f} KB  {rate:6.0f} KB/s')
            last = time.time()
    prog_end()
    return buf


def read_exact(ser, n):
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise TimeoutError(f"{n}바이트 중 {len(buf)}만 받음")
        buf += chunk
    return buf


# ── 모델 읽기 ─────────────────────────────────────────────
class Model:
    def __init__(self, path):
        self.f = open(path, 'rb')
        self.size = os.path.getsize(path)
        magic = self.f.read(4)
        if magic != b'RCHL':
            raise ValueError(f"RCHL 이 아님: {magic}")
        v = struct.unpack('<6i', self.f.read(24))
        self.ver, self.d, self.L, self.H, self.V, self.ctx = v

    def raw(self, off, n):
        self.f.seek(off)
        return self.f.read(n)


# ── 명령들 ────────────────────────────────────────────────
def cmd_ping(ser):
    print("[핑]")
    t0 = time.time()
    ser.write(b'P')
    r = read_exact(ser, 1)
    el = (time.time() - t0) * 1000
    print(f"  응답 {r}  왕복 {el:.1f} ms")
    return r == b'K'


def cmd_bench(ser, m):
    print("\n[MAC 커널 속도 — Due 에서 측정]")
    cols, rep = 320, 2000
    w = m.raw(m.size - 2_000_000, cols)
    import random
    random.seed(42)
    x = [random.randint(-32000, 32000) for _ in range(cols)]

    # PC 정답
    expect = sum(struct.unpack(f'{cols}b', w)[i] * x[i] for i in range(cols))
    expect = max(-2147483648, min(2147483647, expect))

    print(f"  {rep}회 x {cols} MAC 을 Due 에서 측정 중... (몇 초 걸립니다)")
    ser.write(b'B' + struct.pack('<HH', cols, rep))
    ser.write(struct.pack(f'<{cols}h', *x))
    ser.write(w)
    out = read_exact(ser, 16)
    us1, us2, r1, r2 = struct.unpack('<IIii', out)

    for name, us, r in (("v1 (매항 int64)", us1, r1), ("v2 (16개마다)", us2, r2)):
        mac_s = rep * cols / (us * 1e-6)
        cyc = 84e6 / mac_s
        ok = "OK" if r == expect else f"다름! ({r} vs {expect})"
        print(f"  {name:<18}{mac_s/1e6:>7.2f} M MAC/s  {cyc:>5.1f}사이클  {ok}")

    best = max(rep * cols / (us1 * 1e-6), rep * cols / (us2 * 1e-6))
    sent = 9830400 * 130 / best
    print(f"\n  최고 속도로 RACHEL 한 문장: {sent/60:.1f}분 (계산만)")
    return best


def cmd_throughput(ser, m):
    print("\n[USB 전송 속도]")
    cols = 1280
    rows_list = [8, 32, 128]
    x = struct.pack(f'<{cols}h', *([1000] * cols))

    for rows in rows_list:
        w = m.raw(1024, rows * cols)
        nbytes = 4 + len(x) + len(w) + rows * 4
        t0 = time.time()
        ser.write(b'M' + struct.pack('<HH', rows, cols))
        ser.write(x)
        send_rows_acked(ser, w, rows, cols, f'{rows:>3}행 전송')
        _ = read_exact(ser, rows * 4)
        el = time.time() - t0
        print(f"    → {rows:>3}행 x {cols}열: {nbytes/1024:>6.0f} KB  "
              f"{el*1000:>7.1f} ms  {nbytes/1024/el:>7.0f} KB/s\n")


def cmd_usbspeed(ser, m):
    """계산 없이 순수 전송 속도만 잰다 — USB 가 병목인지 확인"""
    print("\n[USB 순수 속도 — Due 는 받기만 하고 버린다]")
    for kb in (64, 256):
        total = kb * 1024
        data = m.raw(1024, total)
        ser.write(b'S' + struct.pack('<I', total))
        t0 = time.time()
        sent = 0
        CH = 8192
        while sent < total:
            n = ser.write(data[sent:sent + CH])
            sent += n if n else CH
            if (sent // CH) % 4 == 0 or sent >= total:
                el = time.time() - t0
                prog('  전송', sent, total,
                     f'{sent/1024:5.0f}/{kb} KB  {sent/1024/max(el,1e-9):6.0f} KB/s')
        ser.flush()
        prog_end()
        got, us = struct.unpack('<II', read_exact(ser, 8))
        pc_el = time.time() - t0
        print(f"    → {kb} KB: PC 기준 {pc_el*1000:7.1f} ms  {kb/pc_el:6.0f} KB/s")
        print(f"       Due 기준 {us/1000:7.1f} ms  {got/1024/(us*1e-6):6.0f} KB/s  "
              f"(받은 양 {got/1024:.0f} KB)\n")


def cmd_matvec(ser, m):
    print("\n[실제 가중치로 행렬곱 — PC 와 대조]")
    d = m.d
    rows, cols = 64, d
    # 헤더(28) + 텐서 목록을 건너뛴 지점에서 대충 뽑는다 (검증 목적)
    off = m.size - 3_000_000
    w = m.raw(off, rows * cols)
    import random
    random.seed(7)
    x = [random.randint(-20000, 20000) for _ in range(cols)]

    t0 = time.time()
    ser.write(b'M' + struct.pack('<HH', rows, cols))
    ser.write(struct.pack(f'<{cols}h', *x))
    send_rows_acked(ser, w, rows, cols, '가중치 전송')
    got = struct.unpack(f'<{rows}i', read_exact_prog(ser, rows * 4, '결과 수신'))
    el = time.time() - t0

    ws = struct.unpack(f'{rows*cols}b', w)
    bad = 0
    for r in range(rows):
        acc = sum(ws[r*cols + i] * x[i] for i in range(cols))
        acc = max(-2147483648, min(2147483647, acc))
        if acc != got[r]:
            if bad < 3:
                prog_end()
                print(f"  행 {r}: Due={got[r]}  PC={acc}  <<< 다름")
            bad += 1
        if (r & 7) == 7 or r == rows - 1:
            prog('PC 대조   ', r + 1, rows, f'{r+1}/{rows}행  불일치 {bad}')
    prog_end()

    print(f"  {rows}행 x {cols}열, {el*1000:.1f} ms")
    if bad == 0:
        print(f"  *** {rows}행 전부 비트 단위 일치 ***")
    else:
        print(f"  !!! {bad}행 불일치 !!!")
    return bad == 0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    port = sys.argv[1]
    args = sys.argv[2:] or ['--all']

    m = Model(find_weights())
    print(f"모델 v{m.ver}: d={m.d} L={m.L} H={m.H} V={m.V} ctx={m.ctx}")
    print(f"  {m.size:,} 바이트 ({m.size/1048576:.2f} MB)\n")

    ser = open_port(port)
    print(f"포트 {port} 열림\n")

    try:
        do_all = '--all' in args
        if do_all or '--ping' in args:
            if not cmd_ping(ser):
                print("  응답이 없습니다. Native USB 포트인지, 보드 설정이 맞는지 확인하세요.")
                return
        if do_all or '--bench' in args:
            cmd_bench(ser, m)
        if do_all or '--matvec' in args:
            cmd_matvec(ser, m)
        if do_all or '--usbspeed' in args:
            cmd_usbspeed(ser, m)
        if do_all or '--throughput' in args:
            cmd_throughput(ser, m)
    finally:
        ser.close()


if __name__ == '__main__':
    main()
