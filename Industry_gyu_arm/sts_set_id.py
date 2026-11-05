#!/usr/bin/env python3
"""모터 번호 하나 바꾸기 — 2번 팔 번호 정리용 (2026-10-29).

  ~/anaconda3/envs/gyu_arm/bin/python sts_set_id.py 1 11      # 1번 -> 11번

순서
  ① 선에 붙은 번호를 모두 훑어 보여 준다 (정상 / 번호 겹침)
  ② 바꿀 번호가 '하나만' 대답하는지, 새 번호가 비어 있는지 확인 — 아니면 멈춘다
  ③ 그 모터 위치를 계속 보여 준다 → 관절을 손으로 돌려 숫자가 바뀌는지 보고 어느 관절인지 확인
  ④ y 를 누르면 번호만 바꾸고 다시 읽어 확인한다 (토크는 켜지 않는다 — 안 움직임)
앱·다른 모터 도구는 꺼 두세요 (포트).
"""
import glob, os, select, sys, time
from collections import Counter
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler

ADDR_ID, ADDR_MIN, ADDR_MAX, ADDR_TORQUE, ADDR_LOCK, ADDR_PRESENT = 5, 9, 11, 40, 55, 56
OK, TIMEOUT = 0, -6

if len(sys.argv) != 3: print(__doc__); sys.exit(1)
OLD, NEW = int(sys.argv[1]), int(sys.argv[2])
assert 0 <= OLD <= 253 and 0 <= NEW <= 253 and OLD != NEW

def find_port():
    d = sorted(glob.glob("/dev/serial/by-id/*1a86*"))
    if d: return d[0]
    for t in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):   # WSL 에서 by-id 가 안 생길 때
        try:
            if open(f"/sys/class/tty/{os.path.basename(t)}/device/../idVendor").read().strip() == "1a86": return t
        except Exception: pass
    return None

dev = find_port()
if not dev: print("로봇 어댑터가 안 보입니다 — usbipd attach 를 확인하세요."); sys.exit(1)
port = PortHandler(dev)
if not port.openPort(): print("포트를 못 열었습니다 — 앱·다른 도구를 끄고 다시 하세요."); sys.exit(1)
port.setBaudRate(1000000)
pk = PacketHandler(0)

def pings(i, n=10):
    c = Counter()
    for _ in range(n):
        try: _, r, _ = pk.ping(port, i)
        except Exception: r = "예외"
        c[r] += 1; time.sleep(0.005)
    return c

def rd(i, a):
    for _ in range(3):
        try:
            v, r, _ = pk.read2ByteTxRx(port, i, a)
            if r == OK: return v
        except Exception: pass
        time.sleep(0.01)
    return None

def scan():
    ok, clash = [], []
    for i in range(0, 32):
        c = pings(i, 3)
        if c[OK] == 3: ok.append(i)
        elif set(c) != {TIMEOUT}: clash.append(i)
    print(f"  정상: {ok}" + (f"   ⚠ 번호 겹침(여러 모터가 같은 번호): {clash}" if clash else ""))
    return ok, clash

print(f"어댑터 {dev}\n① 선에 붙은 번호 (0~31)")
scan()

# ② 확인
c = pings(OLD)
if c[OK] != 10:
    if set(c) == {TIMEOUT}: print(f"\nID{OLD} 이 대답하지 않습니다.")
    else: print(f"\nID{OLD} 은 모터 여러 개가 같은 번호입니다 ({dict(c)}).\n"
                f"  → 뒤쪽 선을 빼서 ID{OLD} 모터가 하나만 남게 한 뒤 다시 실행하세요.")
    port.closePort(); sys.exit(1)
if set(pings(NEW)) != {TIMEOUT}:
    print(f"\nID{NEW} 에 이미 모터가 있습니다 — 다른 번호를 고르세요."); port.closePort(); sys.exit(1)

# ③ 어느 관절인지 눈으로 확인
print(f"\n③ ID{OLD} 위치를 보여 줍니다. 관절을 하나씩 손으로 돌려 보세요 — 숫자가 바뀌는 관절이 이 모터입니다.")
print(f"   (한계 {rd(OLD, ADDR_MIN)}~{rd(OLD, ADDR_MAX)})   확인했으면 Enter")
base = rd(OLD, ADDR_PRESENT)
while True:
    v = rd(OLD, ADDR_PRESENT)
    mark = "  ◀ 움직임" if (v is not None and base is not None and abs(v - base) > 15) else ""
    sys.stdout.write(f"\r   ID{OLD} 위치 {v}{mark}            "); sys.stdout.flush()
    if select.select([sys.stdin], [], [], 0.1)[0]:
        sys.stdin.readline(); break

# ④ 바꾸기
if input(f"\nID{OLD} -> ID{NEW} 로 바꿀까요? (y/n) ").strip().lower() != "y":
    print("바꾸지 않았습니다."); port.closePort(); sys.exit(0)
pk.write1ByteTxRx(port, OLD, ADDR_TORQUE, 0); time.sleep(0.03)    # 토크 끔
pk.write1ByteTxRx(port, OLD, ADDR_LOCK, 0); time.sleep(0.03)      # 잠금 풀기
pk.write1ByteTxRx(port, OLD, ADDR_ID, NEW); time.sleep(0.1)       # 번호 바꾸기
pk.write1ByteTxRx(port, NEW, ADDR_LOCK, 1); time.sleep(0.05)      # 잠금
a, b = pings(NEW), pings(OLD)
print(f"\n바꾼 뒤: ID{NEW} 정상 {a[OK]}/10 · ID{OLD} 정상 {b[OK]}/10")
print(f"✓ ID{NEW} 위치 {rd(NEW, ADDR_PRESENT)}" if a[OK] == 10 and b[OK] == 0 else "⚠ 확인 필요 — 이 화면을 알려 주세요.")
print("\n지금 선에 붙은 번호"); scan()
port.closePort()
