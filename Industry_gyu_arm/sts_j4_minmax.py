#!/usr/bin/env python3
"""j4(ID 4) 를 잰 한계의 끝으로 천천히 보내 보는 시험 (2026-10-29).

  ~/anaconda3/envs/gyu_arm/bin/python sts_j4_minmax.py

  1 → 최소(min) 로      2 → 최대(max) 로      3 → 가운데로
  0 → 힘 빼기 (토크 끔, 손으로 돌릴 수 있음)      q → 힘 빼고 끝내기   (Ctrl+C 도 힘 빼고 끝남)

· min·max 는 모터에 저장된 한계 (10-29 에 손으로 잰 범위에서 양쪽 20 안쪽) 를 그대로 읽어 쓴다.
· 다른 관절은 건드리지 않는다 (ID 4 만 힘을 준다). 팔이 처지지 않게 손으로 받쳐 주세요.
· 처음 힘을 줄 때 목표를 '지금 위치' 로 먼저 맞춰서 튀지 않게 한다. 속도는 느리게 (SPEED).
"""
import glob, sys, termios, time, tty
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler

ID = 4
SPEED = 250            # 느리게 (앱 기본 150~300 사이)
ACCEL = 8              # 실측으로 정한 값 (높이면 떨림)
TORQUE_LIMIT = 300
ADDR_MIN, ADDR_MAX, ADDR_TORQUE, ADDR_ACCEL, ADDR_GOAL, ADDR_SPEED, ADDR_TQ_LIMIT, ADDR_PRESENT = 9, 11, 40, 41, 42, 46, 48, 56

dev = sorted(glob.glob("/dev/serial/by-id/*1a86*"))
if not dev: print("로봇 어댑터가 안 보입니다."); sys.exit(1)
port = PortHandler(dev[0])
if not port.openPort(): print("포트를 못 열었습니다 — 다른 프로그램(앱·측정 도구)을 끄고 다시 하세요."); sys.exit(1)
port.setBaudRate(1000000)
pk = PacketHandler(0)

def rd(a, n=2):
    f = pk.read2ByteTxRx if n == 2 else pk.read1ByteTxRx
    for _ in range(5):
        try:
            v, r, _ = f(port, ID, a)
            if r == 0: return v
        except Exception: pass
        time.sleep(0.02)
    return None
def wr(a, v, n=2):
    f = pk.write2ByteTxRx if n == 2 else pk.write1ByteTxRx
    for _ in range(5):
        try:
            if f(port, ID, a, int(v))[0] == 0: return True
        except Exception: pass
        time.sleep(0.02)
    return False

lo, hi = rd(ADDR_MIN), rd(ADDR_MAX); pos = rd(ADDR_PRESENT)
if None in (lo, hi, pos): print("ID 4 응답이 없습니다."); sys.exit(1)
mode = rd(33, 1)
if mode != 0:
    print(f"모터가 위치 모드가 아닙니다 (모드 {mode}) — 위치 모드(0)로 바꿉니다")
    wr(33, 0, 1); time.sleep(0.05)
print(f"j4(ID 4) · 저장된 한계 min {lo} ~ max {hi} · 지금 위치 {pos} · 모드 {rd(33, 1)}")
print("1=최소  2=최대  3=가운데  0=힘 빼기  q=끝내기")

def torque_off():
    wr(ADDR_TORQUE, 0, 1)

def go(target, name):
    cur = rd(ADDR_PRESENT)
    if not rd(ADDR_TORQUE, 1):                     # 처음 힘 줄 때 튀지 않게: 목표 = 지금 위치 먼저
        wr(ADDR_TQ_LIMIT, TORQUE_LIMIT); wr(ADDR_ACCEL, ACCEL, 1); wr(ADDR_SPEED, SPEED)
        wr(ADDR_GOAL, cur); wr(ADDR_TORQUE, 1, 1); time.sleep(0.1)
    wr(ADDR_SPEED, SPEED); wr(ADDR_GOAL, target)
    print(f"\r→ {name} {target} 로 이동 (지금 {cur})            ")
    t0 = time.time()
    while time.time() - t0 < 8:
        p = rd(ADDR_PRESENT)
        sys.stdout.write(f"\r   위치 {p}   목표 {target}   차이 {None if p is None else p - target}      "); sys.stdout.flush()
        if p is not None and abs(p - target) <= 6: break
        time.sleep(0.05)
    print(f"\r   도착 위치 {rd(ADDR_PRESENT)} (목표 {target})                    ")

fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
try:
    tty.setcbreak(fd)
    while True:
        k = sys.stdin.read(1)
        if k == "1": go(lo, "최소")
        elif k == "2": go(hi, "최대")
        elif k == "3": go((lo + hi) // 2, "가운데")
        elif k == "0": torque_off(); print("\r힘 뺌 (손으로 돌릴 수 있음)                ")
        elif k in ("q", "Q"): break
except KeyboardInterrupt:
    pass
finally:
    torque_off()
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    port.closePort()
    print("\n힘 빼고 끝냈습니다.")
