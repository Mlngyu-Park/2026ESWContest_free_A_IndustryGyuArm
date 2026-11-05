#!/usr/bin/env python3
"""그리퍼 모터 번호 6 -> 5 (2026-10-29). j5 = ID 5 = 그리퍼 로 맞춘다.

  ~/anaconda3/envs/gyu_arm/bin/python sts_grip_id_6to5.py

· 토크는 켜지 않는다 (안 움직임). 번호 칸(5)만 바꾸고, 바꾼 뒤 다시 읽어 확인한다.
· ID 5 에 이미 다른 모터가 응답하면 (선 뺀 옛 모터를 다시 꽂은 경우 등) 바꾸지 않고 멈춘다.
· 앱·다른 모터 도구는 꺼 두세요 (포트).
"""
import glob, sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler

OLD, NEW = 6, 5
ADDR_ID, ADDR_MIN, ADDR_MAX, ADDR_TORQUE, ADDR_LOCK, ADDR_PRESENT = 5, 9, 11, 40, 55, 56

dev = sorted(glob.glob("/dev/serial/by-id/*1a86*"))
if not dev: print("로봇 어댑터가 안 보입니다."); sys.exit(1)
port = PortHandler(dev[0])
if not port.openPort(): print("포트를 못 열었습니다 — 앱·다른 도구를 끄고 다시 하세요."); sys.exit(1)
port.setBaudRate(1000000)
pk = PacketHandler(0)

def hits(i):
    n = 0
    for _ in range(3):
        try:
            _, r, _ = pk.read2ByteTxRx(port, i, ADDR_PRESENT); n += (r == 0)
        except Exception: pass
        time.sleep(0.01)
    return n

def show():
    for i in range(1, 8):
        try:
            v, r, _ = pk.read2ByteTxRx(port, i, ADDR_PRESENT)
        except Exception: continue
        if r != 0: continue
        lo, _, _ = pk.read2ByteTxRx(port, i, ADDR_MIN); hi, _, _ = pk.read2ByteTxRx(port, i, ADDR_MAX)
        lk, _, _ = pk.read1ByteTxRx(port, i, ADDR_LOCK); tq, _, _ = pk.read1ByteTxRx(port, i, ADDR_TORQUE)
        print(f"  ID{i}: 위치 {v}  한계 {lo}~{hi}  잠금 {lk}  토크 {tq}")

a, b = hits(NEW), hits(OLD)
print(f"바꾸기 전: ID{NEW} {a}/3 · ID{OLD} {b}/3"); show()
if a:
    print(f"ID{NEW} 에 이미 모터가 있습니다 — 바꾸지 않습니다."); port.closePort(); sys.exit(1)
if b == 0:
    print(f"ID{OLD} (그리퍼) 가 응답하지 않습니다 — 이미 바꿨거나 선을 확인하세요."); port.closePort(); sys.exit(1)

pk.write1ByteTxRx(port, OLD, ADDR_TORQUE, 0); time.sleep(0.03)    # 토크 끔
pk.write1ByteTxRx(port, OLD, ADDR_LOCK, 0); time.sleep(0.03)      # 잠금 풀기
pk.write1ByteTxRx(port, OLD, ADDR_ID, NEW); time.sleep(0.1)       # 번호 바꾸기
pk.write1ByteTxRx(port, NEW, ADDR_LOCK, 1); time.sleep(0.05)      # 잠금

a, b = hits(NEW), hits(OLD)
print(f"\n바꾼 뒤: ID{NEW} {a}/3 · ID{OLD} {b}/3"); show()
print("✓ 그리퍼 = ID 5" if a == 3 and b == 0 else "⚠ 확인 필요 — 이 화면을 알려 주세요.")
port.closePort()
