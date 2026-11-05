#!/usr/bin/env python3
"""j4(ID 4) 영점·도는 방향 잡기 — 다른 관절 보정은 건드리지 않는다 (2026-10-29).

원리 (물컵 규칙): 그리퍼가 바닥과 수평이면  j2 + j3 + j4 = 0  → j4 = -(j2 + j3).
어깨(j2)·팔꿈치(j3) 각도는 기존 영점으로 안다. 자세 두 개에서 그리퍼를 수평으로 맞추면
  · 두 자세 사이 손목 raw 변화의 부호 → 도는 방향 (DIR)
  · 그 방향으로 거꾸로 풀면 → 손목 영점 (raw)
읽기만 한다 (토크 꺼진 채 손으로 잡는다). 마지막에 물어보고 sts_zero.py 의 ID 4(j4) 만 바꾼다.

  ~/anaconda3/envs/gyu_arm/bin/python sts_wrist_zero.py
"""
import glob, math, shutil, statistics as st, sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler
from sts_zero import ZERO_RAW

DIR = {2: -1, 3: -1}            # arm_studio_web.py 와 같은 값 (어깨·팔꿈치)
WRIST = 4                       # j4 (10-29 번호 5 -> 4)
K = 2048 / math.pi              # 라디안 -> raw

dev = sorted(glob.glob("/dev/serial/by-id/*1a86*"))[0]
port = PortHandler(dev); assert port.openPort(); port.setBaudRate(1000000)
pk = PacketHandler(0)

def rd(i):
    vals = []
    for _ in range(15):
        try:
            v, r, _ = pk.read2ByteTxRx(port, i, 56)
            if r == 0: vals.append(v)
        except Exception: pass
        time.sleep(0.01)
    return st.median(vals) if vals else None

def ang(i, raw): return DIR[i] * (raw - ZERO_RAW[i]) / K      # 라디안

def capture(n):
    input(f"\n[자세 {n}] 그리퍼를 바닥과 수평으로 맞춰 잡고 Enter ")
    r2, r3, r5 = rd(2), rd(3), rd(WRIST)
    j2, j3 = ang(2, r2), ang(3, r3)
    j4 = -(j2 + j3)
    print(f"  어깨 {math.degrees(j2):+6.1f}°  팔꿈치 {math.degrees(j3):+6.1f}°  "
          f"→ 수평이려면 손목 {math.degrees(j4):+6.1f}°  · 손목 raw {r5:.0f}")
    return j4, r5

print("손목 영점 잡기 — 토크는 꺼진 채로, 손으로 잡고 하세요.")
print("자세 두 개를 팔꿈치를 '많이 다르게' 굽혀서 잡을수록 정확합니다 (예: 펴고 한 번, 많이 접고 한 번).")
a = capture(1); b = capture(2)
d_raw = b[1] - a[1]; d_ang = b[0] - a[0]
if abs(math.degrees(d_ang)) < 20:
    print(f"\n두 자세의 손목 각도 차이가 {math.degrees(d_ang):.1f}° 뿐입니다. 팔꿈치를 더 다르게 굽혀 다시 하세요."); sys.exit(1)
dirw = 1 if d_raw * d_ang > 0 else -1
ratio = abs(d_raw) / (abs(d_ang) * K)
z1 = a[1] - dirw * a[0] * K; z2 = b[1] - dirw * b[0] * K
zero = round((z1 + z2) / 2)
print(f"\n도는 방향 DIR = {dirw:+d}   (raw 변화/각도 변화 비율 {ratio:.2f} — 1 에 가까울수록 수평을 잘 맞춘 것)")
print(f"손목 영점 = {zero}   (자세 1 로 풀면 {z1:.0f}, 자세 2 로 풀면 {z2:.0f} — 차이 {abs(z1-z2):.0f} raw = {abs(z1-z2)/K*180/math.pi:.1f}°)")
if not (0.8 < ratio < 1.25):
    print("⚠ 비율이 1 에서 많이 벗어났습니다 — 수평을 다시 맞춰 한 번 더 하세요. 저장하지 않습니다."); sys.exit(1)
if input("\nsts_zero.py 의 ID 4(j4) 영점을 이 값으로 바꿀까요? (y/n) ").strip().lower() != "y":
    print("바꾸지 않았습니다."); sys.exit(0)
shutil.copy2("sts_zero.py", "sts_zero.py.bak_wrist1029")
src = open("sts_zero.py").read()
new = dict(ZERO_RAW); new[WRIST] = zero
import re
src = re.sub(r"ZERO_RAW=\{[^}]*\}", "ZERO_RAW=" + repr(new), src)
src = src.rstrip() + f"\n# 2026-10-29 j4(ID 4) 영점만 sts_wrist_zero.py 로 다시 잡음 (DIR {dirw:+d})\n"
open("sts_zero.py", "w").write(src)
print(f"저장: sts_zero.py (ID 4 = {zero}) · 원래 파일 sts_zero.py.bak_wrist1029")
if dirw != -1:
    print("※ 도는 방향이 예전 손목(-1)과 반대입니다 — 앱의 DIR 도 바꿔야 합니다. 이 결과를 알려 주세요.")
port.closePort()
