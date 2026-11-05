#!/usr/bin/env python3
"""관절↔모터 번호 확인 + 양쪽 끝 값 재기 (읽기만 — 토크·설정은 절대 쓰지 않는다).

조립하다 모터가 바뀌었을 때 쓴다. 정해진 시간 동안 모든 모터 위치를 계속 기록한다.
사람이 관절을 하나씩 (사이에 몇 초 쉬며) 양쪽 끝까지 돌리면, 기록에서
  ① 언제 어느 번호가 움직였는지 → 관절↔번호
  ② 번호마다 가장 작은·큰 값 (0/4095 넘어감 보정) → 한계
를 뽑는다.

  python sts_watch_ids.py 360          # 360초 기록 -> sts_watch_log.jsonl + 요약 출력
"""
import glob, json, sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 360
IDS = [1, 2, 3, 4, 5, 6]
OUT = "sts_watch_log.jsonl"

dev = sorted(glob.glob("/dev/serial/by-id/*1a86*"))[0]
port = PortHandler(dev); assert port.openPort(); port.setBaudRate(1000000)
pk = PacketHandler(0)

def rd(i):
    v, r, _ = pk.read2ByteTxRx(port, i, 56)
    return v if r == 0 else None

alive = [i for i in IDS if rd(i) is not None]
print(f"기록 시작 ({SECS:.0f}초) · 응답하는 모터 {alive}", flush=True)
last = {}; un = {}; mn = {}; mx = {}
for i in alive:
    v = rd(i); last[i] = v; un[i] = v; mn[i] = v; mx[i] = v
t0 = time.time()
with open(OUT, "w") as f:
    while time.time() - t0 < SECS:
        row = {"t": round(time.time() - t0, 2)}
        for i in alive:
            v = rd(i)
            if v is None: continue
            d = v - last[i]
            if d > 2048: d -= 4096          # 0/4095 넘어감
            elif d < -2048: d += 4096
            un[i] += d; last[i] = v
            mn[i] = min(mn[i], un[i]); mx[i] = max(mx[i], un[i])
            row[i] = un[i]
        f.write(json.dumps(row) + "\n")
        time.sleep(0.05)
port.closePort()

# 요약: 번호마다 움직인 구간 (기준값에서 40 넘게 벗어난 시간)
rows = [json.loads(l) for l in open(OUT)]
print("\n번호  처음값  최소   최대   범위(도)  움직인 시간대(초)")
for i in alive:
    k = str(i); base = rows[0].get(k)
    moving = [r["t"] for r in rows if k in r and abs(r[k] - base) > 40]
    span = (mx[i] - mn[i]) * 360 / 4096
    win = f"{moving[0]:.0f}~{moving[-1]:.0f}" if moving else "안 움직임"
    print(f"{i:3d}  {base:6d} {mn[i]:6d} {mx[i]:6d}  {span:7.1f}   {win}")
