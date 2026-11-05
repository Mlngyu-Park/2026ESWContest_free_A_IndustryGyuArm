#!/usr/bin/env python3
"""관절↔모터 번호 확인 + 한계 재기 — 직접 돌리면서 터미널로 보는 판 (읽기만).

  ~/anaconda3/envs/gyu_arm/bin/python sts_live_limits.py        # 번호 1~6
  ~/anaconda3/envs/gyu_arm/bin/python sts_live_limits.py 2      # 2번 팔 (11~15, j1~j4 + 그리퍼)

· 모터에 아무것도 쓰지 않는다 (토크·한계·번호 그대로). 토크가 켜져 있으면 경고만 한다.
· 0.1초마다 표를 다시 그린다. 지금 움직이는 모터 옆에 '◀ 움직이는 중' 이 뜬다
  → 손으로 관절을 돌려 보면 그 관절이 몇 번 모터인지 바로 보인다.
· 번호마다 가장 작은·큰 값을 기록한다 (0/4095 를 넘어가도 이어서 센다).
· Ctrl+C 로 끝내면 결과를 보여 주고 sts_limits_measured_날짜.py 에 따로 저장한다.
  (지금 쓰는 sts_limits.py 는 덮어쓰지 않는다)
"""
import glob, os, sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler

ARM = int(sys.argv[1]) if len(sys.argv) > 1 else None
IDS = [(ARM - 1) * 10 + j for j in range(1, 6)] if ARM else [1, 2, 3, 4, 5, 6]
JN = {(ARM - 1) * 10 + j: n for j, n in zip(range(1, 6), ["j1", "j2", "j3", "j4", "그리퍼"])} if ARM else {}
ADDR_TORQUE, ADDR_PRESENT = 40, 56
MOVE_RAW = 8          # 이만큼 넘게 바뀌면 '움직이는 중'
HOLD_S = 0.6          # 멈춘 뒤에도 이 시간 동안은 표시를 남긴다

devs = sorted(glob.glob("/dev/serial/by-id/*1a86*"))
if not devs:                       # WSL 에서 by-id 가 안 생길 때
    for t in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
        try:
            if open(f"/sys/class/tty/{os.path.basename(t)}/device/../idVendor").read().strip() == "1a86": devs = [t]
        except Exception: pass
if not devs:
    print("로봇 어댑터가 안 보입니다 (/dev/serial/by-id/*1a86*). usbipd attach 를 확인하세요."); sys.exit(1)
port = PortHandler(devs[0])
if not port.openPort():
    print("포트를 못 열었습니다 — ARM STUDIO 가 켜져 있으면 끄고 다시 하세요."); sys.exit(1)
port.setBaudRate(1000000)
pk = PacketHandler(0)

def rd(i, addr=ADDR_PRESENT, n=2):
    f = pk.read2ByteTxRx if n == 2 else pk.read1ByteTxRx
    for _ in range(3):
        try:
            v, r, _ = f(port, i, addr)
            if r == 0: return v
        except Exception: pass
    return None

alive = [i for i in IDS if rd(i) is not None]
dead = [i for i in IDS if i not in alive]
torque_on = [i for i in alive if rd(i, ADDR_TORQUE, 1)]
print(f"어댑터 {devs[0]}")
print(f"응답하는 모터 {alive}" + (f" · 응답 없음 {dead}" if dead else ""))
if torque_on:
    print(f"⚠ 토크가 켜진 모터 {torque_on} — 손으로 안 돌아갑니다. 앱에서 토크를 끄고 다시 하세요.")
time.sleep(1.5)

last = {}; un = {}; mn = {}; mx = {}; ref = {}; moved_at = {i: 0.0 for i in alive}
for i in alive:
    v = rd(i); last[i] = un[i] = mn[i] = mx[i] = ref[i] = v

def draw():
    now = time.time()
    out = ["\033[H\033[J",   # 화면 지우고 맨 위로
           "관절을 하나씩 양쪽 끝까지 천천히 돌리세요.   끝내기: Ctrl+C\n",
           " 번호   관절      지금값   최소   최대   범위(도)",
           " ----  ------   ------  -----  -----  --------"]
    for i in alive:
        span = (mx[i] - mn[i]) * 360 / 4096
        mark = "   ◀ 움직이는 중" if now - moved_at[i] < HOLD_S else ""
        out.append(f"  {i:2d}   {JN.get(i, '-'):6s}   {un[i]:6d}  {mn[i]:5d}  {mx[i]:5d}  {span:7.1f}{mark}")
    if dead: out.append(f"\n 응답 없음: {dead}")
    sys.stdout.write("\n".join(out) + "\n"); sys.stdout.flush()

def finish():
    stamp = time.strftime("%Y%m%d_%H%M")
    print("\n================ 측정 결과 (raw) ================")
    print(" 번호   관절      최소    최대   범위(도)")
    for i in alive:
        print(f"  {i:2d}   {JN.get(i, '-'):6s}  {mn[i]:6d}  {mx[i]:6d}  {(mx[i]-mn[i])*360/4096:7.1f}"
              + ("   ※ 0/4095 를 넘어감" if mn[i] < 0 or mx[i] > 4095 else ""))
    path = f"sts_limits_measured_{'arm%d_' % ARM if ARM else ''}{stamp}.py"
    with open(path, "w") as f:
        f.write(f"# sts_live_limits.py 측정 ({time.strftime('%Y-%m-%d %H:%M')}) — 아직 적용 안 함\n")
        f.write("# 번호: (최소, 최대) raw. 어느 번호가 어느 관절인지는 측정하며 본 대로 적어 둘 것\n")
        f.write("MEASURED_RAW={\n" + "".join(f"    {i}: ({mn[i]}, {mx[i]}),\n" for i in alive) + "}\n")
    print(f"\n저장: {path}  (sts_limits.py 는 그대로)")

try:
    while True:
        for i in alive:
            v = rd(i)
            if v is None: continue
            d = v - last[i]
            if d > 2048: d -= 4096           # 0/4095 넘어감 보정
            elif d < -2048: d += 4096
            un[i] += d; last[i] = v
            if abs(un[i] - ref[i]) >= MOVE_RAW:      # 천천히 돌려도 잡히게 '마지막 표시 값' 과 비교
                moved_at[i] = time.time(); ref[i] = un[i]
            mn[i] = min(mn[i], un[i]); mx[i] = max(mx[i], un[i])
        draw()
        time.sleep(0.1)
except KeyboardInterrupt:
    finish()
finally:
    port.closePort()
