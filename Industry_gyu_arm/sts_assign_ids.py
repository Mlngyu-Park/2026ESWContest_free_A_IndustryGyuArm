#!/usr/bin/env python3
"""손으로 돌려서 관절 번호 붙이기 → 모터 번호 정리 (2026-10-29).

  ~/anaconda3/envs/gyu_arm/bin/python sts_assign_ids.py 2      # 2번 팔: j1~j4 = 11~14, 그리퍼 = 15
  ~/anaconda3/envs/gyu_arm/bin/python sts_assign_ids.py 1      # 1번 팔: 1~5

· 시작하면 모든 모터 힘을 뺀다 (안 움직임).
· 응답하는 모터마다 한 줄씩 위치를 계속 보여 준다.
· 관절을 손으로 천천히 돌리면 그 줄에 '◀ 움직임' 이 뜬다 → 그때 키보드로 관절 번호(1~5)를 누른다.
    1~4 = j1~j4, 5 = 그리퍼(j5)        0 = 방금 붙인 번호 지우기
· 다 붙였으면 Enter → 바꿀 목록을 보여 주고 y 를 누르면 번호를 바꾼다.   q = 바꾸지 않고 끝내기
· 같은 번호를 쓰는 모터가 여러 개면 (공장 기본 1번 등) 대답이 겹쳐 값을 못 읽는다.
  그 줄은 '겹침' 으로 표시된다 — 선을 빼서 그 번호 모터가 하나만 남게 한 뒤 다시 실행.
앱·다른 모터 도구는 꺼 두세요 (포트).
"""
import glob, json, os, select, sys, termios, time, tty
from collections import Counter
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler

ADDR_ID, ADDR_TORQUE, ADDR_LOCK, ADDR_PRESENT = 5, 40, 55, 56
OK, TIMEOUT, BROADCAST = 0, -6, 0xFE
NJ = 5                      # j1~j4 + 그리퍼
JNAME = {1: "j1", 2: "j2", 3: "j3", 4: "j4", 5: "j5 그리퍼"}
MOVE_RAW = 15               # 이만큼 넘게 바뀌면 '움직임'
HOLD_S = 1.5                # 멈춘 뒤에도 이 시간 동안 표시를 남긴다 (번호 누를 시간)
TEMP0 = 200                 # 번호를 바꿀 때 거쳐 가는 임시 번호 (서로 바뀌는 경우 대비)

if len(sys.argv) != 2 or sys.argv[1] not in ("1", "2", "3", "4"): print(__doc__); sys.exit(1)
ARM = int(sys.argv[1])
def final_id(j): return (ARM - 1) * 10 + j       # 1번 팔 1~5, 2번 팔 11~15, ...

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

def ping_n(i, n):
    c = Counter()
    for _ in range(n):
        try: _, r, _ = pk.ping(port, i)
        except Exception: r = "예외"
        c[r] += 1; time.sleep(0.003)
    return c
def rd(i):
    try:
        v, r, _ = pk.read2ByteTxRx(port, i, ADDR_PRESENT)
        return v if r == OK else None
    except Exception: return None
def scan():
    ok, clash = [], []
    for i in range(0, 254):
        c = ping_n(i, 2)
        if c[OK] == 2: ok.append(i)
        elif set(c) != {TIMEOUT}: clash.append(i)
    return ok, clash

print(f"어댑터 {dev} · {ARM}번 팔 → 새 번호 " + ", ".join(f"{JNAME[j]}={final_id(j)}" for j in range(1, NJ + 1)))
pk.write1ByteTxOnly(port, BROADCAST, ADDR_TORQUE, 0); time.sleep(0.05)     # 모든 모터 힘 빼기 (겹친 번호 포함)
print("모든 모터 힘을 뺐습니다. 번호를 훑는 중...")
ids, clash = scan()
if not ids and not clash: print("대답하는 모터가 없습니다."); port.closePort(); sys.exit(1)

last = {i: rd(i) for i in ids}; ref = dict(last); moved_at = {i: 0.0 for i in ids}
assign = {}                 # 모터 번호 -> 관절 번호
msg = "관절을 하나씩 손으로 천천히 돌리고, '◀ 움직임' 이 뜨면 관절 번호(1~5)를 누르세요."

def latest_mover():
    now = time.time()
    c = [(moved_at[i], i) for i in ids if now - moved_at[i] < HOLD_S]
    return max(c)[1] if c else None

def draw():
    now = time.time(); mv = latest_mover()
    out = ["\033[H\033[J",
           f"{ARM}번 팔 번호 붙이기   1~4=j1~j4  5=그리퍼  0=지우기  Enter=끝내고 바꾸기  q=그만\n",
           "  지금 번호   위치    관절          새 번호",
           "  --------  ------  ------------  -------"]
    for i in ids:
        j = assign.get(i)
        mark = "  ◀ 움직임" if now - moved_at[i] < HOLD_S else ""
        if i == mv and mark: mark = "  ◀◀ 움직임 (지금 누르면 여기)"
        out.append(f"  ID{i:<6d}  {str(last[i]):>6s}  {(JNAME[j] if j else '-'):12s}  "
                   f"{(final_id(j) if j else ''):>7}{mark}")
    for i in clash:
        out.append(f"  ID{i:<6d}  ⚠ 겹침 — 같은 번호 모터가 여러 개라 값을 못 읽음 (선을 빼서 하나만 남기기)")
    out.append(f"\n  {msg}")
    sys.stdout.write("\n".join(out) + "\n"); sys.stdout.flush()

fd = sys.stdin.fileno(); old = termios.tcgetattr(fd); go = False
try:
    tty.setcbreak(fd)
    while True:
        for i in ids:
            v = rd(i)
            if v is None: continue
            last[i] = v
            if ref[i] is None: ref[i] = v
            d = v - ref[i]
            if d > 2048: d -= 4096
            elif d < -2048: d += 4096
            if abs(d) >= MOVE_RAW: moved_at[i] = time.time(); ref[i] = v
        draw()
        if select.select([sys.stdin], [], [], 0.08)[0]:
            k = sys.stdin.read(1)
            if k in ("q", "Q"): break
            if k in ("\n", "\r"): go = True; break
            if k.isdigit():
                n = int(k); i = latest_mover()
                if i is None: msg = "먼저 관절을 돌리세요 — '◀ 움직임' 이 뜬 상태에서 번호를 누릅니다."
                elif n == 0: assign.pop(i, None); msg = f"ID{i} 의 관절 번호를 지웠습니다."
                elif 1 <= n <= NJ:
                    prev = [o for o, j in assign.items() if j == n and o != i]
                    for o in prev: assign.pop(o)
                    msg = (f"{JNAME[n]} 은 ID{prev[0]} 에서 ID{i} 로 옮겼습니다." if prev
                           else f"ID{i} = {JNAME[n]}  (새 번호 {final_id(n)})")
                    assign[i] = n
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)

if not go or not assign:
    print("\n바꾸지 않았습니다."); port.closePort(); sys.exit(0)

plan = [(i, assign[i], final_id(assign[i])) for i in sorted(assign)]
print("\n바꿀 목록")
for i, j, f in plan: print(f"  ID{i} ({JNAME[j]}) -> ID{f}" + ("   (그대로)" if i == f else ""))
miss = [JNAME[j] for j in range(1, NJ + 1) if j not in assign.values()]
if miss: print(f"  ※ 번호를 안 붙인 관절: {', '.join(miss)}")
# 새 번호가 이번에 안 바꾸는 다른 모터와 겹치면 멈춘다
busy = (set(ids) - set(assign)) | set(clash)
bad = [f for _, _, f in plan if f in busy]
if bad:
    print(f"새 번호 {bad} 를 이미 다른 모터가 쓰고 있습니다 — 바꾸지 않습니다."); port.closePort(); sys.exit(1)
if input("\n이대로 번호를 바꿀까요? (y/n) ").strip().lower() != "y":
    print("바꾸지 않았습니다."); port.closePort(); sys.exit(0)

def set_id(a, b):
    pk.write1ByteTxRx(port, a, ADDR_LOCK, 0); time.sleep(0.03)
    pk.write1ByteTxRx(port, a, ADDR_ID, b); time.sleep(0.1)
    pk.write1ByteTxRx(port, b, ADDR_LOCK, 1); time.sleep(0.05)
    return ping_n(b, 5)[OK] == 5 and ping_n(a, 3)[OK] == 0

todo = [(i, j, f) for i, j, f in plan if i != f]
for i, j, f in todo:                                  # ① 임시 번호로 (서로 맞바뀌는 경우에도 안 겹치게)
    if not set_id(i, TEMP0 + j): print(f"⚠ ID{i} -> 임시 {TEMP0 + j} 실패 — 여기서 멈춥니다. 이 화면을 알려 주세요."); sys.exit(1)
for i, j, f in todo:                                  # ② 임시 번호 -> 새 번호
    if not set_id(TEMP0 + j, f): print(f"⚠ 임시 {TEMP0 + j} -> ID{f} 실패 — 여기서 멈춥니다. 이 화면을 알려 주세요."); sys.exit(1)

ok, clash2 = scan()
print("\n바꾼 뒤 대답하는 번호:", ok, (f"   ⚠ 겹침: {clash2}" if clash2 else ""))
for i, j, f in plan: print(f"  {JNAME[j]:10s} ID{f}  " + ("✓" if f in ok else "⚠ 응답 없음"))
rec = dict(arm=ARM, time=time.strftime("%Y-%m-%d %H:%M"), changed=[dict(old=i, joint=j, new=f) for i, j, f in plan],
           answering=ok, clash=clash2)
path = f"sts_assign_arm{ARM}_{time.strftime('%Y%m%d_%H%M')}.json"
json.dump(rec, open(path, "w"), ensure_ascii=False, indent=1)
print(f"기록: {path}")
port.closePort()
