#!/usr/bin/env python3
"""2번 팔(ID 11~15) 한계·위치 보정 적용 — 10-29 에 잰 값으로 (2026-10-30 준비, sts_apply_1029.py 와 같은 방식).

  ~/anaconda3/envs/gyu_arm/bin/python sts_apply_arm2.py

· 토크는 켜지 않는다 (안 움직임). 바꾸기 전 모터 값(위치 보정·한계)을 먼저 파일로 저장한다.
· 잰 범위가 0/4095 경계 근처(50 안쪽)를 넘으면 위치 보정값을 옮겨 범위 가운데가 2048 쯤 오게 한다
  (써 보고 다시 읽어 확인, 틀리면 원래 값으로 되돌리고 멈춘다).
· 모든 모터에 한계를 양쪽 MARGIN 만큼 안쪽으로 넣는다.
· 1번 팔(ID 1~5)이 같은 선에 꽂혀 있어도 11~15 만 건드린다.
결과: sts_eeprom_backup_arm2.json (원래 값) · sts_apply_arm2_result.json (새 값)

잰 값 (sts_live_limits.py 2, 10-29 19:46 · 0/4095 넘어감을 이어서 셈):
  11 j1  -396 ~ 2426   (248.0°)  ← 경계를 넘음
  12 j2  3919 ~ 6391   (217.3°)  ← 경계를 넘음
  13 j3  1871 ~ 4076   (193.8°)  ← 경계에 붙음
  14 j4  1743 ~ 4070   (204.5°)  ← 경계에 붙음
  15 그리퍼 2328 ~ 3788 (128.3°)
"""
import glob, json, os, sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler
from sts_limits_measured_arm2_20261029_1946 import MEASURED_RAW

ADDR_MIN, ADDR_MAX, ADDR_OFS, ADDR_TORQUE, ADDR_LOCK, ADDR_PRESENT = 9, 11, 31, 40, 55, 56
MARGIN = 20
IDS = [11, 12, 13, 14, 15]

def find_port():
    d = sorted(glob.glob("/dev/serial/by-id/*1a86*"))
    if d: return d[0]
    for t in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):   # WSL 에서 by-id 가 안 생길 때
        try:
            if open(f"/sys/class/tty/{os.path.basename(t)}/device/../idVendor").read().strip() == "1a86": return t
        except Exception: pass
    return None

dev = find_port()
if not dev: print("로봇 어댑터가 안 보입니다."); sys.exit(1)
port = PortHandler(dev)
if not port.openPort(): print("포트를 못 열었습니다 — 앱·다른 도구를 끄고 다시 하세요."); sys.exit(1)
port.setBaudRate(1000000)
pk = PacketHandler(0)

def rd(i, a, n=2):
    f = pk.read2ByteTxRx if n == 2 else pk.read1ByteTxRx
    for _ in range(6):
        try:
            v, r, _ = f(port, i, a)
            if r == 0: return v
        except Exception: pass
        time.sleep(0.03)
    return None
def wr(i, a, v, n=2):
    f = pk.write2ByteTxRx if n == 2 else pk.write1ByteTxRx
    for _ in range(6):
        try:
            if f(port, i, a, int(v))[0] == 0: return True
        except Exception: pass
        time.sleep(0.03)
    return False
dec = lambda v: -(v & 0x7FF) if (v & 0x800) else (v & 0x7FF)
enc = lambda v: (((-int(v)) & 0x7FF) | 0x800) if v < 0 else (int(v) & 0x7FF)

# 0. 응답 확인 · 토크 끄기 · 원래 값 저장
miss = [i for i in IDS if rd(i, ADDR_PRESENT) is None]
if miss: print(f"응답 없는 모터 {miss} — 2번 팔을 연결하고 다시 하세요."); sys.exit(1)
for i in IDS: wr(i, ADDR_TORQUE, 0, 1)
bak = {i: dict(ofs=dec(rd(i, ADDR_OFS)), min=rd(i, ADDR_MIN), max=rd(i, ADDR_MAX), pos=rd(i, ADDR_PRESENT)) for i in IDS}
if not os.path.exists("sts_eeprom_backup_arm2.json"):          # 두 번째 실행에서 원래 값을 덮어쓰지 않게
    json.dump(bak, open("sts_eeprom_backup_arm2.json", "w"), indent=1)
    print("원래 값 저장: sts_eeprom_backup_arm2.json")
for i, b in bak.items(): print(f"  ID{i}: 보정 {b['ofs']:+d} · 한계 {b['min']}~{b['max']} · 위치 {b['pos']}")
if os.path.exists("sts_apply_arm2_result.json"):
    print("\n⚠ 이미 한 번 적용했습니다 (sts_apply_arm2_result.json) — 다시 돌리지 마세요."); sys.exit(1)
if any(b["ofs"] != 0 for b in bak.values()):
    if "--ofs-ok" not in sys.argv:
        print("\n⚠ 이미 위치 보정이 들어간 모터가 있습니다 — 이 프로그램은 보정 0 기준으로 잰 값을 씁니다.")
        print("  잴 때(10-29)도 이 보정이 있었다면 --ofs-ok 를 붙여 다시 실행하세요 (지금 보정을 기준으로 씁니다)."); sys.exit(1)
    # 11-02: 11~13 의 +85 는 이 프로그램이 넣은 게 아니라 원래 있던 값 (결과 파일 없음).
    #   10-29 측정도 이 보정이 있는 채로 했다 — 지금 위치가 전부 잰 범위 안. 보정값 옮기기는 지금 값(o0)에서 시작하므로 그대로 맞다.
    print("\n지금 들어 있는 위치 보정을 기준으로 적용합니다 (--ofs-ok).")

res = {}
for i in IDS:
    lo, hi = MEASURED_RAW[i]
    shift = 0
    if lo < 50 or hi > 4045:                                   # 경계를 넘거나 붙음 → 보정값 옮기기
        want = 2048 - (lo + hi) // 2
        want = ((want + 2048) % 4096) - 2048                   # -2048~2047 로 (한 바퀴 돌려 같은 자리)
        p0 = rd(i, ADDR_PRESENT); o0 = bak[i]["ofs"]; ok = False
        for conv in (-1, +1):                                  # 실측: 보정 +X → 위치 -X (아니면 반대)
            cand = max(-2047, min(2047, o0 + conv * want))
            wr(i, ADDR_LOCK, 0, 1); time.sleep(0.03)
            wr(i, ADDR_OFS, enc(cand)); time.sleep(0.1)
            p1 = rd(i, ADDR_PRESENT)
            d = ((p1 - p0 + 2048) % 4096) - 2048 if p1 is not None else 9999
            if abs(d - want) < 60:
                shift = d; ok = True
                print(f"ID{i}: 보정 {o0:+d} → {cand:+d} · 위치 {d:+d} 이동 (기대 {want:+d}) 확인")
                break
            wr(i, ADDR_OFS, enc(o0)); time.sleep(0.1)
            print(f"ID{i}: 방향 {'B' if conv < 0 else 'A'} 불일치 (이동 {d:+d}) → 반대로 시도")
        if not ok:
            wr(i, ADDR_LOCK, 1, 1)
            print(f"ID{i}: 보정 실패 — 원래 값으로 두었습니다. 멈춥니다."); sys.exit(1)
    k = round(((lo + hi) / 2 + shift - 2048) / 4096)          # 이어서 센 값(예: 3919~6391)은 한 바퀴 빼서 0~4095 안으로
    lo, hi = lo - 4096 * k, hi - 4096 * k
    lo2, hi2 = lo + shift + MARGIN, hi + shift - MARGIN
    assert 0 <= lo2 < hi2 <= 4095, (i, lo2, hi2)
    wr(i, ADDR_LOCK, 0, 1); time.sleep(0.03)
    wr(i, ADDR_MIN, lo2); time.sleep(0.03); wr(i, ADDR_MAX, hi2); time.sleep(0.03)
    wr(i, ADDR_LOCK, 1, 1); time.sleep(0.03)
    got = (rd(i, ADDR_MIN), rd(i, ADDR_MAX)); pos = rd(i, ADDR_PRESENT)
    res[i] = dict(measured=(lo + shift, hi + shift), motor_limit=got, shift=shift, pos=pos, ofs=dec(rd(i, ADDR_OFS)))
    print(f"ID{i}: 범위 {lo+shift}~{hi+shift} · 모터 한계 {got[0]}~{got[1]} · 지금 위치 {pos}"
          + ("  ✓" if got == (lo2, hi2) and got[0] <= pos <= got[1] else "  ⚠ 확인 필요"))
json.dump(res, open("sts_apply_arm2_result.json", "w"), indent=1)
print("\n새 값 저장: sts_apply_arm2_result.json")
port.closePort()
