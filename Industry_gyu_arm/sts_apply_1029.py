#!/usr/bin/env python3
"""2026-10-29 재조립 반영 — 손목 = ID 5, ID 4 는 선 없음 (j5 고정).

토크는 켜지 않는다 (안 움직임). 바꾸기 전 모터 값(위치 보정·한계)을 먼저 파일로 저장한다.
  ① ID 5(손목): 잰 범위 2028~4283 이 0/4095 경계를 넘는다 → 위치 보정값을 옮겨 921~3176 쯤으로
     (예전 sts_apply_offsets.py 와 같은 방식 — 써 보고 다시 읽어 확인, 틀리면 원래 값으로)
  ② ID 1·2·3·6: 예전과 같은 모터. 이번엔 끝까지 덜 돌렸을 수 있어 '예전과 이번 중 넓은 쪽' 을 쓴다
  ③ 모든 모터에 한계를 양쪽 MARGIN 만큼 안쪽으로 넣는다
결과: sts_eeprom_backup_1029.json (원래 값) · sts_apply_1029_result.json (새 값)

  ~/anaconda3/envs/gyu_arm/bin/python sts_apply_1029.py
"""
import glob, json, sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler

ADDR_MIN, ADDR_MAX, ADDR_OFS, ADDR_TORQUE, ADDR_LOCK, ADDR_PRESENT = 9, 11, 31, 40, 55, 56
MARGIN = 20
OLD = {1: (118, 2875), 2: (213, 2613), 3: (657, 2913), 6: (822, 2319)}       # sts_limits.py (7-27)
NEW = {1: (124, 2868), 2: (238, 2599), 3: (700, 2886), 5: (2028, 4283), 6: (824, 2237)}   # 10-29 측정
IDS = [1, 2, 3, 5, 6]

dev = sorted(glob.glob("/dev/serial/by-id/*1a86*"))[0]
port = PortHandler(dev); assert port.openPort(); port.setBaudRate(1000000)
pk = PacketHandler(0)

def rd(i, a, n=2):
    f = pk.read2ByteTxRx if n == 2 else pk.read1ByteTxRx
    for _ in range(4):
        v, r, _ = f(port, i, a)
        if r == 0: return v
        time.sleep(0.03)
    return None
def wr(i, a, v, n=2):
    f = pk.write2ByteTxRx if n == 2 else pk.write1ByteTxRx
    for _ in range(4):
        if f(port, i, a, int(v))[0] == 0: return True
        time.sleep(0.03)
    return False
dec = lambda v: -(v & 0x7FF) if (v & 0x800) else (v & 0x7FF)
enc = lambda v: (((-int(v)) & 0x7FF) | 0x800) if v < 0 else (int(v) & 0x7FF)

# 0. 토크 끄기 확인 + 원래 값 저장
for i in IDS: wr(i, ADDR_TORQUE, 0, 1)
bak = {i: dict(ofs=dec(rd(i, ADDR_OFS)), min=rd(i, ADDR_MIN), max=rd(i, ADDR_MAX), pos=rd(i, ADDR_PRESENT)) for i in IDS}
json.dump(bak, open("sts_eeprom_backup_1029.json", "w"), indent=1)
print("원래 값 저장: sts_eeprom_backup_1029.json")
for i, b in bak.items(): print(f"  ID{i}: 보정 {b['ofs']:+d} · 한계 {b['min']}~{b['max']} · 위치 {b['pos']}")

res = {}
for i in IDS:
    lo, hi = NEW[i] if i not in OLD else (min(OLD[i][0], NEW[i][0]), max(OLD[i][1], NEW[i][1]))
    shift = 0
    if lo < 50 or hi > 4045:                                   # 경계를 넘는다 → 보정값 옮기기
        want = 2048 - (lo + hi) // 2
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
    lo2, hi2 = lo + shift + MARGIN, hi + shift - MARGIN
    assert 0 <= lo2 < hi2 <= 4095, (i, lo2, hi2)
    wr(i, ADDR_LOCK, 0, 1); time.sleep(0.03)
    wr(i, ADDR_MIN, lo2); time.sleep(0.03); wr(i, ADDR_MAX, hi2); time.sleep(0.03)
    wr(i, ADDR_LOCK, 1, 1); time.sleep(0.03)
    got = (rd(i, ADDR_MIN), rd(i, ADDR_MAX)); pos = rd(i, ADDR_PRESENT)
    res[i] = dict(measured=(lo + shift, hi + shift), motor_limit=got, shift=shift, pos=pos,
                  ofs=dec(rd(i, ADDR_OFS)))
    print(f"ID{i}: 범위 {lo+shift}~{hi+shift} · 모터 한계 {got[0]}~{got[1]} · 지금 위치 {pos}"
          + ("  ✓" if got == (lo2, hi2) and got[0] <= pos <= got[1] else "  ⚠ 확인 필요"))
json.dump(res, open("sts_apply_1029_result.json", "w"), indent=1)
print("\n새 값 저장: sts_apply_1029_result.json")
port.closePort()
