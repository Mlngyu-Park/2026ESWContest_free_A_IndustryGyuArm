#!/usr/bin/env python3
"""j4(ID4) 모터의 펌웨어 각도 한계만 최신화.

배경: 2026-07-27 에 모터 안에 박아둔 한계(169~1975)가 남아 있는데,
j4 모터를 돌려 재조립하면서 실제 가동범위가 1747~4030 으로 바뀌었다.
그 결과 새 base(raw 2895)로 보내도 모터가 1975 에서 잘라버려
"base 로 갔는데 엉뚱한 자세" 가 됐다.  -> j4 한계만 다시 쓴다.

다른 축은 건드리지 않는다 (사용자 지시).
토크는 켜지 않으므로 팔은 움직이지 않는다.
"""
import sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER = 16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from sts_limits import LIMIT_RAW

PORT = '/dev/ttyACM0'; BAUD = 1000000
ADDR_TORQUE = 40; ADDR_PRESENT = 56
ADDR_MIN_ANGLE = 9; ADDR_MAX_ANGLE = 11; ADDR_LOCK = 55
TARGET_ID = 4                 # j4 만
MARGIN = 20                   # 끝단 충돌 방지용 여유 (raw)

port = PortHandler(PORT)
if not port.openPort(): print('포트 실패'); sys.exit(1)
port.setBaudRate(BAUD)
packet = PacketHandler(0)

def wr1(i, a, v): packet.write1ByteTxRx(port, i, a, v)
def wr2(i, a, v): packet.write2ByteTxRx(port, i, a, v)
def rd(i, a, tries=6):
    """읽기 (통신 오류가 잦아 여러 번 시도)."""
    for _ in range(tries):
        try:
            v, r, _ = packet.read2ByteTxRx(port, i, a)
            if r == COMM_SUCCESS: return v
        except Exception:
            pass
        time.sleep(0.05)
    return None

wr1(TARGET_ID, ADDR_TORQUE, 0); time.sleep(0.2)   # 안전: 토크 OFF

before_lo = rd(TARGET_ID, ADDR_MIN_ANGLE)
before_hi = rd(TARGET_ID, ADDR_MAX_ANGLE)
pos = rd(TARGET_ID, ADDR_PRESENT)
mn, mx = LIMIT_RAW[TARGET_ID]
lo = max(0, min(4095, mn + MARGIN))
hi = max(0, min(4095, mx - MARGIN))

print(f'j4(ID{TARGET_ID}) 펌웨어 각도 한계 최신화')
print(f'  이전(모터 안): {before_lo} ~ {before_hi}')
print(f'  새 값(측정치 {mn}~{mx}, 여유 {MARGIN}): {lo} ~ {hi}')
print(f'  현재 위치: {pos}')

wr1(TARGET_ID, ADDR_LOCK, 0); time.sleep(0.05)    # EEPROM 잠금 해제
wr2(TARGET_ID, ADDR_MIN_ANGLE, lo); time.sleep(0.05)
wr2(TARGET_ID, ADDR_MAX_ANGLE, hi); time.sleep(0.05)
wr1(TARGET_ID, ADDR_LOCK, 1); time.sleep(0.05)    # 다시 잠금

r_lo = rd(TARGET_ID, ADDR_MIN_ANGLE); r_hi = rd(TARGET_ID, ADDR_MAX_ANGLE)
ok = (r_lo == lo and r_hi == hi)
print(f'  확인: {r_lo} ~ {r_hi}   {"OK" if ok else "쓰기 실패"}')

try:
    from sts_zero import ZERO_RAW
    z = ZERO_RAW.get(TARGET_ID)
    if z is not None:
        inside = r_lo <= z <= r_hi
        print(f'  base 영점 {z} 이 한계 안: {"예 OK" if inside else "아니오 ❌"}')
except Exception:
    pass

port.closePort()
sys.exit(0 if ok else 2)
