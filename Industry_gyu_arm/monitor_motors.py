#!/usr/bin/env python3
"""
모터 위치 실시간 모니터링
손으로 움직여서 각 모터 ID와 조인트 매핑 확인용
"""
import time
import sys
from dynamixel_sdk import PortHandler, PacketHandler

PORT     = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL = 2.0

ADDR_TORQUE_ENABLE    = 64
ADDR_PRESENT_POSITION = 132

MOTORS = {
    0: 'ID0',
    1: 'ID1',
    2: 'ID2',
    3: 'ID3',
    4: 'ID4',
}

port   = PortHandler(PORT)
packet = PacketHandler(PROTOCOL)

if not port.openPort():
    raise RuntimeError('포트 열기 실패')
if not port.setBaudRate(BAUDRATE):
    raise RuntimeError('Baudrate 설정 실패')

# 토크 OFF → 손으로 움직일 수 있게
print('토크 OFF (손으로 자유롭게 움직일 수 있습니다)')
for dxl_id in MOTORS:
    packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)

print('Ctrl+C 로 종료\n')

prev = {i: None for i in MOTORS}

LINES = 2 + len(MOTORS) + 2  # 헤더2 + 모터수 + 푸터2

try:
    first = True
    while True:
        lines = []
        lines.append('=' * 55)
        lines.append(f'  {"ID":<6} {"현재값":>8}  {"변화량":>8}  {"도수(deg)":>10}')
        lines.append('-' * 55)
        for dxl_id in sorted(MOTORS.keys()):
            raw, result, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            if result != 0:
                lines.append(f'  ID{dxl_id:<4} 통신 오류')
                continue
            if raw > 0x7FFFFFFF:
                raw -= 0x100000000

            deg   = raw * 360.0 / 4096.0
            delta = raw - prev[dxl_id] if prev[dxl_id] is not None else 0
            arrow = ' ▲' if delta > 5 else (' ▼' if delta < -5 else '  ')
            lines.append(f'  ID{dxl_id:<4} {raw:>8}  {delta:>+8}  {deg:>9.1f}°{arrow}')
            prev[dxl_id] = raw

        lines.append('=' * 55)
        lines.append('손으로 움직여서 어떤 ID가 변하는지 확인하세요')

        # 첫 출력은 그냥 print, 이후는 커서 올려서 덮어쓰기
        if not first:
            sys.stdout.write(f'\033[{len(lines)}A')
        for line in lines:
            sys.stdout.write(line + '\n')
        sys.stdout.flush()
        first = False

        time.sleep(0.2)

except KeyboardInterrupt:
    print('\n\n종료')
finally:
    port.closePort()
