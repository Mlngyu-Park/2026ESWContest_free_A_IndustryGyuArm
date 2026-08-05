#!/usr/bin/env python3
from dynamixel_sdk import PortHandler, PacketHandler

PORT        = '/dev/ttyUSB0'
BAUDRATE    = 57600
PROTOCOL    = 2.0

# XM430/XM540 레지스터 주소
ADDR_PRESENT_POSITION = 132
ADDR_HOMING_OFFSET    = 20

# 모터 ID → 조인트 이름 매핑
MOTORS = {
    0: 'j1 (XM540, 베이스)',
    1: 'j2 (XM430)',
    2: 'j3 (XM430)',
    3: 'j4 (XM540)',
    4: 'j5 (XM430)',
}

# 0자세 raw값 (직접 측정)
ZERO_RAW = {
    0: 3072,
    1: 4095,
    2: 3584,
    3: 1253,
    4: 2048,
}

def raw_to_rad(dxl_id, raw, homing_offset=0):
    return (raw - ZERO_RAW[dxl_id]) * (3.14159265 / 2048.0)

portHandler   = PortHandler(PORT)
packetHandler = PacketHandler(PROTOCOL)

if not portHandler.openPort():
    print('포트 열기 실패')
    exit()

if not portHandler.setBaudRate(BAUDRATE):
    print('baudrate 설정 실패')
    exit()

print('=' * 55)
print(f'{"ID":<5} {"조인트":<20} {"raw":>8} {"rad":>10} {"deg":>8}')
print('-' * 55)

for dxl_id, name in MOTORS.items():
    raw, result, error = packetHandler.read4ByteTxRx(
        portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if result != 0:
        print(f'{dxl_id:<5} {name:<20} 통신 오류: {packetHandler.getTxRxResult(result)}')
        continue
    if raw > 0x7FFFFFFF:
        raw -= 0x100000000

    homing, result2, error2 = packetHandler.read4ByteTxRx(
        portHandler, dxl_id, ADDR_HOMING_OFFSET)
    if result2 != 0:
        homing = 0
    if homing > 0x7FFFFFFF:
        homing -= 0x100000000

    rad = raw_to_rad(dxl_id, raw, homing)
    deg = rad * 180.0 / 3.14159265
    print(f'{dxl_id:<5} {name:<20} {raw:>8} offset={homing:>7} {rad:>10.4f} {deg:>8.2f}°')

print('=' * 55)
portHandler.closePort()
