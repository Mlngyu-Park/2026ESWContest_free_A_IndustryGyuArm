#!/usr/bin/env python3
"""
현재 모터 자세를 '영점(ZERO_RAW)'으로 캡처.
로봇을 손으로 기준자세(팔 곧게 편 0자세)로 맞춘 뒤 실행하면,
현재 raw값을 ZERO_RAW dict 형식으로 출력한다.
"""
from dynamixel_sdk import PortHandler, PacketHandler

PORT     = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL = 2.0
ADDR_PRESENT_POSITION = 132
MOTORS = {0: 'j1', 1: 'j2', 2: 'j3', 3: 'j4', 4: 'j5', 5: 'j6'}


def signed32(v):
    return v - 0x100000000 if v > 0x7FFFFFFF else v


def main():
    port = PortHandler(PORT)
    packet = PacketHandler(PROTOCOL)
    if not port.openPort():
        print('포트 열기 실패'); return
    port.setBaudRate(BAUDRATE)

    captured = {}
    print('현재 자세 캡처:')
    print('-' * 40)
    for dxl_id, jname in MOTORS.items():
        pos, r, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
        if r != 0:
            print(f'  {jname} (ID{dxl_id}): 읽기 실패!')
            continue
        pos = signed32(pos)
        captured[dxl_id] = pos
        print(f'  {jname} (ID{dxl_id}): raw = {pos}  ({pos*360.0/4096.0:.1f}°)')
    print('-' * 40)
    print('\n# 아래를 ZERO_RAW로 사용:')
    print('ZERO_RAW = {')
    for dxl_id in sorted(captured):
        print(f'    {dxl_id}: {captured[dxl_id]},')
    print('}')
    port.closePort()


if __name__ == '__main__':
    main()
