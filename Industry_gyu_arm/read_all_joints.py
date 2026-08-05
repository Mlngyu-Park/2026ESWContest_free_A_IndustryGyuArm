#!/usr/bin/env python3
"""
모든 조인트(j1~j5)의 현재 위치를 실시간으로 표시.
- raw / deg / rad(영점 기준 환산) 출력
- URDF 한계를 벗어나면 터미널에 '한계값!' 경고
손으로 모터를 돌리며 가동 범위를 확인하는 용도. Ctrl+C로 종료.
"""
import time
from dynamixel_sdk import PortHandler, PacketHandler

PORT     = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL = 2.0
ADDR_PRESENT_POSITION = 132

MOTORS = {0: 'j1', 1: 'j2', 2: 'j3', 3: 'j4', 4: 'j5'}

# 영점(곧게 편 자세에서 캡처한 raw). raw->rad 환산 기준.
ZERO_RAW = {
    0: 3101,
    1: 4086,
    2: 549,
    3: 1137,
    4: 2061,
}

# URDF 관절 한계 (rad)
LIMITS = {
    0: (-1.571, 1.571),   # j1
    1: (-1.571, 1.571),   # j2
    2: (-0.785, 1.571),   # j3
    3: (-3.141, 3.141),   # j4
    4: (-1.571, 1.571),   # j5
}

RATE_HZ = 10.0


def signed32(v):
    return v - 0x100000000 if v > 0x7FFFFFFF else v


def raw_to_rad(dxl_id, raw):
    return (raw - ZERO_RAW[dxl_id]) * (3.14159265 / 2048.0)


def main():
    port = PortHandler(PORT)
    packet = PacketHandler(PROTOCOL)
    if not port.openPort():
        print('포트 열기 실패'); return
    port.setBaudRate(BAUDRATE)

    print('실시간 조인트 모니터 — 손으로 모터를 돌려보세요. (Ctrl+C 종료)')
    print('=' * 72)
    try:
        while True:
            lines = []
            for dxl_id, jname in MOTORS.items():
                pos, r, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
                if r != 0:
                    lines.append(f"{jname}: 읽기실패")
                    continue
                pos = signed32(pos)
                norm = pos % 4096                 # 0~4095 정규화 위치
                rad = raw_to_rad(dxl_id, pos)

                # raw 경계(0/4095) 검사만 — 모터가 한 바퀴 경계에 걸리면 위험
                flag = '  <<< RAW경계!(0/4095)' if (norm >= 3950 or norm <= 150) else ''
                lines.append(
                    f"{jname}: raw={pos:5d}(정규{norm:4d}) rad={rad:+.3f}{flag}"
                )
            # 한 줄씩 갱신 출력 (화면 클리어)
            print('\033[2J\033[H', end='')  # 화면 지우고 커서 맨 위로
            print('실시간 조인트 모니터 (Ctrl+C 종료)')
            print('=' * 72)
            for ln in lines:
                print(ln)
            time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        print('\n종료')
    finally:
        port.closePort()


if __name__ == '__main__':
    main()
