#!/usr/bin/env python3
"""
5개 모터(j1~j5) 핵심 상태 한눈에 확인 (재시도 내장).
- Operating Mode / Homing Offset / Present Position
usbip 통신이 간헐적으로 끊겨도 항목별로 최대 5회 재시도한다.
"""
import time
from dynamixel_sdk import PortHandler, PacketHandler

PORT = '/dev/ttyUSB0'
BAUD = 57600
MOTORS = {0: 'j1', 1: 'j2', 2: 'j3', 3: 'j4', 4: 'j5', 5: 'j6'}
MODE = {1: 'Velocity', 3: 'Position', 4: 'Extended', 5: 'Curr-Pos', 16: 'PWM'}


def s32(v):
    return v - 0x100000000 if v > 0x7FFFFFFF else v


def read(packet, port, dxl_id, addr, size, retries=5):
    for _ in range(retries):
        if size == 1:
            v, r, _ = packet.read1ByteTxRx(port, dxl_id, addr)
        elif size == 2:
            v, r, _ = packet.read2ByteTxRx(port, dxl_id, addr)
        else:
            v, r, _ = packet.read4ByteTxRx(port, dxl_id, addr)
        if r == 0:
            return s32(v)
        time.sleep(0.05)
    return None


def main():
    port = PortHandler(PORT)
    packet = PacketHandler(2.0)
    if not port.openPort():
        print('포트 열기 실패'); return
    port.setBaudRate(BAUD)

    print('=' * 70)
    print(f"{'조인트':<6}{'ID':<4}{'Mode':<11}{'HomingOff':>10}{'PresentPos':>12}{'정규':>7}")
    print('-' * 70)
    for dxl_id, jname in MOTORS.items():
        mode = read(packet, port, dxl_id, 11, 1)
        homing = read(packet, port, dxl_id, 20, 4)
        pos = read(packet, port, dxl_id, 132, 4)
        if mode is None and pos is None:
            print(f"{jname:<6}{dxl_id:<4}통신 실패 (모두 재시도 소진)")
            continue
        mode_s = MODE.get(mode, f'?({mode})') if mode is not None else '?'
        homing_s = str(homing) if homing is not None else '?'
        pos_s = str(pos) if pos is not None else '?'
        norm_s = str(pos % 4096) if pos is not None else '?'
        warn = ''
        if pos is not None and (pos % 4096 >= 3950 or pos % 4096 <= 150):
            warn = '  <<경계'
        print(f"{jname:<6}{dxl_id:<4}{mode_s:<11}{homing_s:>10}{pos_s:>12}{norm_s:>7}{warn}")
    print('=' * 70)
    port.closePort()


if __name__ == '__main__':
    main()
