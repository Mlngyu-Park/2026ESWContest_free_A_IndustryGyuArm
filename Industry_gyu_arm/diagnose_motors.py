#!/usr/bin/env python3
"""
Dynamixel 모터 진단:
- Operating Mode (멀티턴/위치 모드 판별)
- Present Position (raw, 정규화, 환산 각도)
- Torque Enable 상태
- Model Number
멀티턴(Extended Position) 모드면 위치값이 0~4095를 넘어 누적된다.
"""
from dynamixel_sdk import PortHandler, PacketHandler

PORT     = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL = 2.0

ADDR_MODEL_NUMBER     = 0
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_PRESENT_POSITION = 132
ADDR_MIN_POS_LIMIT    = 52   # Min Position Limit (4byte)
ADDR_MAX_POS_LIMIT    = 48   # Max Position Limit (4byte)

MOTORS = {0: 'j1', 1: 'j2', 2: 'j3', 3: 'j4', 4: 'j5'}

OP_MODE_NAME = {
    1:  'Velocity',
    3:  'Position (0~4095, 단일회전)',
    4:  'Extended Position (멀티턴!)',
    5:  'Current-based Position',
    16: 'PWM',
}


def signed32(v):
    return v - 0x100000000 if v > 0x7FFFFFFF else v


def main():
    port = PortHandler(PORT)
    packet = PacketHandler(PROTOCOL)
    if not port.openPort():
        print('포트 열기 실패'); return
    if not port.setBaudRate(BAUDRATE):
        print('Baudrate 실패'); return

    print('=' * 78)
    print(f"{'ID':<3}{'조인트':<6}{'모델':<7}{'OperatingMode':<28}{'Torque':<8}{'raw':>8}{'정규화':>8}{'deg':>8}")
    print('-' * 78)
    for dxl_id, jname in MOTORS.items():
        model, r1, _ = packet.read2ByteTxRx(port, dxl_id, ADDR_MODEL_NUMBER)
        mode,  r2, _ = packet.read1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE)
        torq,  r3, _ = packet.read1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE)
        pos,   r4, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
        if r4 != 0:
            print(f"{dxl_id:<3}{jname:<6}  읽기 실패")
            continue
        pos = signed32(pos)
        norm = pos % 4096            # 한 바퀴 내 위치
        deg  = pos * 360.0 / 4096.0  # 누적 각도(멀티턴이면 360 초과)
        mode_str = OP_MODE_NAME.get(mode, f'알수없음({mode})')
        torq_str = 'ON' if torq == 1 else 'OFF'
        print(f"{dxl_id:<3}{jname:<6}{model:<7}{mode_str:<28}{torq_str:<8}{pos:>8}{norm:>8}{deg:>8.1f}")

    print('=' * 78)
    print("판독 팁:")
    print(" - OperatingMode가 'Extended Position(멀티턴)'이면 raw가 4095를 넘을 수 있음")
    print(" - deg가 360을 크게 넘으면 모터가 여러 바퀴 돈 상태 → 영점/한계 재정의 필요")
    print(" - Torque가 ON이면 손으로 안 돌아감. 손으로 맞추려면 OFF 필요")
    port.closePort()


if __name__ == '__main__':
    main()
