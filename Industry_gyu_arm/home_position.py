#!/usr/bin/env python3
"""
기본 위치(Home Position)로 천천히 이동
- j1 → j2 → j3 → j4 → j5 순서로 한 개씩
- 각 모터가 목표 위치 도달 후 다음 모터 이동
- 안전을 위해 천천히 이동 (Profile Velocity 사용)
"""
import time
import signal
from dynamixel_sdk import PortHandler, PacketHandler

PORT        = '/dev/ttyUSB0'
BAUDRATE    = 57600
PROTOCOL    = 2.0

# 레지스터 주소 (XM430/XM540 공통)
ADDR_TORQUE_ENABLE      = 64
ADDR_OPERATING_MODE     = 11
ADDR_PROFILE_VELOCITY   = 112   # 이동 속도 (낮을수록 천천히)
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_MOVING             = 122   # 1=이동중, 0=정지

# 기본 위치 raw값
HOME_RAW = {
    0: 3043,   # j1 XC430 베이스
    1: 4095,   # j2 XM430
    2: 3532,   # j3 XM430
    3: 1242,   # j4 XC430
    4: 1993,   # j5 XC430
}

MOTOR_NAMES = {
    0: 'j1 (XC430, 베이스)',
    1: 'j2 (XM430)',
    2: 'j3 (XM430)',
    3: 'j4 (XC430)',
    4: 'j5 (XC430)',
}

# 이동 속도 (0~32767, 낮을수록 천천히)
# 100 = 매우 천천히, 300 = 보통
PROFILE_VELOCITY = 20

# 목표 위치 도달 판정 허용 오차 (raw 단위, 1 = 약 0.088도)
POSITION_TOLERANCE = 10

# 모터 도달 대기 타임아웃 (초)
TIMEOUT_SEC = 10.0


def open_port():
    port = PortHandler(PORT)
    packet = PacketHandler(PROTOCOL)
    if not port.openPort():
        raise RuntimeError('포트 열기 실패: ' + PORT)
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError('Baudrate 설정 실패')
    print(f'포트 연결 성공: {PORT}')
    return port, packet


def enable_torque(port, packet, dxl_id, enable=True):
    val = 1 if enable else 0
    result, error = packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, val)
    if result != 0:
        print(f'  [경고] ID:{dxl_id} 토크 {"활성화" if enable else "비활성화"} 실패')


def set_profile_velocity(port, packet, dxl_id, velocity):
    result, error = packet.write4ByteTxRx(port, dxl_id, ADDR_PROFILE_VELOCITY, velocity)
    if result != 0:
        print(f'  [경고] ID:{dxl_id} 속도 설정 실패')


def get_present_position(port, packet, dxl_id):
    raw, result, error = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    if result != 0:
        return None
    if raw > 0x7FFFFFFF:
        raw -= 0x100000000
    return raw


def nearest_goal(current, goal):
    """현재 위치에서 목표까지 가까운 방향의 goal raw값 반환 (0~4095 회전 기준)"""
    direct = goal - current          # 직접 방향 차이
    wrap   = direct - 4096 if direct > 0 else direct + 4096  # 반대 방향 차이
    if abs(direct) <= abs(wrap):
        return goal                  # 직접 방향이 더 가까움
    else:
        return current + wrap        # 반대 방향 (범위 벗어나도 모터가 처리)


def move_to_home(port, packet, dxl_id):
    name    = MOTOR_NAMES[dxl_id]
    goal    = HOME_RAW[dxl_id]
    current = get_present_position(port, packet, dxl_id)

    if current is None:
        print(f'  [오류] ID:{dxl_id} {name} 현재 위치 읽기 실패 - 스킵')
        return False

    # 가까운 방향으로 goal 결정
    actual_goal = nearest_goal(current, goal)
    direct_dist = abs(goal - current)
    wrap_dist   = 4096 - direct_dist
    direction   = '직접' if direct_dist <= wrap_dist else '반대방향(짧은쪽)'

    print(f'\n[{dxl_id}] {name}')
    print(f'  현재: {current}  →  목표: {goal}  ({direction}, 차이: {min(direct_dist, wrap_dist)})')

    if abs(current - goal) <= POSITION_TOLERANCE:
        print(f'  이미 기본 위치에 있음, 스킵')
        return True

    # 속도 설정
    set_profile_velocity(port, packet, dxl_id, PROFILE_VELOCITY)

    # 목표 위치 전송 (가까운 방향)
    result, error = packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, actual_goal & 0xFFFFFFFF)
    if result != 0:
        print(f'  [오류] 목표 위치 전송 실패')
        return False

    print(f'  이동 시작...', end='', flush=True)

    # 도달 대기
    start = time.time()
    while True:
        pos = get_present_position(port, packet, dxl_id)
        if pos is not None and abs(pos - goal) <= POSITION_TOLERANCE:
            print(f' 완료! (현재: {pos})')
            break
        if time.time() - start > TIMEOUT_SEC:
            print(f' [경고] 타임아웃! (현재: {pos})')
            break
        print('.', end='', flush=True)
        time.sleep(0.1)

    time.sleep(0.3)  # 진동 안정화 대기
    return True


def main():
    print('=' * 50)
    print('  로봇 팔 기본 위치 이동 시작')
    print('  순서: j1 → j2 → j3 → j4 → j5')
    print(f'  속도: {PROFILE_VELOCITY} (낮을수록 천천히)')
    print('=' * 50)

    port, packet = open_port()

    try:
        # 모든 모터 토크 활성화
        print('\n모든 모터 토크 활성화...')
        for dxl_id in sorted(HOME_RAW.keys()):
            enable_torque(port, packet, dxl_id, True)
            time.sleep(0.1)

        # j1 → j2 → j3 → j4 → j5 순서로 이동
        for dxl_id in sorted(HOME_RAW.keys()):
            move_to_home(port, packet, dxl_id)

        print('\n' + '=' * 50)
        print('  기본 위치 이동 완료!')
        print('=' * 50)

    except KeyboardInterrupt:
        print('\n\n[중단] 즉시 정지 - 모든 모터 현재 위치 고정...')
        # 즉시 정지: 현재 위치를 goal로 덮어써서 멈춤
        for dxl_id in sorted(HOME_RAW.keys()):
            pos = get_present_position(port, packet, dxl_id)
            if pos is not None:
                packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, pos)
        print('정지 완료')

        # 토크 해제 중 Ctrl+C 무시 (반드시 완료해야 안전)
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        # 천천히 토크 해제 (j5 → j4 → j3 → j2 → j1 역순)
        print('토크 천천히 해제 중... (Ctrl+C 무시됨, 잠시 기다려주세요)')
        for dxl_id in sorted(HOME_RAW.keys(), reverse=True):
            time.sleep(0.5)
            enable_torque(port, packet, dxl_id, False)
            print(f'  ID{dxl_id} ({MOTOR_NAMES[dxl_id]}) 토크 해제')
        print('토크 해제 완료')
    finally:
        port.closePort()
        print('포트 닫힘')


if __name__ == '__main__':
    main()
