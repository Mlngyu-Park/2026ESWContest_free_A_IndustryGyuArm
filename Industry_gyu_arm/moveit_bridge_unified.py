#!/usr/bin/env python3
"""
MoveIt2(RViz) <-> 실제 Dynamixel 모터 통합 브리지
- /arm_controller/follow_joint_trajectory 액션 서버 (RViz/MoveIt 명령 -> 모터)
- joint_states 발행 (모터 실제 위치 -> RViz 반영)
- 하나의 시리얼 포트만 열어 읽기/쓰기 충돌 방지 (Lock 직렬화)
- 시작 시 기본 자세(HOME)로 이동

기존 moveit_bridge.py + joint_state_publisher.py 를 한 프로세스로 통합한 버전.
포트를 두 프로세스가 동시에 여는 문제(serial multiple access)를 해결한다.
"""
import time
import signal
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from dynamixel_sdk import PortHandler, PacketHandler

PORT     = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL = 2.0

ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

# 모터 ID → 조인트 이름
MOTOR_JOINTS = {
    0: 'j1',
    1: 'j2',
    2: 'j3',
    3: 'j4',
    4: 'j5',
}
JOINT_MOTORS = {v: k for k, v in MOTOR_JOINTS.items()}

# 기본 자세 raw값 — 0자세(ZERO_RAW)와 동일하게 설정
HOME_RAW = {
    0: 3076,
    1: 2043,
    2: 1004,
    3: 1253,
    4: 2042,
}

# 0자세 raw값 (rad → raw 변환 기준) — 2026-06-16 현재 자세를 0점으로 재캡처
ZERO_RAW = {
    0: 3076,
    1: 2043,
    2: 1004,
    3: 1253,
    4: 2042,
}

PROFILE_VELOCITY_HOME = 20   # 기본 자세 이동 속도 (느리게)
PROFILE_VELOCITY_MOVE = 50   # 트래젝토리 실행 속도
POSITION_TOLERANCE    = 10   # 도달 판정 오차
TIMEOUT_SEC           = 15.0

JOINT_STATE_HZ = 50.0        # joint_states 발행 주기


def nearest_goal(current, goal):
    """현재 위치에서 목표까지 가까운 방향의 goal raw값 반환"""
    direct = goal - current
    wrap   = direct - 4096 if direct > 0 else direct + 4096
    if abs(direct) <= abs(wrap):
        return goal
    else:
        return current + wrap


def rad_to_raw(dxl_id, rad):
    raw = int(ZERO_RAW[dxl_id] + rad * 2048.0 / 3.14159265)
    return raw  # 범위 제한 제거 - nearest_goal이 방향 결정


def raw_to_rad(dxl_id, raw):
    return (raw - ZERO_RAW[dxl_id]) * (3.14159265 / 2048.0)


class DynamixelBridge(Node):
    def __init__(self):
        super().__init__('dynamixel_bridge')

        # Dynamixel 연결 (포트는 이 노드에서 단 하나만 연다)
        self.port   = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL)
        # 읽기(joint_states 타이머)와 쓰기(trajectory/home)가
        # 같은 포트를 두드리므로 Lock으로 접근을 직렬화한다.
        self.io_lock = threading.Lock()

        if not self.port.openPort():
            raise RuntimeError('포트 열기 실패: ' + PORT)
        if not self.port.setBaudRate(BAUDRATE):
            raise RuntimeError('Baudrate 설정 실패')
        self.get_logger().info('Dynamixel 연결 성공')

        # 토크 활성화
        for dxl_id in MOTOR_JOINTS:
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info('토크 활성화 완료 - 현재 자세 유지')

        # joint_states 발행 (모터 실제 위치 -> RViz)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.js_timer = self.create_timer(1.0 / JOINT_STATE_HZ, self.publish_joint_states)

        # FollowJointTrajectory 액션 서버
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('액션 서버 시작 - RViz에서 플래닝 가능')

    # ---- 저수준 포트 접근 (Lock 보호) -------------------------------------

    def _read_position_locked(self, dxl_id):
        """주의: 호출자가 io_lock을 이미 잡고 있어야 함."""
        raw, result, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        if result != 0:
            return None
        if raw > 0x7FFFFFFF:
            raw -= 0x100000000
        return raw

    def get_position(self, dxl_id):
        with self.io_lock:
            return self._read_position_locked(dxl_id)

    def _write_goal_locked(self, dxl_id, raw):
        self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_GOAL_POSITION, raw & 0xFFFFFFFF)

    # ---- joint_states 발행 ----------------------------------------------

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.io_lock:
            for dxl_id, joint_name in MOTOR_JOINTS.items():
                raw = self._read_position_locked(dxl_id)
                if raw is None:
                    return  # 한 모터라도 실패하면 이번 주기는 건너뜀
                msg.name.append(joint_name)
                msg.position.append(raw_to_rad(dxl_id, raw))
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
        self.js_pub.publish(msg)

    # ---- HOME 이동 -------------------------------------------------------

    def go_home(self):
        self.get_logger().info('기본 자세로 이동 중... (j1→j2→j3→j4→j5)')
        for dxl_id in sorted(HOME_RAW.keys()):
            goal = HOME_RAW[dxl_id]
            current = self.get_position(dxl_id)
            if current is None:
                self.get_logger().warn(f'ID{dxl_id} 읽기 실패, 스킵')
                continue
            if abs(current - goal) <= POSITION_TOLERANCE:
                self.get_logger().info(f'  ID{dxl_id} 이미 기본 위치')
                continue

            actual_goal = nearest_goal(current, goal)
            cur_deg  = current * 360.0 / 4096.0
            goal_deg = goal    * 360.0 / 4096.0
            self.get_logger().info(f'  ID{dxl_id} 현재: {current} ({cur_deg:.1f}°) → 목표: {goal} ({goal_deg:.1f}°)')
            with self.io_lock:
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY_HOME)
                self._write_goal_locked(dxl_id, actual_goal)

            start = time.time()
            while True:
                pos = self.get_position(dxl_id)
                if pos is not None:
                    pos_deg = pos * 360.0 / 4096.0
                    self.get_logger().info(f'  ID{dxl_id} 이동중: {pos} ({pos_deg:.1f}°) / 목표: {goal} ({goal_deg:.1f}°)')
                    if abs(pos - goal) <= POSITION_TOLERANCE:
                        self.get_logger().info(f'  ID{dxl_id} 완료!')
                        break
                if time.time() - start > TIMEOUT_SEC:
                    self.get_logger().warn(f'  ID{dxl_id} 타임아웃')
                    break
                time.sleep(0.2)
            time.sleep(0.3)

        self.get_logger().info('기본 자세 완료')

    # ---- 액션 콜백 -------------------------------------------------------

    def goal_callback(self, goal_request):
        self.get_logger().info('트래젝토리 요청 수신')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('트래젝토리 취소 요청')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('트래젝토리 실행 시작')
        # FollowJointTrajectory.Goal.trajectory 는 이미 JointTrajectory.
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points

        # 속도 설정
        with self.io_lock:
            for dxl_id in MOTOR_JOINTS:
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY_MOVE)

        for i, point in enumerate(points):
            if goal_handle.is_cancel_requested:
                # 현재 위치에서 즉시 정지
                with self.io_lock:
                    for dxl_id in MOTOR_JOINTS:
                        pos = self._read_position_locked(dxl_id)
                        if pos is not None:
                            self._write_goal_locked(dxl_id, pos)
                goal_handle.canceled()
                self.get_logger().info('트래젝토리 취소됨')
                return FollowJointTrajectory.Result()

            # 각 조인트 위치 전송
            with self.io_lock:
                for j, joint_name in enumerate(joint_names):
                    if joint_name not in JOINT_MOTORS:
                        continue
                    dxl_id = JOINT_MOTORS[joint_name]
                    rad    = point.positions[j]
                    raw    = rad_to_raw(dxl_id, rad)
                    self._write_goal_locked(dxl_id, raw)

            self.get_logger().info(f'  포인트 {i+1}/{len(points)} 전송')

            # 다음 포인트까지 대기
            if i < len(points) - 1:
                next_time = points[i+1].time_from_start
                curr_time = point.time_from_start
                dt = (next_time.sec - curr_time.sec) + \
                     (next_time.nanosec - curr_time.nanosec) * 1e-9
                if dt > 0:
                    time.sleep(dt)

        goal_handle.succeed()
        self.get_logger().info('트래젝토리 실행 완료')
        return FollowJointTrajectory.Result()

    # ---- 종료 -----------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('종료 중 - 즉시 정지...')
        # joint_states 타이머 먼저 멈춰 포트 접근 충돌 방지
        try:
            self.js_timer.cancel()
        except Exception:
            pass
        # 즉시 정지: 현재 위치 고정
        with self.io_lock:
            for dxl_id in MOTOR_JOINTS:
                pos = self._read_position_locked(dxl_id)
                if pos is not None:
                    self._write_goal_locked(dxl_id, pos)
        self.get_logger().info('정지 완료')

        # 토크 해제 중 Ctrl+C 무시
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        self.get_logger().info('토크 천천히 해제 중... (j5→j4→j3→j2→j1)')
        for dxl_id in sorted(MOTOR_JOINTS.keys(), reverse=True):
            time.sleep(0.5)
            with self.io_lock:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            self.get_logger().info(f'  ID{dxl_id} 토크 해제')
        self.get_logger().info('토크 해제 완료')

        self.port.closePort()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = DynamixelBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        else:
            # node 생성 중 Ctrl+C → 포트만 열린 상태일 수 있음
            try:
                port   = PortHandler(PORT)
                packet = PacketHandler(PROTOCOL)
                if port.openPort():
                    port.setBaudRate(BAUDRATE)
                    signal.signal(signal.SIGINT, signal.SIG_IGN)
                    print('\n[긴급] 토크 해제 중...')
                    for dxl_id in sorted(MOTOR_JOINTS.keys(), reverse=True):
                        time.sleep(0.5)
                        packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                        print(f'  ID{dxl_id} 토크 해제')
                    port.closePort()
                    print('완료')
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
