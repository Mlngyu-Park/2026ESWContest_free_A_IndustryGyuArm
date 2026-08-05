#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import PortHandler, PacketHandler

PORT      = '/dev/ttyUSB0'
BAUDRATE  = 57600
PROTOCOL  = 2.0

ADDR_PRESENT_POSITION = 132

# 모터 ID → 조인트 이름
MOTORS = {
    0: 'j1',
    1: 'j2',
    2: 'j3',
    3: 'j4',
    4: 'j5',
}

# 0자세 raw값 (직접 측정)
ZERO_RAW = {
    0: 3072,
    1: 4095,
    2: 3584,
    3: 1253,
    4: 2048,
}

def raw_to_rad(dxl_id, raw):
    return (raw - ZERO_RAW[dxl_id]) * (3.14159265 / 2048.0)


class DynamixelJointStatePublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_joint_state_publisher')

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz

        self.port = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL)

        if not self.port.openPort():
            self.get_logger().error('포트 열기 실패')
            raise RuntimeError('포트 열기 실패')

        if not self.port.setBaudRate(BAUDRATE):
            self.get_logger().error('baudrate 설정 실패')
            raise RuntimeError('baudrate 설정 실패')

        self.get_logger().info('Dynamixel 연결 성공')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for dxl_id, joint_name in MOTORS.items():
            raw, result, _ = self.packet.read4ByteTxRx(
                self.port, dxl_id, ADDR_PRESENT_POSITION)
            if result != 0:
                self.get_logger().warn(f'ID:{dxl_id} 읽기 실패')
                return
            if raw > 0x7FFFFFFF:
                raw -= 0x100000000

            msg.name.append(joint_name)
            msg.position.append(raw_to_rad(dxl_id, raw))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

        self.publisher.publish(msg)

    def destroy_node(self):
        self.port.closePort()
        super().destroy_node()


def main():
    rclpy.init()
    node = DynamixelJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
