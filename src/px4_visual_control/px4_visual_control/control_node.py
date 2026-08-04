#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def normalize_angle(angle):
    """将角度限制到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    """从 ROS 四元数中提取 yaw。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion(roll, pitch, yaw):
    """Roll、Pitch、Yaw 转四元数。输入单位为弧度。"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class MAVROSVisualAttitudeControl(Node):

    def __init__(self):
        super().__init__('mavros_visual_attitude_control')

        # =========================
        # 可调参数
        # =========================

        # 看到目标时的固定前倾角。
        # 按当前约定，负值表示向前倾。
        self.declare_parameter('forward_pitch_deg', -2.0)

        # 图像水平误差 dx 转横滚角：
        # roll_deg = roll_kp * dx
        self.declare_parameter('roll_kp', -0.01)
        self.declare_parameter('max_roll_deg', 5.0)

        # 图像垂直误差 dy 转 thrust：
        # thrust = neutral_thrust + thrust_kp * dy
        self.declare_parameter('thrust_kp', 0.0005)
        self.declare_parameter('neutral_thrust', 0.5)
        self.declare_parameter('min_thrust', 0.40)
        self.declare_parameter('max_thrust', 0.60)

        # msg.data[3] 作为旋转误差。
        # 单位可以是像素或其他自定义误差。
        self.declare_parameter('yaw_rate_kp', 0.002)
        self.declare_parameter('max_yaw_rate_deg_s', 20.0)

        # 目标丢失时是否继续保持当前 yaw。
        self.declare_parameter('hold_yaw_when_lost', True)

        self.forward_pitch_deg = float(
            self.get_parameter('forward_pitch_deg').value
        )
        self.roll_kp = float(self.get_parameter('roll_kp').value)
        self.max_roll_deg = float(
            self.get_parameter('max_roll_deg').value
        )

        self.thrust_kp = float(self.get_parameter('thrust_kp').value)
        self.neutral_thrust = float(
            self.get_parameter('neutral_thrust').value
        )
        self.min_thrust = float(
            self.get_parameter('min_thrust').value
        )
        self.max_thrust = float(
            self.get_parameter('max_thrust').value
        )

        self.yaw_rate_kp = float(
            self.get_parameter('yaw_rate_kp').value
        )
        self.max_yaw_rate = math.radians(
            float(self.get_parameter('max_yaw_rate_deg_s').value)
        )

        # =========================
        # 控制状态
        # =========================

        self.dx = 0.0
        self.dy = 0.0
        self.rotation_error = 0.0
        self.object_found = False

        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.imu_received = False

        # 20 Hz，dt = 0.05 秒。
        self.control_period = 0.05

        # 输入格式：
        # data[0] = dx，图像水平误差
        # data[1] = dy，图像垂直误差
        # data[2] = 是否发现目标，>0.5 表示发现
        # data[3] = 旋转误差
        # data[4] = cx
        # data[5] = cy
        self.create_subscription(
            Float32MultiArray,
            '/camera/image_deviation',
            self.deviation_callback,
            10
        )

        # 获取飞行器当前姿态，用于保持当前航向。
        self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10
        )

        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop
        )

        self.get_logger().info(
            '视觉姿态控制节点启动，等待 IMU 和视觉目标'
        )

    def imu_callback(self, msg):
        self.current_yaw = quaternion_to_yaw(msg.orientation)

        if not self.imu_received:
            # 第一次收到 IMU 时，使用当前航向作为目标航向。
            self.target_yaw = self.current_yaw
            self.imu_received = True

            self.get_logger().info(
                f'已获取初始 yaw：'
                f'{math.degrees(self.current_yaw):.1f} deg'
            )

    def deviation_callback(self, msg):
        if len(msg.data) < 3:
            self.object_found = False
            return

        self.dx = float(msg.data[0])
        self.dy = float(msg.data[1])
        self.object_found = msg.data[2] > 0.5

        # 如果视觉节点提供旋转误差，放在 data[3]。
        if len(msg.data) >= 4:
            self.rotation_error = float(msg.data[3])
        else:
            self.rotation_error = 0.0

    def control_loop(self):
        if not self.imu_received:
            return

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # 忽略三个 body_rate，使用四元数姿态。
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )

        if self.object_found:
            # 固定小角度前倾，使飞行器向前运动。
            pitch_deg = self.forward_pitch_deg

            # dx 控制左右横滚。
            roll_deg = self.roll_kp * self.dx
            roll_deg = clamp(
                roll_deg,
                -self.max_roll_deg,
                self.max_roll_deg
            )

            # dy 控制上升/下降。
            thrust = self.neutral_thrust + self.thrust_kp * self.dy
            thrust = clamp(
                thrust,
                self.min_thrust,
                self.max_thrust
            )

            # rotation_error 控制旋转速度，并积分成目标 yaw。
            yaw_rate = self.yaw_rate_kp * self.rotation_error
            yaw_rate = clamp(
                yaw_rate,
                -self.max_yaw_rate,
                self.max_yaw_rate
            )

            self.target_yaw += yaw_rate * self.control_period
            self.target_yaw = normalize_angle(self.target_yaw)

        else:
            # 目标丢失：恢复水平、停止升降、保持航向。
            roll_deg = 0.0
            pitch_deg = 0.0
            thrust = self.neutral_thrust

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)

        qx, qy, qz, qw = euler_to_quaternion(
            roll,
            pitch,
            self.target_yaw
        )

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        msg.thrust = float(thrust)

        self.attitude_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MAVROSVisualAttitudeControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()