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
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class MAVROSVisualControl(Node):

    def __init__(self):
        super().__init__('mavros_visual_control')

        # =========================
        # 可调参数
        # =========================

        # 找到目标时的固定前倾角
        self.declare_parameter('forward_pitch_deg', -2.0)

        # dx=-1～1 转换成左右横滚角
        self.declare_parameter('roll_gain_deg', 3.0)
        self.declare_parameter('max_roll_deg', 3.0)

        # dy=-1～1 转换成升降命令
        self.declare_parameter('vertical_gain', 0.03)
        self.declare_parameter('neutral_thrust', 0.5)
        self.declare_parameter('min_thrust', 0.47)
        self.declare_parameter('max_thrust', 0.53)

        # 丢失目标后的搜索旋转速度
        self.declare_parameter('search_yaw_rate_deg_s', 15.0)

        # 水平和垂直误差死区
        self.declare_parameter('horizontal_deadband', 0.05)
        self.declare_parameter('vertical_deadband', 0.05)

        # 连续多少秒没检测到才认为丢失
        self.declare_parameter('lost_timeout', 0.3)

        self.forward_pitch_deg = float(
            self.get_parameter('forward_pitch_deg').value
        )
        self.roll_gain_deg = float(
            self.get_parameter('roll_gain_deg').value
        )
        self.max_roll_deg = float(
            self.get_parameter('max_roll_deg').value
        )

        self.vertical_gain = float(
            self.get_parameter('vertical_gain').value
        )
        self.neutral_thrust = float(
            self.get_parameter('neutral_thrust').value
        )
        self.min_thrust = float(
            self.get_parameter('min_thrust').value
        )
        self.max_thrust = float(
            self.get_parameter('max_thrust').value
        )

        self.search_yaw_rate = math.radians(
            float(
                self.get_parameter(
                    'search_yaw_rate_deg_s'
                ).value
            )
        )

        self.horizontal_deadband = float(
            self.get_parameter('horizontal_deadband').value
        )
        self.vertical_deadband = float(
            self.get_parameter('vertical_deadband').value
        )
        self.lost_timeout = float(
            self.get_parameter('lost_timeout').value
        )

        # =========================
        # 状态变量
        # =========================

        self.dx = 0.0
        self.dy = 0.0
        self.area = 0.0
        self.cx = 0.0
        self.cy = 0.0

        self.object_found = False
        self.last_detection_time = None

        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.imu_received = False

        self.was_tracking = False
        self.control_period = 0.05  # 20 Hz

        self.create_subscription(
            Float32MultiArray,
            '/camera/image_deviation',
            self.deviation_callback,
            10
        )

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

        self.get_logger().info('视觉姿态控制节点启动')

    def imu_callback(self, msg):
        self.current_yaw = quaternion_to_yaw(msg.orientation)

        if not self.imu_received:
            self.target_yaw = self.current_yaw
            self.imu_received = True

    def deviation_callback(self, msg):
        # 视觉消息：
        # [dx, dy, found, area, cx, cy]
        if len(msg.data) < 6:
            return

        self.dx = float(msg.data[0])
        self.dy = float(msg.data[1])
        self.object_found = msg.data[2] > 0.5
        self.area = float(msg.data[3])
        self.cx = float(msg.data[4])
        self.cy = float(msg.data[5])

        if self.object_found:
            self.last_detection_time = self.get_clock().now()

    def target_is_visible(self):
        if self.object_found:
            return True

        if self.last_detection_time is None:
            return False

        elapsed = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        return elapsed <= self.lost_timeout

    def control_loop(self):
        if not self.imu_received:
            return

        visible = self.target_is_visible()

        if visible:
            # 从搜索状态刚刚重新找到目标。
            if not self.was_tracking:
                # 停止旋转，保持找到目标瞬间的航向。
                self.target_yaw = self.current_yaw
                self.get_logger().info('找到目标，停止旋转')

            # 前倾并向目标接近。
            pitch_deg = self.forward_pitch_deg

            # 目标偏右 dx>0，则向右横移。
            if abs(self.dx) > self.horizontal_deadband:
                roll_deg = self.roll_gain_deg * self.dx
            else:
                roll_deg = 0.0

            roll_deg = clamp(
                roll_deg,
                -self.max_roll_deg,
                self.max_roll_deg
            )

            # 目标在画面上方 dy>0，则上升。
            if abs(self.dy) > self.vertical_deadband:
                thrust = (
                    self.neutral_thrust
                    + self.vertical_gain * self.dy
                )
            else:
                thrust = self.neutral_thrust

            thrust = clamp(
                thrust,
                self.min_thrust,
                self.max_thrust
            )

            self.was_tracking = True

        else:
            if self.was_tracking:
                # 刚刚丢失目标，从当前航向开始搜索。
                self.target_yaw = self.current_yaw
                self.get_logger().warn('目标丢失，开始旋转搜索')

            # 丢失时保持水平和高度。
            roll_deg = 0.0
            pitch_deg = 0.0
            thrust = self.neutral_thrust

            # 持续向左旋转搜索。
            self.target_yaw += (
                self.search_yaw_rate * self.control_period
            )
            self.target_yaw = normalize_angle(self.target_yaw)

            self.was_tracking = False

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)

        qx, qy, qz, qw = euler_to_quaternion(
            roll,
            pitch,
            self.target_yaw
        )

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # 忽略三个角速度字段，使用四元数姿态。
        msg.type_mask = 7

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
    node = MAVROSVisualControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()