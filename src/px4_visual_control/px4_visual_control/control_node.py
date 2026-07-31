#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
import math

class PX4VisualControl(Node):
    def __init__(self):
        super().__init__('px4_visual_control')
        
        # 参数
        self.declare_parameter('takeoff_relative_altitude', 1.0)
        self.declare_parameter('proportional_gain', 0.8)
        self.declare_parameter('vertical_gain', 0.5)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_vertical_speed', 1.0)
        
        self.takeoff_alt = self.get_parameter('takeoff_relative_altitude').value
        self.kp = self.get_parameter('proportional_gain').value
        self.kv = self.get_parameter('vertical_gain').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_vspeed = self.get_parameter('max_vertical_speed').value
        
        # 状态
        self.altitude = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.object_found = False
        self.lost_counter = 0
        self.state = 'IDLE'  # IDLE -> TAKEOFF -> TRACKING -> LOST
        
        # 订阅
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, 10)
        self.dev_sub = self.create_subscription(Float32MultiArray, '/camera/image_deviation', self.deviation_callback, 10)
        
        # 发布
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # 50Hz 控制循环
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('✅ 视觉控制节点已启动')
    
    def odom_callback(self, msg):
        self.altitude = msg.position[2]
    
    def deviation_callback(self, msg):
        if len(msg.data) >= 3:
            self.object_found = msg.data[2] > 0.5
            if self.object_found:
                self.dx = msg.data[0]
                self.dy = msg.data[1]
                self.lost_counter = 0
            else:
                self.lost_counter += 1
    
    def control_loop(self):
        # 状态机
        if self.state == 'IDLE':
            self.arm()
            self.offboard()
            if self.lost_counter < 5:  # 随便找个条件切状态
                self.state = 'TAKEOFF'
                self.get_logger().info('🚀 起飞')
        
        elif self.state == 'TAKEOFF':
            if self.altitude < -self.takeoff_alt * 0.9:
                self.state = 'TRACKING'
                self.get_logger().info('🎯 到达高度，开始追踪')
            self.publish_trajectory(0, 0, 0.5, 0)
        
        elif self.state == 'TRACKING':
            if self.object_found:
                vx = self.kp * self.dx
                vy = self.kp * self.dy
                speed = math.sqrt(vx**2 + vy**2)
                if speed > self.max_speed:
                    vx = vx / speed * self.max_speed
                    vy = vy / speed * self.max_speed
                
                vz = self.kv * (self.altitude + self.takeoff_alt)
                vz = max(-self.max_vspeed, min(self.max_vspeed, vz))
                
                self.publish_trajectory(vx, vy, vz, 0)
            else:
                self.lost_counter += 1
                if self.lost_counter > 30:
                    self.state = 'LOST'
                    self.get_logger().warn('⚠️ 目标丢失')
                self.publish_trajectory(0, 0, 0, 0)
        
        elif self.state == 'LOST':
            self.publish_trajectory(0, 0, 0, 0)
            if self.object_found:
                self.state = 'TRACKING'
                self.get_logger().info('🎯 重新捕获')
    
    def publish_trajectory(self, vx, vy, vz, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float('nan')] * 3
        msg.velocity = [vx, vy, vz]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)
    
    def arm(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)
        self.get_logger().info('🔓 Arm 指令已发送')
    
    def offboard(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4VisualControl()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
