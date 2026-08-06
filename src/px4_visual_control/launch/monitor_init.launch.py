from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def message_interval_process(message_id, message_rate):
    return ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/mavros/set_message_interval',
            'mavros_msgs/srv/MessageInterval',
            (
                f'{{message_id: {message_id}, '
                f'message_rate: {message_rate}}}'
            ),
        ],
        output='screen',
    )


def generate_launch_description():
    # 1. 启动 MAVROS
    mavros_launch = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'mavros',
            'px4.launch',
            'fcu_url:=/dev/ttyACM0:115200',
        ],
        output='screen',
    )

    # 2. 请求 SERVO_OUTPUT_RAW
    request_servo_output = message_interval_process(
        message_id=36,
        message_rate=10.0,
    )

    # 3. 请求 ATTITUDE
    request_attitude = message_interval_process(
        message_id=30,
        message_rate=20.0,
    )

    # 4. 请求 ATTITUDE_QUATERNION
    request_attitude_quaternion = message_interval_process(
        message_id=31,
        message_rate=20.0,
    )

    # 5. 请求 HIGHRES_IMU
    request_highres_imu = message_interval_process(
        message_id=105,
        message_rate=20.0,
    )

    # 6. 启动网页监控节点
    web_monitor = Node(
        package='px4_visual_control',
        executable='web_monitor_node',
        name='mavros_web_monitor',
        output='screen',
        parameters=[{
            'port': 8080,
        }],
    )

    return LaunchDescription([
        # 首先启动 MAVROS
        mavros_launch,

        # 等待 MAVROS 初始化，然后请求消息36。
        # 即使3秒后服务尚未出现，ros2 service call也会继续等待。
        TimerAction(
            period=3.0,
            actions=[request_servo_output],
        ),

        # 36完成后请求30
        RegisterEventHandler(
            OnProcessExit(
                target_action=request_servo_output,
                on_exit=[request_attitude],
            )
        ),

        # 30完成后请求31
        RegisterEventHandler(
            OnProcessExit(
                target_action=request_attitude,
                on_exit=[request_attitude_quaternion],
            )
        ),

        # 31完成后请求105
        RegisterEventHandler(
            OnProcessExit(
                target_action=request_attitude_quaternion,
                on_exit=[request_highres_imu],
            )
        ),

        # 所有消息频率设置完成后启动网页
        RegisterEventHandler(
            OnProcessExit(
                target_action=request_highres_imu,
                on_exit=[web_monitor],
            )
        ),
    ])