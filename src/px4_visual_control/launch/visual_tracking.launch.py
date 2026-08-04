from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='balloon_detector',
            executable='hsv_detector_node',
            name='balloon_detector',
            output='screen'
        ),

        Node(
            package='px4_visual_control',
            executable='control_node',
            name='mavros_visual_control',
            output='screen',
            parameters=[{
                'forward_pitch_deg': -2.0,
                'roll_gain_deg': 3.0,
                'max_roll_deg': 3.0,
                'vertical_gain': 0.03,
                'neutral_thrust': 0.5,
                'min_thrust': 0.47,
                'max_thrust': 0.53,
                'search_yaw_rate_deg_s': 15.0,
                'horizontal_deadband': 0.05,
                'vertical_deadband': 0.05,
                'lost_timeout': 0.3,
            }]
        )
    ])