from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'require_enable_button': False,
                'enable_turbo_button': 2,
                'axis_linear.x': 1,
                'scale_linear.x': 0.5,
                'scale_linear_turbo.x': 1.0,
                'axis_angular.yaw': 0,
                'scale_angular.yaw': 2.0,
                'scale_angular_turbo.yaw': 4.0,
            }],
        ),
    ])
