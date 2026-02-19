from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
        ),
        Node(
            package='xbox_controller_bridge',
            executable='joy_locomotion_bridge',
            name='xbox_controller_bridge',
            output='screen',
            parameters=[{
                'enable_turbo_button': 2,
                'axis_x': 1,
                'axis_y': 2,
                'axis_yaw': 0,
                'axis_z': 3,
                'scale_x': 0.5,
                'scale_z_rate': 0.05,
                'scale_x_turbo': 1.0,
                'scale_z_rate_turbo': 0.1,
                'scale_y': 0.2,
                'scale_y_turbo': 0.4,
                'scale_yaw': 2.0,
                'scale_yaw_turbo': 4.0,
            }],
        ),
    ])
