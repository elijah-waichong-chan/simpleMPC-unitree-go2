from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

import os


def generate_launch_description():
    repo = os.path.expanduser('~/go2-convex-mpc')
    xml_path = os.path.join(repo, 'models', 'MJCF', 'go2', 'scene.xml')

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT',
            '[{severity}] [{name}]: {message}'
        ),
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
        Node(
            package='mujoco_robot',
            executable='mujoco_robot_node',
            name='mujoco_robot_node',
            output='screen',
            parameters=[{
                'xml_path': xml_path,
                'freeze_base': False,
                'enable_viewer': True,
                'render_hz': 30.0,
                'sim_hz': 500.0,
                'pub_hz': 250.0,
            }],
        ),
        Node(
            package='contact_force_mpc',
            executable='mpc_node',
            name='mpc_node',
            output='screen',
            parameters=[{
                'cost_q': [1.0, 1.0, 100.0,  10.0, 20.0, 1.0,   2.0, 2.0, 1.0,   1.0, 1.0, 1.0],
            }],
        ),
        Node(
            package='locomotion_controller',
            executable='locomotion_node',
            name='locomotion_controller',
            output='screen',
            parameters=[{
                'ctrl_hz': 250.0,
                'swing_kp': 350.0, 
                'swing_kd': 60.0,
                'ground_offset': 0.02,
                'swing_height': 0.05,
                'stance_force_min': 1.0,
                'stance_fallback_force_z': 30.0,
            }],
        )
    ])
