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
            package='mujoco_robot',
            executable='mujoco_robot_node',
            name='mujoco_robot_node',
            output='screen',
            parameters=[{
                'xml_path': xml_path,
                'freeze_base': True,
                'enable_viewer': True,
                'render_hz': 30.0,
            }],
        ),
        Node(
            package='contact_force_mpc',
            executable='mpc_node',
            name='mpc_node',
            output='screen',
            additional_env={
                'PYTHONPATH': os.path.join(repo, 'src') + ':' + os.environ.get('PYTHONPATH', '')
            }
        ),
        Node(
            package='contact_force_mpc',
            executable='contact_force_mpc',
            name='contact_force_mpc',
            output='screen',
            additional_env={
                'PYTHONPATH': os.path.join(repo, 'src') + ':' + os.environ.get('PYTHONPATH', '')
            }
        )
    ])
