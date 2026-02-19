from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    repo = os.path.expanduser('~/go2-convex-mpc')
    xml_path = os.path.join(repo, 'models', 'MJCF', 'go2', 'scene.xml')

    return LaunchDescription([
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
                'pub_hz': 500.0,
                'debug_print': True,
                'debug_publish': False,
                'foot_force_lpf_alpha': 0.0,
                'imu_gyro_noise_std': 0.005,
                'imu_acc_noise_std': 0.1,
                'imu_noise_seed': 1,
            }],
        )
    ])
