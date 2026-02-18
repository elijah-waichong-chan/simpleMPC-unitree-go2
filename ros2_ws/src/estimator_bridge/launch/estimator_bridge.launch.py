from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    go2_odom_launch = os.path.join(
        get_package_share_directory("go2_odometry"),
        "launch",
        "go2_inekf_odometry.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(go2_odom_launch),
            ),
            Node(
                package="estimator_bridge",
                executable="qdq_est_bridge",
                name="qdq_est_bridge",
                output="screen",
                parameters=[{
                    "odom_topic": "/odometry/filtered",
                    "joint_states_topic": "/joint_states",
                    "qdq_topic": "/qdq_est",
                }],
            ),
        ]
    )
