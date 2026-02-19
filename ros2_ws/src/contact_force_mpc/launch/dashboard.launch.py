from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="contact_force_mpc",
            executable="launch_manager",
            name="launch_manager",
            output="screen",
        ),
        Node(
            package="telemetry_dashboard",
            executable="telemetry_dashboard",
            name="telemetry_dashboard",
            output="screen",
        ),
    ])
