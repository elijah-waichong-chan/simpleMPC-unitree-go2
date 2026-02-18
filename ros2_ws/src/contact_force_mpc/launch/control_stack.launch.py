from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    go2_odom_launch = (
        get_package_share_directory('go2_odometry')
        + '/launch/go2_inekf_odometry.launch.py'
    )
    state_topic = LaunchConfiguration('state_topic')

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT',
            '[{severity}] [{name}]: {message}'
        ),
        DeclareLaunchArgument(
            'state_topic',
            default_value='/qdq',
            description='State topic for MPC/locomotion (use /qdq or /qdq_est)'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(go2_odom_launch),
        ),
        Node(
            package='locomotion_controller',
            executable='stand_up_init',
            name='stand_up_init',
            output='screen',
        ),
        Node(
            package='estimator_bridge',
            executable='qdq_est_bridge',
            name='qdq_est_bridge',
            output='screen',
            parameters=[{
                'odom_topic': '/odometry/filtered',
                'joint_states_topic': '/joint_states',
                'qdq_topic': '/qdq_est',
            }],
        ),
        Node(
            package='contact_force_mpc',
            executable='mpc_node',
            name='mpc_node',
            output='screen',
            parameters=[{
                'cost_q': [1.0, 1.0, 100.0,  10.0, 20.0, 1.0,   2.0, 2.0, 1.0,   1.0, 1.0, 1.0],
                'debug_publish': False,
                'qdq_topic': state_topic,
            }],
        ),
        Node(
            package='locomotion_controller',
            executable='locomotion_node',
            name='locomotion_controller',
            output='screen',
            parameters=[{
                'ctrl_hz': 250.0,
                'swing_kp': 400.0,
                'swing_kd': 75.0,
                'ground_offset': 0.02,
                'swing_height': 0.1,
                'stance_force_min': 1.0,
                'stance_fallback_force_z': 35.0,
                'qdq_topic': state_topic,
            }],
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
    ])
