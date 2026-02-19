from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch.substitutions import LaunchConfiguration # type: ignore
from launch_ros.actions import Node # type: ignore
from ament_index_python.packages import get_package_share_directory # type: ignore


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
            default_value='/qdq_est',
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
                'cost_q': [1.0, 1.0, 50.0,  10.0, 50.0, 1.0,   2.0, 2.0, 1.0,   1.0, 1.0, 1.0],
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
                'gait_hz': 3.0,
                'gait_duty': 0.6,
                'ctrl_hz': 250.0,
                'swing_kp': 400.0,
                'swing_kd': 75.0,
                'ground_offset': 0.02,
                'swing_height': 0.05,
                'stance_force_min': 0.0,
                'stance_fallback_force_z': 0.0,
                'joint_vel_limit': 30.0,
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
                'scale_y': 0.3,
                'scale_y_turbo': 0.6,
                'scale_yaw': 2.0,
                'scale_yaw_turbo': 4.0,
            }],
        ),
    ])
