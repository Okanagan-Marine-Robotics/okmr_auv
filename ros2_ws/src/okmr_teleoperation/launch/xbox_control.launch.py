from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_params = PathJoinSubstitution(
        [FindPackageShare('okmr_teleoperation'), 'params', 'joy_node.yaml']
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params],
            output='screen',
        ),
        Node(
            package='okmr_teleoperation',
            executable='teleop_node',
            name='teleop_node',
            parameters=[
                {'surge_scale': 1.0},   # m/s per full stick deflection
                {'sway_scale': 0.5},    # m/s
                {'heave_scale': 0.6},   # m/s
                {'yaw_scale': 10.0},    # rad/s
                {'roll_scale': 0.3},    # rad/s
                {'pitch_scale': -0.3},  # rad/s
            ],
            output='screen',
        ),
    ])
