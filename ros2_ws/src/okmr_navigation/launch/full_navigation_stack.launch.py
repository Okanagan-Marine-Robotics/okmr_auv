from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
import os


def generate_launch_description():

    navigation_dir = get_package_share_directory("okmr_navigation")
    dead_reckoning_params = os.path.join(
        navigation_dir, "params", "dead_reckoning.yaml"
    )
    static_transforms_launch = os.path.join(
        navigation_dir, "launch", "static_transforms.launch.py"
    )

    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description="Whether to run in simulation mode (true) or real hardware mode (false)",
    )

    imu_topic = PythonExpression(
        ["'/camera1/camera1/imu' if '", LaunchConfiguration("sim_mode"), "' == 'true' else '/ogopogo/front/imu'"]
    )

    return LaunchDescription(
        [
            sim_mode_arg,
            IncludeLaunchDescription(
                static_transforms_launch,
            ),
            Node(
                package="okmr_navigation",
                executable="dead_reckoning",
                remappings=[
                    ("/imu", imu_topic),
                ],
                parameters=[],
            ),
            Node(
                package="okmr_navigation",
                executable="navigator_action_server",
            ),
            Node(
                package="okmr_navigation",
                executable="relative_pose_target_server",
            ),
            Node(
                package="okmr_navigation",
                executable="velocity_target_server",
            ),
        ]
    )
