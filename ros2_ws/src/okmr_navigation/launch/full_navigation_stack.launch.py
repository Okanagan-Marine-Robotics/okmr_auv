from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():

    navigation_dir = get_package_share_directory("okmr_navigation")
    dead_reckoning_params = os.path.join(
        navigation_dir, "params", "dead_reckoning.yaml"
    )
    static_transforms_launch = os.path.join(
        navigation_dir, "launch", "static_transforms.launch.py"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                static_transforms_launch,
            ),
            Node(
                package="okmr_navigation",
                executable="dead_reckoning",
                remappings=[
                    ("/imu", "/ogopogo/front/imu"),
                ],
                parameters=[dead_reckoning_params],
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
