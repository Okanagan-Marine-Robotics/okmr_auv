from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    realsense_launch_dir = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "multi_camera_launch"]
    )
    hardware_interface_launch_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_hardware_interface"), "launch"]
    )
    
    ogopogo_dir = PathJoinSubstitution([FindPackageShare("ogopogo"), "launch"])

    controls_dir = PathJoinSubstitution([FindPackageShare("okmr_controls"), "launch"])

    navigation_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_navigation"), "launch"]
    )

    cameras_launch = PathJoinSubstitution([ogopogo_dir, "cameras.launch.py"])

    ogopogo_hardware_interface_launch = PathJoinSubstitution(
        [hardware_interface_launch_dir, "ogopogo_hardware_interface.launch.py"]
    )

    navigation_launch = PathJoinSubstitution(
        [navigation_dir, "full_navigation_stack.launch.py"]
    )

    full_control_stack_launch = PathJoinSubstitution(
        [controls_dir, "full_control_stack.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(cameras_launch),
            IncludeLaunchDescription(
                full_control_stack_launch,
                launch_arguments={
                    "folder": "default",
                }.items(),
            ),
            IncludeLaunchDescription(navigation_launch),
            IncludeLaunchDescription(
                ogopogo_hardware_interface_launch,
            ),
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge'
            ),
        ]
    )
