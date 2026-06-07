from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    GroupAction,
    IncludeLaunchDescription,
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # The configs will be installed to share/okmr_automated_planner/state_machine_configs
    # by default use the dev folder
    pkg_share = get_package_share_directory("okmr_automated_planner")
    config_share_path = os.path.join(pkg_share, "state_machine_configs")
    navigation_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_navigation"), "launch"]
    )

    # Launch arguments
    config_share_path_arg = DeclareLaunchArgument(
        "config_share_path",
        default_value=config_share_path,
        description="Share directory to prepend to config_folder parameter. Usually shouldnt be touched unless you didnt install the config folder before launching (not recommended)",
    )

    config_folder_arg = DeclareLaunchArgument(
        "config_folder",
        default_value="prequal",
        description="Folder to use for configs within the share.",
    )

    root_config_arg = DeclareLaunchArgument(
        "root_config",
        default_value="root.yaml",
        description="Name of the root configuration file (relative to config_base_path/config_folder)",
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value="params.yaml",
        description="Name of the ros2 parameter file defining settings for state machines(relative to config_base_path/config_folder)",
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="Enable debug logging for automated_planner node",
    )

    # Create the automated planner node
    automated_planner_node = Node(
        package="okmr_automated_planner",
        executable="automated_planner",
        name="automated_planner",
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            {
                "config_base_path": PathJoinSubstitution(
                    [
                        LaunchConfiguration("config_share_path"),
                        LaunchConfiguration("config_folder"),
                    ]
                )
            },
            {"root_config": LaunchConfiguration("root_config")},
            PathJoinSubstitution(
                [
                    LaunchConfiguration("config_share_path"),
                    LaunchConfiguration("config_folder"),
                    LaunchConfiguration("param_file"),
                ]
            ),
        ],
        output="screen",
        ros_arguments=[
            "--log-level",
            PythonExpression(
                [
                    '"debug" if "',
                    LaunchConfiguration("debug"),
                    '" == "true" else "info"',
                ]
            ),
            "--log-level",
            "rcl:=warn",  # Suppress noisy RCL debug messages
            "--log-level",
            "rcl_action:=warn",  # Suppress RCL action client messages
            "--log-level",
            "rmw_fastrtps_cpp:=warn",  # Suppress FastRTPS sub topic messages
        ],
    )

    navigation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([navigation_dir, "full_navigation_stack.launch.py"])
    )

    # Include Full Control Stack Launch
    control_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("okmr_controls"),
                        "launch",
                        "full_control_stack.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "folder": "default",
        }.items(),
    )

    # Include Object Detection Launch
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("okmr_object_detection"),
                        "launch",
                        "full_object_detection_system.launch.py",
                    ]
                )
            ]
        )
    )

    # Include Hardware Interface Launch (includes DVL, ESP32 bridge, temp sensor)
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("okmr_hardware_interface"),
                        "launch",
                        "ogopogo_hardware_interface.launch.py",
                    ]
                )
            ]
        )
    )

    # Set colorized output for better log readability
    colorized_output = SetEnvironmentVariable(
        name="RCUTILS_COLORIZED_OUTPUT", value="1"
    )

    ogopogo_dir = PathJoinSubstitution([FindPackageShare("ogopogo"), "launch"])

    cameras_launch = IncludeLaunchDescription(
        PathJoinSubstitution([ogopogo_dir, "cameras.launch.py"])
    )

    # Return the launch description
    return LaunchDescription(
        [
            navigation_launch,
            colorized_output,
            config_share_path_arg,
            config_folder_arg,
            param_file_arg,
            root_config_arg,
            debug_arg,
            automated_planner_node,
            control_stack_launch,
            cameras_launch,
            #object_detection_launch,
            hardware_interface_launch,
        ]
    )
