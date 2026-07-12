from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    automated_planning_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_automated_planner"), "launch"]
    )
    navigation_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_navigation"), "launch"]
    )
    controls_dir = PathJoinSubstitution([FindPackageShare("okmr_controls"), "launch"])
    stonefish_dir = PathJoinSubstitution([FindPackageShare("okmr_stonefish"), "launch"])
    mapping_dir = PathJoinSubstitution([FindPackageShare("okmr_mapping"), "launch"])
    object_detection_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_object_detection"), "launch"]
    )
    realsense_launch_dir = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch"]
    )
    hardware_interface_launch_dir = PathJoinSubstitution(
        [FindPackageShare("okmr_hardware_interface"), "launch"]
    )

    # Launch file paths
    sim_launch = PathJoinSubstitution([stonefish_dir, "sim.launch.py"])
    full_control_stack_launch = PathJoinSubstitution(
        [controls_dir, "full_control_stack.launch.py"]
    )
    full_navigation_stack_launch = PathJoinSubstitution(
        [navigation_dir, "full_navigation_stack.launch.py"]
    )
    full_object_detection_system_launch = PathJoinSubstitution(
        [object_detection_dir, "full_object_detection_system.launch.py"]
    )
    full_mapping_system_launch = PathJoinSubstitution(
        [mapping_dir, "full_mapping_system.launch.py"]
    )
    automated_planner_launch = PathJoinSubstitution(
        [automated_planning_dir, "automated_planner.launch.py"]
    )
    rs_multi_camera_launch = PathJoinSubstitution(
        [realsense_launch_dir, "rs_multi_camera_launch.py"]
    )
    ogopogo_hardware_interface_launch = PathJoinSubstitution(
        [hardware_interface_launch_dir, "ogopogo_hardware_interface.launch.py"]
    )

    # Declare launch arguments
    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description="Whether to run in simulation mode (true) or real hardware mode (false)",
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="Enable debug mode for all nodes that use it",
    )

    control_params_folder = PythonExpression(
        ["'sim' if '", LaunchConfiguration("sim_mode"), "' == 'true' else 'default'"]
    )

    root_config_arg = DeclareLaunchArgument(
        "root_config",
        default_value="test.yaml",
        description="Root state machine configuration file for automated planner",
    )

    config_folder_arg = DeclareLaunchArgument(
        "config_folder",
        default_value="testing",
        description="Configuration folder for automated planner",
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value="params.yaml",
        description="Parameter file for automated planner",
    )

    return LaunchDescription(
        [
            sim_mode_arg,
            debug_arg,
            root_config_arg,
            config_folder_arg,
            param_file_arg,
            IncludeLaunchDescription(
                sim_launch,
                launch_arguments={
                    "scenario_name": "simple.scn",
                }.items(),
                condition=IfCondition(LaunchConfiguration("sim_mode")),
            ),
            IncludeLaunchDescription(
                full_control_stack_launch,
                launch_arguments={
                    "folder": control_params_folder,
                }.items(),
            ),
            IncludeLaunchDescription(
                full_navigation_stack_launch,
                launch_arguments={
                    "sim_mode": LaunchConfiguration("sim_mode"),
                }.items(),
            ),
            IncludeLaunchDescription(
                full_object_detection_system_launch,
                launch_arguments={
                    "debug": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                full_mapping_system_launch,
                launch_arguments={
                    "use_semantic_subscriber": "false",
                    "sim_mode": LaunchConfiguration("sim_mode"),
                }.items(),
            ),
            IncludeLaunchDescription(
                automated_planner_launch,
                launch_arguments={
                    "debug": LaunchConfiguration("debug"),
                    "root_config": LaunchConfiguration("root_config"),
                    "config_folder": LaunchConfiguration("config_folder"),
                    "param_file": LaunchConfiguration("param_file"),
                }.items(),
            ),
            IncludeLaunchDescription(
                rs_multi_camera_launch,
                condition=UnlessCondition(LaunchConfiguration("sim_mode")),
            ),
            IncludeLaunchDescription(
                ogopogo_hardware_interface_launch,
                condition=UnlessCondition(LaunchConfiguration("sim_mode")),
            ),
        ]
    )
