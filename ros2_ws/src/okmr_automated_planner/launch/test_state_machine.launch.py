from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    automated_planning_dir = PathJoinSubstitution([FindPackageShare('okmr_automated_planner'), 'launch'])
    navigation_dir = PathJoinSubstitution([FindPackageShare('okmr_navigation'), 'launch'])
    controls_dir = PathJoinSubstitution([FindPackageShare('okmr_controls'), 'launch'])
    stonefish_dir = PathJoinSubstitution([FindPackageShare('okmr_stonefish'), 'launch'])

    return LaunchDescription([
        # Launch the Stonefish simulation
        IncludeLaunchDescription(
            PathJoinSubstitution([stonefish_dir, 'sim.launch.py']),
            launch_arguments={
                'scenario_name': 'simple.scn',
            }.items()
        ),

        # Launch the full control stack
        IncludeLaunchDescription(
            PathJoinSubstitution([controls_dir, 'full_control_stack.launch.py'])
        ),
               
        # Launch the full navigation stack
        IncludeLaunchDescription(
            PathJoinSubstitution([navigation_dir, 'full_navigation_stack.launch.py'])
        ),
        
        # Launch the automated planner with test state machine
        IncludeLaunchDescription(
            PathJoinSubstitution([automated_planning_dir, 'automated_planner.launch.py']),
            launch_arguments={
                'debug': 'true',
                'root_config': 'test.yaml',
                'config_folder': 'testing',
                'param_file': 'params.yaml'
            }.items()
        ),
    ])
