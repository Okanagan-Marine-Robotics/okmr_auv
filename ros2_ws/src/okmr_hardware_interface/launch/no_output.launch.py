from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get path to parameter file
    package_dir = get_package_share_directory("okmr_hardware_interface")
    mega_params_file = os.path.join(package_dir, "params", "mega_params.yaml")
    thrust_curve_file = os.path.join(package_dir, "tables", "t200.csv")

    return LaunchDescription(
        [
            Node(
                package="okmr_hardware_interface",
                executable="dvl_driver",
            ),
            Node(
                package="okmr_hardware_interface",
                executable="thrust_to_pwm_node",
                parameters=[{"thrust_curve_file": thrust_curve_file}],
            ),
            Node(
                package="okmr_hardware_interface",
                executable="temp_sensor",
            ),
        ]
    )
