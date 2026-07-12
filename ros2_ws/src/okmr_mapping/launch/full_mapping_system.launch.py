#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description="Whether to run in simulation mode (true) or real hardware mode (false)",
    )

    rgb_topic = PythonExpression(
        ["'/camera1/camera1/image_raw' if '", LaunchConfiguration("sim_mode"), "' == 'true' else '/camera/camera/color/image_raw'"]
    )
    depth_topic = PythonExpression(
        ["'/camera1/camera1/image_depth' if '", LaunchConfiguration("sim_mode"), "' == 'true' else '/camera/camera/depth/image_rect_raw'"]
    )
    camera_info_topic = PythonExpression(
        ["'/camera1/camera1/depth_camera_info' if '", LaunchConfiguration("sim_mode"), "' == 'true' else '/camera/camera/depth/camera_info'"]
    )

    # Launch front facing depth_to_pointcloud node
    front_depth_to_pointcloud_node = Node(
        package="okmr_mapping",
        executable="depth_to_pointcloud",
        name="front_depth_to_pointcloud_node",
        output="screen",
        parameters=[
            {
                "max_dist": 6.0,
                "min_dist": 0.0,
                "use_mask": False,
                "use_rgb": True,
            }
        ],
        remappings=[
            ("/rgb", rgb_topic),
            ("/depth", depth_topic),
            ("/mask", "/labeled_image"),
            ("/camera_info", camera_info_topic),
            ("/pointcloud", "/camera1/pointcloud"),
        ],
    )

    bottom_depth_to_pointcloud_node = Node(
        package="okmr_mapping",
        executable="depth_to_pointcloud",
        name="bottom_depth_to_pointcloud_node",
        output="screen",
        parameters=[
            {
                "max_dist": 2.0,
                "min_dist": 0.07,
                "use_mask": False,
                "use_rgb": True,
            }
        ],
        remappings=[
            ("/rgb", "/camera2/camera2/image_raw"),
            ("/depth", "/camera2/camera2/image_depth"),
            ("/camera_info", "/camera2/camera2/depth_camera_info"),
            ("/pointcloud", "/camera2/pointcloud"),
        ],
    )

    return LaunchDescription(
        [
            sim_mode_arg,
            front_depth_to_pointcloud_node,
            # bottom_depth_to_pointcloud_node,
        ]
    )
