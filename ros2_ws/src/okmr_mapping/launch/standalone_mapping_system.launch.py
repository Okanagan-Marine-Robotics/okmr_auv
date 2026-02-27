#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch front facing depth_to_pointcloud node
    ogopogo_dir = PathJoinSubstitution([FindPackageShare("ogopogo"), "launch"])
    cameras_launch = PathJoinSubstitution([ogopogo_dir, "cameras.launch.py"])
    
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
            ("/rgb", "/ogopogo/front/color/image_raw"),
            ("/depth", "/ogopogo/front/depth/image_rect_raw"),
            ("/mask", "/labeled_image"),
            ("/camera_info", "/ogopogo/front/depth/camera_info"),
            ("/pointcloud", "/front/pointcloud"),
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
            IncludeLaunchDescription(cameras_launch),
            front_depth_to_pointcloud_node,
            # bottom_depth_to_pointcloud_node,
        ]
    )
