from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_name',
            default_value='simple.scn',
            description='scenario file to use'
        ),
        Node(
            package='stonefish_ros2',
            executable='stonefish_simulator',
            namespace='stonefish_ros2',
            name='stonefish_simulator',
            arguments=[
                PathJoinSubstitution([FindPackageShare('okmr_stonefish'), 'data']),
                PathJoinSubstitution([FindPackageShare('okmr_stonefish'),'data', 'scenarios', LaunchConfiguration('scenario_name')]),
                '200.0',
                '1200',
                '800',
                'high'
            ],
            output='log',
            remappings=[
                ('/stonefish/imu', '/camera1/camera1/imu'),
                ('/stonefish/front_camera/image_raw/image_color', '/camera1/camera1/image_raw'),
                ('/stonefish/front_camera/image_raw/camera_info', '/camera1/camera1/camera_info'),
                ('/stonefish/front_depth_camera/depth/image_depth', '/camera1/camera1/image_depth'),
                ('/stonefish/front_depth_camera/depth/camera_info', '/camera1/camera1/depth_camera_info'),
                ('/stonefish/down_camera/image_raw/image_color', '/camera2/camera2/image_raw'),
                ('/stonefish/down_camera/image_raw/camera_info', '/camera2/camera2/camera_info'),
                ('/stonefish/down_depth_camera/depth/image_depth', '/camera2/camera2/image_depth'),
                ('/stonefish/down_depth_camera/depth/camera_info', '/camera2/camera2/depth_camera_info'),
            ]
        ),
        Node(
            package='okmr_stonefish',
            executable='sim_adaptor',
        ),
        
        # Include static transforms for coordinate frame conversion
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('okmr_stonefish'),
                    'launch',
                    'static_transforms.launch.py'
                ])
            ])
        ),

    ])
