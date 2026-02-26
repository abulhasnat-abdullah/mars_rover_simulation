from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    cloud_in_topic = LaunchConfiguration('cloud_in_topic')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'cloud_in_topic',
            default_value='/zed/zed_node/depth/depth_registered/points',
            description='Input point cloud topic'
        ),



        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', cloud_in_topic),
                ('scan', '/scan_converted')
            ],
            parameters=[{
                'target_frame': 'zed_camera_center', 
                'transform_tolerance': 0.1,  # Increased for reliability
                'min_height': -0.3,
                'max_height': 1.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.5,    # Reduced from 0.8
                'range_max': 80.0,   # Match SLAM config
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        )
    ])