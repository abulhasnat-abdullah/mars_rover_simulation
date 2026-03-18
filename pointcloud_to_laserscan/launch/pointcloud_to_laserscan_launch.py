from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'cloud_in_topic',
            default_value='/zed/zed_node/point_cloud/cloud_registered',
            description='Input point cloud topic'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
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
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'target_frame': 'laser_link',  # Changed to match exactly where your lidar is mounted
                'transform_tolerance': 0.1,  
                'queue_size': 1,
                'min_height': -0.05,         # 5cm below the lidar
                'max_height': 0.05,          # 5cm above the lidar
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