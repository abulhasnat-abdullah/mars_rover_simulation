#!/usr/bin/env python3
"""
2D RTAB-Map launch for terrain mapping.
Uses depth image projected to 2D grid (no laser scan required).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rtabmap_terrain_mapping')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='odometry/local',
        description='Odometry topic (odometry/local from EKF odom, odometry/global from EKF map)'
    )
    
    # Config path
    rtabmap_config = PathJoinSubstitution([pkg_dir, 'config', 'rtabmap_params.yaml'])
    
    # RTAB-Map 2D mode using depth for grid
    rtabmap_2d = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_2d',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'subscribe_scan_cloud': False,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'approx_sync': True,
            # 2D specific - generate grid from depth
            'Grid/FromDepth': 'true',
            'Grid/3D': 'false',
            'Grid/CellSize': '0.05',
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '10.0',
            'Grid/MaxGroundAngle': '45.0',
            # Visual registration
            'Reg/Strategy': '0',
            'Reg/Force3DoF': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            'Mem/IncrementalMemory': 'true',
        }],
        remappings=[
            ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
            ('odom', LaunchConfiguration('odom_topic')),
        ],
        arguments=['--delete_db_on_start']
    )
    
    # Terrain processor for 2D
    terrain_processor = Node(
        package='rtabmap_terrain_mapping',
        executable='terrain_processor.py',
        name='terrain_processor_2d',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        odom_topic_arg,
        rtabmap_2d,
        terrain_processor,
    ])
