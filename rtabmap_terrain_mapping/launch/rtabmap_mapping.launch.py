#!/usr/bin/env python3
"""
Main RTAB-Map launch file for terrain mapping in simulation.
Uses ZED point cloud only (no laser scan).
Integrates with your rover_ws workspace.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rtabmap_terrain_mapping')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Run in localization mode (use existing map)'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RTAB-Map viz'
    )
    
    use_3d_arg = DeclareLaunchArgument(
        'use_3d',
        default_value='true',
        description='Enable 3D mapping'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='odometry/local',
        description='Odometry topic (odometry/local from EKF odom, odometry/global from EKF map)'
    )
    
    # Config file path
    rtabmap_config = PathJoinSubstitution([pkg_dir, 'config', 'rtabmap_params.yaml'])
    
    # RTAB-Map node - MAPPING MODE (point cloud only)
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Point cloud only - no laser scan
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'queue_size': 10,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
            }
        ],
        remappings=[
            # ZED camera topics
            ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
            # Point cloud instead of laser scan
            ('scan_cloud', '/zed/zed_node/depth/depth_registered/points'),
            # Odometry from your EKF
            ('odom', LaunchConfiguration('odom_topic')),
        ],
        arguments=['--delete_db_on_start'],
        condition=UnlessCondition(LaunchConfiguration('localization'))
    )
    
    # RTAB-Map node - LOCALIZATION MODE
    rtabmap_localization_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                # Localization specific
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
            }
        ],
        remappings=[
            ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
            ('scan_cloud', '/zed/zed_node/depth/depth_registered/points'),
            ('odom', LaunchConfiguration('odom_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('localization'))
    )
    
    # RTAB-Map visualization
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Terrain processor
    terrain_processor_node = Node(
        package='rtabmap_terrain_mapping',
        executable='terrain_processor.py',
        name='terrain_processor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'min_height': -0.5,
            'max_height': 1.0,
        }]
    )
    
    # Point cloud handler
    point_cloud_handler_node = Node(
        package='rtabmap_terrain_mapping',
        executable='point_cloud_handler.py',
        name='point_cloud_handler',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Mapping coordinator
    mapping_node = Node(
        package='rtabmap_terrain_mapping',
        executable='mapping_node.py',
        name='rtabmap_mapping_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_3d_mapping': LaunchConfiguration('use_3d'),
        }]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        localization_arg,
        rviz_arg,
        use_3d_arg,
        odom_topic_arg,
        
        # Nodes
        rtabmap_node,
        rtabmap_localization_node,
        rtabmap_viz_node,
        terrain_processor_node,
        point_cloud_handler_node,
        mapping_node,
    ])
