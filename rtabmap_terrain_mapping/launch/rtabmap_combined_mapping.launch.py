#!/usr/bin/env python3
"""
Combined RTAB-Map launch for comprehensive terrain mapping.
Uses RGB-D and point cloud from ZED camera (no laser scan).

Run after:
  ros2 launch rover_bringup simulated_robot.launch.py world_name:=mars_yard world_extension:=.sdf
  ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_navsat.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rtabmap_terrain_mapping')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    rtabmap_viz_arg = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map viz (specialized SLAM debugging)'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 (general visualization)'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/local',
        description='Odometry topic (odometry/local from EKF odom, odometry/global from EKF map)'
    )
    
    # Config path
    rtabmap_config = PathJoinSubstitution([pkg_dir, 'config', 'rtabmap_params.yaml'])
    
    # Combined RTAB-Map with RGB-D and point cloud
    rtabmap_combined = Node(
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
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'approx_sync': True,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
                # Topic parameters instead of remappings for odom
                'odom_topic': LaunchConfiguration('odom_topic'),
            }
        ],
        remappings=[
            ('rgb/image', '/zed/zed_node/rgb/color/rect/image'),
            ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/image/camera_info'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
            ('scan_cloud', '/zed/zed_node/point_cloud/cloud_registered'),
            # Remap all outputs to rtabmap namespace to avoid collisions
            ('cloud_map', '/rtabmap/cloud_map'),
            ('cloud_ground', '/rtabmap/cloud_ground'),
            ('cloud_obstacles', '/rtabmap/cloud_obstacles'),
            ('grid_map', '/rtabmap/grid_map'),
            ('grid_prob_map', '/rtabmap/grid_prob_map'),
            ('map', '/rtabmap/map'),
            ('mapData', '/rtabmap/mapData'),
            ('mapGraph', '/rtabmap/mapGraph'),
            ('mapPath', '/rtabmap/mapPath'),
            ('localization_pose', '/rtabmap/localization_pose'),
        ],
        arguments=['--delete_db_on_start']
    )
    
    # Point Cloud Map Assembler - assembles point clouds over time
    point_cloud_map = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'assembling_time': 5.0,
            'fixed_frame_id': 'map',
            'voxel_size': 0.05,
        }],
        remappings=[
            ('cloud', '/zed/zed_node/point_cloud/cloud_registered'),
            ('odom', LaunchConfiguration('odom_topic')),
            ('assembled_cloud', '/rtabmap/assembled_cloud'),
        ]
    )
    
    # RTAB-Map visualization (specialized SLAM debugging)
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('rtabmap_viz'))
    )
    
    # RViz2 config path
    rviz_config = PathJoinSubstitution([pkg_dir, 'rviz', 'rtabmap_visualization.rviz'])
    
    # RViz2 (general visualization)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Terrain processor
    terrain_processor = Node(
        package='rtabmap_terrain_mapping',
        executable='terrain_processor.py',
        name='terrain_processor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Point cloud handler
    point_cloud_handler = Node(
        package='rtabmap_terrain_mapping',
        executable='point_cloud_handler.py',
        name='point_cloud_handler',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Mapping coordinator
    mapping_coordinator = Node(
        package='rtabmap_terrain_mapping',
        executable='mapping_node.py',
        name='mapping_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_3d_mapping': True,
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        rtabmap_viz_arg,
        rviz_arg,
        odom_topic_arg,
        rtabmap_combined,
        point_cloud_map,
        rtabmap_viz_node,
        rviz_node,
        terrain_processor,
        point_cloud_handler,
        mapping_coordinator,
    ])