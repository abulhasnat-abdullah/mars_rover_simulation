#!/usr/bin/env python3

"""
ENU (East-North-Up) Localization Launch File
=============================================
True global localization using GPS in ENU coordinate frame.

This launch file provides:
1. Dual EKF Setup:
   - EKF Local: odom->base_footprint (wheels+IMU for smooth odometry)
   - EKF Global: map->base_footprint (wheels+IMU+GPS for global pose)
2. GPS Fusion: navsat_transform converts GPS to map frame
3. NO AMCL: GPS provides the map->odom transform directly

FRAME HIERARCHY:
- map (ENU frame, origin at GPS datum)
  └─ odom (local odometry frame)
     └─ base_footprint (robot frame)

Use this for outdoor navigation with global GPS positioning.
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    # Config paths
    localization_dir = get_package_share_directory("rover_localization")
    ekf_local_config = os.path.join(localization_dir, "config", "ekf_local.yaml")
    ekf_global_config = os.path.join(localization_dir, "config", "ekf_global_enu.yaml")
    
    # ============ EKF LOCAL (odom->base_footprint) ============
    # Provides smooth continuous odometry from wheels + IMU
    # This is your "local" odometry that doesn't drift too much in short term
    
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_filter_node",
        output="screen",
        parameters=[
            ekf_local_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/local')
        ]
    )
    
    # ============ NAVSAT TRANSFORM ============
    # Converts GPS (lat/lon/alt) to Cartesian coordinates in map frame (ENU)
    # Publishes: /odometry/gps (GPS position in map frame)

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            ekf_global_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('imu', 'imu/out'),
            ('gps/fix', 'mobile_sensor/gps'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global')  # Uses global EKF output
        ]
    )
    
    # ============ EKF GLOBAL (map->base_footprint) ============
    # Fuses wheels + IMU + GPS for global position in ENU frame
    # Publishes map->base_footprint transform
    # This provides your position in the global ENU coordinate system

    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_filter_node",
        output="screen",
        parameters=[
            ekf_global_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/global')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,
    ])