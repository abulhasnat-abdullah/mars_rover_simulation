#!/usr/bin/env python3

"""
Global Localization Launch File - UPDATED
==========================================
Conditional EKF for sensor fusion with AMCL.

This launch file provides:
1. Map Server: Loads your pre-built map
2. AMCL: Adaptive Monte Carlo Localization using laser scans
3. EKF: Extended Kalman Filter (single node, configured based on GPS availability)
4. GPS Fusion: navsat_transform + EKF with GPS (when enabled)
5. Lifecycle Manager: Manages node lifecycle

MODES:
- use_gps:=false → Single EKF (wheels+IMU), AMCL provides map correction
- use_gps:=true  → EKF with GPS fusion, smoother global localization

Use this for navigation with a known map.
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    use_gps = LaunchConfiguration("use_gps")
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("rover_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )
    
    use_gps_arg = DeclareLaunchArgument(
        "use_gps",
        default_value="false",
        description="Enable GPS fusion for outdoor navigation",
        choices=["true", "false"]
    )

    # Map path
    map_path = PathJoinSubstitution([
        get_package_share_directory("rover_mapping"),
        "maps",
        "map.yaml"
    ])
    
    # Config paths
    localization_dir = get_package_share_directory("rover_localization")
    ekf_local_config = os.path.join(localization_dir, "config", "ekf_local.yaml")
    ekf_gps_config = os.path.join(localization_dir, "config", "ekf_gps.yaml")
    
    # ============ MAP SERVER ============
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    # ============ AMCL ============
    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ============ EKF WITHOUT GPS ============
    # Single EKF node: fuses wheels + IMU
    # Publishes: odom->base_footprint
    # AMCL provides: map->odom (from laser scans)
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_local_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/local')
        ],
        condition=UnlessCondition(use_gps)
    )
    
    # ============ GPS FUSION NODES ============
    # When GPS enabled: Different strategy
    # EKF fuses wheels + IMU + GPS for global positioning
    
    # Navsat Transform - converts GPS lat/lon to map frame odometry
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            ekf_gps_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('imu', 'imu/out'),     #in case use zed imu 'zed/zed_node/imu/data'
            ('gps/fix', 'mobile_sensor/gps'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/local')  # Uses EKF output for heading
        ],
        condition=IfCondition(use_gps)
    )
    
    # EKF with GPS - fuses wheels + IMU + GPS
    # Publishes: odom->base_footprint (with GPS correction)
    # AMCL still provides: map->odom (for local refinement)
    ekf_gps_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_gps_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/local')
        ],
        condition=IfCondition(use_gps)
    )

    # ============ LIFECYCLE MANAGER ============
    lifecycle_nodes = ["map_server", "amcl"]
    
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        amcl_config_arg,
        use_gps_arg,
        nav2_map_server,
        nav2_amcl,
        ekf_local_node,        # Runs when GPS disabled (wheels+IMU only)
        navsat_transform_node, # Runs when GPS enabled (GPS converter)
        ekf_gps_node,          # Runs when GPS enabled (wheels+IMU+GPS)
        nav2_lifecycle_manager,
    ])