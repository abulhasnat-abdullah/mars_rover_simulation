"""
Optimized Simulated Robot Launch File

Main entry point for the complete simulated rover system with optional SLAM mapping.
Uses longer delays to ensure controller_manager has time to fully initialize.

Usage:
    ros2 launch rover_description simulated_robot.launch.py
    ros2 launch rover_description simulated_robot.launch.py mapping:=true world_name:=mars
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, OrSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========== LAUNCH ARGUMENTS ==========
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time. Must be true for Gazebo."
    )

    use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        description="Whether to start RViz2 (used by gps_nav)."
    )
    
    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty",
        description="Gazebo world name (without extension)"
    )
    
    world_extension_arg = DeclareLaunchArgument(
        name="world_extension",
        default_value=".world",
        description="World file extension (.world or .sdf)"
    )
    
    mapping_arg = DeclareLaunchArgument(
        name="mapping",
        default_value="false",
        description="Enable SLAM mapping (launches localization and SLAM)"
    )

    navigation_arg = DeclareLaunchArgument(
        name="navigation",
        default_value="false",
        description="Enable Nav2 for Autonomous Navigation "
    )
    
    gps_nav_arg = DeclareLaunchArgument(
        name="gps_nav",
        default_value="false",
        description="Enable GPS Navigation"
    )

    rtab_arg = DeclareLaunchArgument(
        name="rtab",
        default_value="false",
        description="Enable RTAB-Map combined terrain mapping"
    )

    pc2ls_arg = DeclareLaunchArgument(
        name="pc2ls",
        default_value="true",
        description="Enable pointcloud_to_laserscan (publishes /scan_converted)"
    )
    # ========== COMPONENT LAUNCH FILES ==========
    
    # Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover_description"),
                "launch",
                "gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "world_name": LaunchConfiguration("world_name"),
            "world_extension": LaunchConfiguration("world_extension")
        }.items()
    )
    
    # Robot controllers
    # NOTE: If controller.launch.py contains spawners, they have built-in timeout
    # waiting for controller_manager. Longer delay here ensures manager is ready.
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover_controller"),
                "launch",
                "controller.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )
    
    # Joystick teleoperation
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover_controller"),
                "launch",
                "joystick_teleop.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )

    # Global Localization - only if mapping=false (SLAM disabled)
    global_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover_localization"),
                "launch",
                "global_localization.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation"))
    )
    
    
    # Localization - only if mapping=true
    local_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover_localization"),
                "launch",
                "local_localization.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=IfCondition(LaunchConfiguration("mapping"))
    )

    gps_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_gps_waypoint_follower"),
                "launch",
                "dual_ekf_navsat.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=IfCondition(OrSubstitution(LaunchConfiguration("gps_nav"), LaunchConfiguration("rtab")))
    )
    
    # SLAM - only if mapping=true
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover_mapping"),
                "launch",
                "slam.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=IfCondition(LaunchConfiguration("mapping"))
    )


    # Nav2
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover_navigation"),
                "launch",
                "navigation.launch.py"
            ])
        ),
        condition=IfCondition(LaunchConfiguration("navigation"))
    )


    gps_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_gps_waypoint_follower"),
                "launch",
                "gps_navigation.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("gps_nav"))
    )

    pointcloud_to_laserscan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("pointcloud_to_laserscan"),
                "launch",
                "pointcloud_to_laserscan_launch.py",
            ])
        ),
        launch_arguments={
            # This topic is already bridged from Gazebo in rover_description/launch/gazebo.launch.py
            "cloud_in_topic": "/zed/zed_node/point_cloud/cloud_registered",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("pc2ls"))
    )

    # RTAB-Map combined terrain mapping
    rtabmap_combined = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rtabmap_terrain_mapping"),
                "launch",
                "rtabmap_combined_mapping.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=IfCondition(LaunchConfiguration("rtab"))
    )

    # ========== TIMER-BASED SEQUENCING ==========
    # Now that controller.launch.py sequences properly, we can use shorter delays
    
    delayed_controller = TimerAction(
        period=10.0,  # Robot spawned at 8s in gazebo.launch.py
        actions=[controller]
    )
    
    delayed_teleop = TimerAction(
        period=18.0,  # Controllers fully spawned at ~15s (10s + 5s internal delay)
        actions=[joystick]
    )
    
    delayed_local_localization = TimerAction(
        period=14.0,
        actions=[local_localization]
    )
    
    delayed_global_localization = TimerAction(
        period=14.0,
        actions=[global_localization]
    )

    delayed_gps_localization = TimerAction(
        period=14.0,
        actions=[gps_localization]
    )
    
    delayed_slam = TimerAction(
        period=18.0,  # After localization stabilizes
        actions=[slam]
    )

    delayed_navigation = TimerAction(
        period=18.0,  # After localization stabilizes
        actions=[navigation]
    )

    delayed_gps_navigation = TimerAction(
        period=18.0,  # After localization stabilizes
        actions=[gps_navigation]
    )

    delayed_pointcloud_to_laserscan = TimerAction(
        period=18.0,  # After sensor bridges start (sensor_delay is 8s) and zed pointcloud is available
        actions=[pointcloud_to_laserscan]
    )

    delayed_rtabmap = TimerAction(
        period=20.0,  # After SLAM/localization stabilizes
        actions=[rtabmap_combined]
    )
    
    # ========== LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        use_rviz_arg,
        world_name_arg,
        world_extension_arg,
        mapping_arg,
        navigation_arg,
        gps_nav_arg,
        rtab_arg,
        pc2ls_arg,
        
        # Launch sequence
        gazebo,                          # t=0s: Simulation starts
        delayed_controller,              # t=10s: Controllers start (robot spawned at 8s)
        delayed_teleop,                  # t=18s: Teleoperation starts
        delayed_local_localization,      # t=14s: Localization starts (if mapping=true)
        delayed_global_localization,     # t=14s: Global Localization starts (if mapping=false and navigation=true)
        delayed_gps_localization,        # t=14s: GPS Localization starts (if gps_nav=true)
        delayed_slam,                    # t=18s: SLAM starts (if mapping=true)
        delayed_gps_navigation,
        delayed_pointcloud_to_laserscan,
        delayed_navigation,
        delayed_rtabmap,                 # t=20s: RTAB-Map starts (if rtab=true)
    ])  