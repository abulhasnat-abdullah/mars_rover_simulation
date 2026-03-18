"""
Optimized Real Robot Launch File

Main entry point for the complete real rover system with proper sequencing.
Uses delays to ensure hardware interface and controllers initialize properly.

Usage:
    ros2 launch rover_bringup real_robot.launch.py
    ros2 launch rover_bringup real_robot.launch.py mapping:=true
    ros2 launch rover_bringup real_robot.launch.py navigation:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========== PACKAGE DIRECTORIES ==========
    rover_bringup_dir = get_package_share_directory("rover_bringup")
    rover_controller_dir = get_package_share_directory("rover_controller")
    rover_localization_dir = get_package_share_directory("rover_localization")
    rover_mapping_dir = get_package_share_directory("rover_mapping")
    rover_navigation_dir = get_package_share_directory("rover_navigation")
    
    # ========== LAUNCH ARGUMENTS ==========
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation time. Must be false for real hardware."
    )
    
    mapping_arg = DeclareLaunchArgument(
        name="mapping",
        default_value="false",
        description="Enable SLAM mapping (launches localization and SLAM)"
    )

    navigation_arg = DeclareLaunchArgument(
        name="navigation",
        default_value="false",
        description="Enable Nav2 for Autonomous Navigation"
    )
    
    # ========== LIDAR DRIVER ==========
    # RPLidar A1 driver node
    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[
            os.path.join(rover_bringup_dir, "config", "rplidar_a1.yaml")
        ],
        output="screen"
    )
    
    # ========== ROBOT DESCRIPTION ==========
    # Process xacro file to generate URDF
    robot_description = ParameterValue(
        Command([
            'xacro ', 
            LaunchConfiguration('model'), 
            ' is_ignition:=', is_ignition,
            ' is_sim:=', LaunchConfiguration('is_sim'),
            ' use_sim_time:=', LaunchConfiguration('use_sim_time')
        ]), 
        value_type=str
    )
    
    # Robot State Publisher - publishes /robot_description and TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration('use_sim_time')
        }],
        output="screen"
    )
    

    # ========== TELEOPERATION ==========
    # Joystick control
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_controller_dir, "launch", "joystick_teleop.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )
    
    # ========== LOCALIZATION ==========
    # Local localization - in general (navigation=false)
    local_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_localization_dir, "launch", "local_localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("navigation"))
    )
    
    # Global localization - when using pre-built map (navigation=true, mapping=false)
    global_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_localization_dir, "launch", "global_localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation"))
    )
    
    # ========== SLAM MAPPING ==========
    # SLAM - only when mapping=true
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_mapping_dir, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=IfCondition(LaunchConfiguration("mapping"))
    )
    
    # ========== NAVIGATION ==========
    # Nav2 - only when navigation=true
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_navigation_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation"))
    )
    
    # ========== TIMER-BASED SEQUENCING ==========
    # Proper startup sequence to ensure each component is ready before the next
    
    # # Wait for hardware interface to initialize ODrive connections
    # delayed_controller = TimerAction(
    #     period=10.0,  # Wait 5s for hardware interface and ODrive initialization
    #     actions=[controller]
    # )
    
    # Wait for controllers to be fully spawned and active
    delayed_teleop = TimerAction(
        period=15.0,  # Wait 10s for controllers to be ready
        actions=[joystick]
    )
    
    # Start lidar driver early (it can initialize in parallel)
    delayed_laser = TimerAction(
        period=7.0,  # Start lidar early
        actions=[laser_driver]
    )
    
    # Wait for controllers and sensors before starting localization
    delayed_local_localization = TimerAction(
        period=13.0,  # Start after controllers begin initializing
        actions=[local_localization]
    )
    
    delayed_global_localization = TimerAction(
        period=13.0,
        actions=[global_localization]
    )
    
    # Start SLAM after localization is running
    delayed_slam = TimerAction(
        period=17.0,  # Start after localization stabilizes
        actions=[slam]
    )
    
    # Start navigation after localization is running
    delayed_navigation = TimerAction(
        period=17.0,  # Start after localization stabilizes
        actions=[navigation]
    )
    
    # ========== LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        mapping_arg,
        navigation_arg,
        
        # Launch sequence
        # hardware_interface,              # t=0s:  Hardware interface starts (ODrive init)
        delayed_laser,                   # t=2s:  Lidar driver starts
        # delayed_controller,              # t=5s:  Controllers start spawning
        delayed_local_localization,      # t=8s:  Local localization (if mapping=true)
        delayed_global_localization,     # t=8s:  Global localization (if navigation=true)
        delayed_teleop,                  # t=10s: Teleoperation ready
        delayed_slam,                    # t=12s: SLAM starts (if mapping=true)
        delayed_navigation,              # t=12s: Nav2 starts (if navigation=true)
        robot_state_publisher_node,
    ])