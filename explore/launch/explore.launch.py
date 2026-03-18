from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Locate packages
    explore_pkg = FindPackageShare('explore')
    mapping_pkg = FindPackageShare('rover_mapping')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # Configuration variables
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration("rviz")
    
    # Declare arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([explore_pkg, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Run RViz simultaneously.",
        choices=["True", "true", "False", "false"],
    )

    # robot_localization = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[os.path.join(get_package_share_directory("rover_localization"), "config", "ekf.yaml"),
    #                 {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #                 ],
    # )
    
    # SLAM launch (from husarion_ugv_mapping package)

    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([mapping_pkg, 'launch', 'slam.launch.py'])
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'rviz': 'false'  # Disable RViz from SLAM launch to avoid conflicts
    #     }.items()
    # )
    
    # Navigation launch - exclude map_server to avoid conflicts with SLAM
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            # 'use_lifecycle_mgr': 'false'  # Let SLAM handle lifecycle management
        }.items(),
    )
    
    # Just the exploration node instead of the full launch file
    explore_node = Node(
        package='explore',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([explore_pkg, 'config', 'explore.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Single RViz instance with explore config
    rviz_config_path = os.path.join(
        get_package_share_directory("explore"),
        "rviz",
        "explore.rviz"
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        declare_params_file_cmd,
        use_sim_time_arg,
        declare_use_rviz_arg,
        # robot_localization,
        # slam_launch,
        nav2_bringup_launch,
        explore_node,  # Use single node instead of full launch
        rviz_node
    ])