import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_gps_waypoint_follower_dir = get_package_share_directory('nav2_gps_waypoint_follower')

    # Path to nav2 params file
    params_file = os.path.join(
        nav2_gps_waypoint_follower_dir,
        'config',
        'nav2_no_map_params.yaml'
    )

    # Path to rviz config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('nav2_gps_waypoint_follower'),
        'rviz',
        'gps_nav.rviz'
    ])

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Include nav2 navigation launch
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        nav2_navigation,
        rviz2_node,
    ])
