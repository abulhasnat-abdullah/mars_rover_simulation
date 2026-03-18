import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    pkg_share = get_package_share_directory('rover_nav2_slam_bringup')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (Gazebo) if true',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2',
    )

    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share, 'config', 'slam_toolbox_nav2.yaml'),
        description='Full path to slam_toolbox params yaml',
    )

    declare_nav2_params_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_slam_params.yaml'),
        description='Full path to Nav2 params yaml',
    )

    # 1) Local localization (odom->base_footprint) from rover_localization
    local_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_localization'),
                'launch',
                'local_localization.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 2) Live SLAM Toolbox (map->odom + /map)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_mapping'),
                'launch',
                'slam.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_config': slam_params_file,
            'rviz': 'false',
        }.items(),
    )

    # 3) Nav2 navigation stack (no AMCL/map_server; consumes /map from slam_toolbox)
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('rover_navigation'), 'rviz', 'nav2.rviz']
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_use_rviz_cmd,
            declare_slam_params_cmd,
            declare_nav2_params_cmd,
            local_localization,
            slam,
            nav2_navigation,
            rviz2_node,
        ]
    )
