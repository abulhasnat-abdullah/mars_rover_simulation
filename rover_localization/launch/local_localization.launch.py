from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time if true"
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("rover_localization"), "config", "ekf.yaml"),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_localization,
    ])