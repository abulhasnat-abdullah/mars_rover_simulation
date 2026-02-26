import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rover_description_dir = get_package_share_directory("rover_description")
    
    # Change 1: Update default file extension from .xacro to .urdf
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(rover_description_dir, "urdf", "rover_description.urdf"),  # Changed from .urdf.xacro to .urdf
        description="Absolute path to robot urdf file"
    )
    
    # Change 2: Read URDF file directly without xacro processing
    # Remove Command and ParameterValue, read file content directly
    def get_urdf_content():
        urdf_file_path = os.path.join(rover_description_dir, "urdf", "rover_description.urdf")
        with open(urdf_file_path, 'r') as file:
            return file.read()
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": get_urdf_content()}]  # Changed: direct file content instead of xacro processing
    )
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(rover_description_dir, "rviz", "display.rviz")],
    )
    
    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])