import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="True",
        description="Use simulated time"
    )
    
    # Add new launch argument for gamepad
    gamepad_arg = DeclareLaunchArgument(
        name="gamepad",
        default_value="fn",
        description="Gamepad brand to use. Supported: 'eZ' for easysmx, 'fn' for fantech"
    )
    
    # Use PathJoinSubstitution to dynamically load the right config file based on gamepad arg
    joy_teleop_param_file = PathJoinSubstitution([
        get_package_share_directory("rover_controller"), 
        "config", 
        ["joy_teleop_", LaunchConfiguration("gamepad"), ".yaml"]
    ])
    
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            joy_teleop_param_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(get_package_share_directory("rover_controller"), "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )
    
    # CHANGED: Direct Node launch instead of IncludeLaunchDescription
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(get_package_share_directory("rover_controller"), "config", "twist_mux_topics.yaml"),
            os.path.join(get_package_share_directory("rover_controller"), "config", "twist_mux_locks.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        remappings=[
            ("cmd_vel_out", "rover_controller/cmd_vel")
        ]
    )
    
# Complete Data Flow:
# Input Sources:
# ├─ Joystick → /joy_vel ────┐
# ├─ Keyboard → /key_vel ────┤
# └─ Navigation → /cmd_vel ──┤
#                            │
#                            ├──→ twist_mux (prioritizes & outputs)
#                            │
#                            └──→ /rover_controller/cmd_vel (TwistStamped)
#                                         │
#                                         ├──→ rover_controller (subscribes)
#                                         │
#                                         └──→ Controls wheels (joint_FL, joint_FR, joint_BL, joint_BR)



    return LaunchDescription([
        use_sim_time_arg,
        gamepad_arg,
        joy_teleop,
        joy_node,
        twist_mux,
    ])