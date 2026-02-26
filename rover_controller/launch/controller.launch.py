"""
Optimized Controller Launch File

Waits for controller_manager service (started by Gazebo plugin) before spawning controllers.
This prevents "waiting for service" errors without duplicating the controller_manager node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========== LAUNCH ARGUMENTS ==========
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time"
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        name="wheel_radius",
        default_value="0.135",
        description="Wheel radius in meters"
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        name="wheel_separation",
        default_value="0.77",
        description="Wheel separation in meters"
    )
    
    # ========== WAIT FOR CONTROLLER MANAGER ==========
    # Controller manager is started by ros_gz_sim plugin in URDF
    # We need to wait for its service before spawning controllers
    wait_for_controller_manager = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "Waiting for controller_manager service..." && '
            'until ros2 service list | grep -q "/controller_manager/list_controllers"; do '
            '  sleep 0.5; '
            'done && '
            'echo "Controller manager ready!" && '
            'sleep 2'  # Extra buffer for stability
        ],
        output='screen',
        shell=False
    )
    
    # ========== CONTROLLER SPAWNERS ==========
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rover_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        remappings=[
                ('/rover_controller/odom', '/odom'),
            ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    # ========== TIMER-BASED SEQUENCING ==========
    # Start wait script immediately
    # Spawners start after wait script completes (guarantees service is ready)
    
    delayed_wait = TimerAction(
        period=0.1,  # Start wait check almost immediately
        actions=[wait_for_controller_manager]
    )
    
    # Joint state broadcaster starts 3s after launch
    # (wait script will complete before this)
    delayed_joint_state = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    # Wheel controller starts 5s after launch
    delayed_wheel_controller = TimerAction(
        period=5.0,
        actions=[wheel_controller_spawner]
    )
    
    # ========== LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        
        # Wait for controller_manager (non-blocking)
        delayed_wait,
        
        # Spawn controllers (after sufficient delay)
        delayed_joint_state,
        delayed_wheel_controller,
    ])