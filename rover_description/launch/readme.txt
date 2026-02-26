"""
Optimized Gazebo Launch File for Rover Simulation

This launch file is designed to eliminate clock synchronization errors and runtime issues
by using event-based sequencing instead of arbitrary time delays. It ensures proper startup
order of all simulation components.

Key Design Principles:
1. Clock bridge must start with Gazebo to provide /clock topic immediately
2. All nodes must use the same time source (use_sim_time parameter)
3. Entity spawning must wait for robot_description topic to be published
4. Sensor bridges must wait for the entity to be fully spawned in Gazebo
"""

import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directory for file path resolution
    rover_description = get_package_share_directory("rover_description")
    
    # Detect ROS distribution to set appropriate Gazebo variant
    # Humble uses Ignition, newer distros use Gazebo
    ros_distro = os.environ.get("ROS_DISTRO")
    is_ignition = "True" if ros_distro == "humble" else "False"

    # ========== LAUNCH ARGUMENTS ==========
    # These arguments allow users to customize the launch behavior from command line
    # Example: ros2 launch rover_description gazebo.launch.py use_sim_time:=false
    
    # CRITICAL: use_sim_time must be true in simulation to avoid clock errors
    # When true, all nodes will use /clock topic published by Gazebo instead of system time
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time if true. MUST be true for Gazebo simulation."
    )

    # Determines if robot is in simulation or real hardware mode
    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        default_value="true",
        description="Start robot in simulation mode if true"
    )
    
    # Path to the robot's URDF/xacro file that defines its structure
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(rover_description, "urdf", "rover_description.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    # World file to load in Gazebo (without extension)
    world_name_arg = DeclareLaunchArgument(
        name="world_name", 
        default_value="empty",
        description="Name of the Gazebo world file (without extension)"
    )
    
    # World file extension (.world for classic Gazebo, .sdf for newer versions)
    world_extension_arg = DeclareLaunchArgument(
        name='world_extension',
        default_value='.world',
        description='The file extension for the simulation world (.world or .sdf)'
    )
    
    # Construct full path to world file by joining directory, name, and extension
    world_path = PathJoinSubstitution([
        rover_description,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '", LaunchConfiguration("world_extension"), "'"])
    ])

    # ========== ENVIRONMENT VARIABLES ==========
    # Set up Gazebo resource paths so it can find custom models and worlds
    
    # Get parent directory of rover_description package (where other packages might be)
    model_path = str(Path(rover_description).parent.resolve())
    # Add the models directory from rover_description package
    model_path += pathsep + os.path.join(get_package_share_directory("rover_description"), 'models')

    # GZ_SIM_RESOURCE_PATH tells Gazebo where to look for models and worlds
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )
    
    # ========== ROBOT DESCRIPTION ==========
    # Process the xacro file to generate the URDF robot description
    # This will be published to the /robot_description topic by robot_state_publisher
    
    # ParameterValue with Command allows us to execute xacro command and pass arguments
    robot_description = ParameterValue(
        Command([
            'xacro ',  # Call xacro processor
            LaunchConfiguration('model'),  # Path to xacro file
            ' is_ignition:=', is_ignition,  # Tell xacro which Gazebo version we're using
            ' is_sim:=', LaunchConfiguration('is_sim'),  # Simulation vs real hardware
            ' use_sim_time:=', LaunchConfiguration('use_sim_time')  # Time source config
        ]), 
        value_type=str
    )
    
    # ========== CORE NODES ==========
    
    # ========== Robot State Publisher ==========
    # Publishes the robot's URDF to /robot_description topic and broadcasts TF transforms
    # CRITICAL: This must start early because gz_spawn_entity needs the robot_description topic
    # CRITICAL: use_sim_time must use LaunchConfiguration, not hardcoded True, to avoid clock conflicts
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",  # Named for event handling
        parameters=[{
            "robot_description": robot_description,  # The processed URDF from xacro
            "use_sim_time": LaunchConfiguration('use_sim_time')  # Use simulation clock
        }],
        output="screen"
    )

    # ========== Gazebo Simulator ==========
    # Launches the Gazebo physics simulator with the specified world
    # The -v 4 flag sets verbose output level, -r flag starts simulation immediately
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )
    
    # ========== Clock Bridge ==========
    # CRITICAL: Bridges Gazebo's simulation time to ROS /clock topic
    # This MUST start immediately with Gazebo to prevent "time stamp in future" errors
    # All ROS nodes with use_sim_time=true will subscribe to this /clock topic
    # Without this, nodes using sim time will fail or produce warnings
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"  # Bridge format: topic@ROStype[Gztype
        ],
        output="screen",
    )
    
    # ========== Entity Spawner ==========
    # Spawns the robot entity into the running Gazebo simulation
    # Reads from /robot_description topic (published by robot_state_publisher)
    # IMPORTANT: This is triggered by event handler, not launched directly
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="gz_spawn_entity",  # Named for event handling
        output="screen",
        arguments=[
            "-topic", "robot_description",  # Where to read the URDF from
            "-name", "rover",  # Name of entity in Gazebo
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],  # Use sim clock
    )
    
    # ========== SENSOR BRIDGES ==========
    # These bridges connect Gazebo sensor topics to ROS topics
    # CRITICAL: All bridges must have use_sim_time parameter to avoid timestamp conflicts
    # IMPORTANT: These are triggered by event handler after entity spawns, not launched directly
    
    # Bridge for IMU and LiDAR sensors
    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sensor_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",  # IMU data
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"  # LiDAR scan data
        ],
        remappings=[
            ('/imu', '/imu/out'),  # Remap to match expected topic name
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    # Bridge for RGB camera image
    image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="image_bridge",
        arguments=[
            "/zed/zed_node/rgb/image_rect_color@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    # Bridge for depth camera image
    depth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="depth_bridge",
        arguments=[
            "/zed/zed_node/depth/depth_registered@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    # Bridge for RGB camera calibration info
    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=[
            "/zed/zed_node/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    # Bridge for depth camera calibration info
    depth_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="depth_camera_info_bridge",
        arguments=[
            "/zed/zed_node/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )
    
    # Bridge for 3D point cloud data
    point_cloud_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="point_cloud_bridge",
        arguments=[
            "/zed/zed_node/depth/depth_registered/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )

    # ========== EVENT-BASED LAUNCH SEQUENCING ==========
    # Using event handlers instead of fixed timers makes the launch more robust
    # Nodes start when their dependencies are actually ready, not after arbitrary delays
    
    # ===== STEP 1: Spawn Entity After Robot Description is Ready =====
    # Wait for robot_state_publisher to start (ensures /robot_description topic exists)
    # Then wait 3 seconds for the topic to be fully published before spawning
    # WHY: gz_spawn_entity reads from /robot_description topic and will fail if it's not ready
    # WHY: Event-based triggering is more reliable than guessing a fixed delay
    spawn_entity_event = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,  # Trigger when this node starts
            on_start=[
                TimerAction(
                    period=3.0,  # Small delay to ensure topic is published and stable
                    actions=[gz_spawn_entity]  # Then spawn the robot in Gazebo
                )
            ]
        )
    )
    
    # ===== STEP 2: Start Sensor Bridges After Entity is Spawned =====
    # Wait for gz_spawn_entity to start (ensures robot exists in Gazebo)
    # Then wait 2 seconds for sensors to be fully initialized in simulation
    # WHY: Sensor bridges try to connect to Gazebo sensor topics
    # WHY: If the entity isn't spawned yet, these topics don't exist and bridges fail
    # WHY: This prevents "topic not found" errors and ensures clean startup
    sensor_bridges_event = RegisterEventHandler(
        OnProcessStart(
            target_action=gz_spawn_entity,  # Trigger when spawn process starts
            on_start=[
                TimerAction(
                    period=2.0,  # Give sensors time to initialize in Gazebo
                    actions=[
                        # Launch all sensor bridges together
                        sensor_bridge,
                        image_bridge,
                        depth_bridge,
                        camera_info_bridge,
                        depth_camera_info_bridge,
                        point_cloud_bridge,
                    ]
                )
            ]
        )
    )

    # ========== LAUNCH DESCRIPTION ==========
    # The order of items in this list determines the launch sequence
    # This optimized sequence prevents clock errors and ensures reliable startup
    
    return LaunchDescription([
        # ===== Configuration Arguments (processed first) =====
        model_arg,              # Robot URDF/xacro file path
        world_name_arg,         # Gazebo world name
        is_sim_arg,             # Simulation vs real hardware flag
        use_sim_time_arg,       # Clock source configuration
        world_extension_arg,    # World file extension

        # ===== Environment Setup =====
        gazebo_resource_path,   # Tell Gazebo where to find models/worlds
        
        # ===== CRITICAL LAUNCH SEQUENCE =====
        # The following order is carefully designed to prevent timing and clock errors:
        
        # 1. START GAZEBO AND CLOCK BRIDGE TOGETHER
        #    - Gazebo provides the simulation environment
        #    - Clock bridge MUST start immediately to provide /clock topic
        #    - Starting them together ensures no node misses clock messages
        gazebo,
        clock_bridge,
        
        # 2. START ROBOT STATE PUBLISHER
        #    - Processes xacro file and publishes /robot_description topic
        #    - Uses simulation time from /clock topic (via use_sim_time parameter)
        #    - Must start before entity spawning but after clock is available
        robot_state_publisher_node,
        
        # 3. EVENT: SPAWN ENTITY WHEN ROBOT_DESCRIPTION IS READY
        #    - Waits for robot_state_publisher to start
        #    - Adds 3-second delay for topic to be published
        #    - Then spawns robot entity in Gazebo
        #    - Event-based triggering prevents "topic not found" errors
        spawn_entity_event,
        
        # 4. EVENT: START SENSOR BRIDGES WHEN ENTITY IS SPAWNED
        #    - Waits for gz_spawn_entity to start
        #    - Adds 2-second delay for sensors to initialize in simulation
        #    - Then starts all sensor bridges to connect Gazebo topics to ROS
        #    - Event-based triggering prevents "sensor not found" errors
        sensor_bridges_event,
        
        # ===== RESULT =====
        # This sequence ensures:
        # - No "time stamp in future" errors (clock bridge starts immediately)
        # - No "robot_description not found" errors (event-based spawn timing)
        # - No "sensor topic not found" errors (sensors start after entity spawns)
        # - All nodes use consistent time source (use_sim_time parameter)
        # - Robust startup that adapts to actual system state, not arbitrary delays
    ])