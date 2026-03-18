from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        # Ensure ROS nodes don't accidentally use user-site Python packages
        # (e.g. a pip-installed NumPy 2.x) that can break ROS Jazzy binary
        # wheels like cv_bridge.
        SetEnvironmentVariable(name="PYTHONNOUSERSITE", value="1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="aruco_nav",
            executable="aruco_nav.py",
            name="aruco_nav",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                # ── Marker sequence to visit ──
                {"marker_sequence": [51, 52, 53, 54, 55, 56, 57]},
                {"ignore_unlisted_markers": True},
                {"allowed_marker_ids": [51, 52, 53, 54, 55, 56, 57]},
                {"max_map_pose_jump": 1.0},
                {"marker_pose_alpha": 0.35},
                # ── Stability / retargeting ──
                {"min_target_detections_for_nav": 1},
                {"target_pose_stable_radius": 0.35},
                {"nav_goal_retarget_distance": 0.30},
                {"nav_goal_retarget_min_interval": 1.0},
                {"nav_goal_retarget_min_detections": 1},
                # ── ArUco config ──
                {"aruco_dict_id": 6},              # DICT_5X5_250
                {"marker_size": 0.15},             # match your Gazebo model
                {"standoff_distance": 1.0},
                {"approach_distance": 0.5},
                {"visit_distance": 1.5},
                # ── Camera ──
                {"camera_topic": "/zed/zed_node/rgb/color/rect/image"},
                # Use optical frame (OpenCV solvePnP assumes optical coords)
                {"camera_frame": "zed_left_camera_frame_optical"},
                # Dedicated twist_mux input for ArUco behaviors
                {"cmd_vel_topic": "/aruco_cmd_vel"},
                # Compute map pose using TF at the image timestamp
                {"map_tf_use_image_stamp": True},
                # Keep aruco_detected_* TF visible as "latest"
                {"tf_use_image_stamp": False},
                # ── Nav2 failure handling ──
                {"nav_abort_holdoff": 2.0},
                {"abort_fallback_to_approach": True},
                {"abort_fallback_max_distance": 3.5},
                # ── Scan behaviour ──
                {"scan_angular_vel": 0.25},
                {"scan_full_rotations": 1.5},
                # ── Explore behaviour ──
                {"explore_linear_vel": 0.25},
                {"explore_angular_vel": 0.3},
                {"explore_drive_time": 5.0},
                {"explore_turn_time": 3.0},
                {"max_explore_cycles": 20},
                # ── Camera intrinsics ──
                {"camera_fx": 528.0},
                {"camera_fy": 528.0},
                {"camera_cx": 640.0},
                {"camera_cy": 360.0},
                {"dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]},
            ],
        ),
    ])