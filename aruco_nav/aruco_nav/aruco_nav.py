#!/usr/bin/env python3
"""
ArUco Tag-to-Tag Navigation Node (Search & Navigate)

The rover does NOT know where ArUco markers are. It must find them.

State Machine:
  SCAN_ROTATE        – spin in-place 360° scanning for the target marker
  EXPLORE_FRONTIER   – drive toward unexplored areas while scanning camera
  NAVIGATE_TO_MARKER – Nav2 NavigateToPose to a standoff point in front of
                       the detected marker
  APPROACH_MARKER    – slow final approach using direct cmd_vel toward marker
  ALL_DONE           – every marker in the sequence has been visited

Usage:
  ros2 run rover_controller aruco_navigator.py \
    --ros-args -p marker_sequence:=[31,30,29,28] \
               -p use_sim_time:=true
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

import cv2
import numpy as np
from cv_bridge import CvBridge

from tf_transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_multiply,
    quaternion_matrix,
)


# ════════════════════════════════════════════════════════════════════════
#  States
# ════════════════════════════════════════════════════════════════════════
class State:
    SCAN_ROTATE = "SCAN_ROTATE"
    EXPLORE_FRONTIER = "EXPLORE_FRONTIER"
    NAVIGATE_TO_MARKER = "NAVIGATE_TO_MARKER"
    APPROACH_MARKER = "APPROACH_MARKER"
    ALL_DONE = "ALL_DONE"


class ArucoNavigator(Node):
    """
    Full search-and-navigate node.
    Discovers ArUco markers by actively exploring, then drives to each one.
    """

    def __init__(self):
        super().__init__("aruco_navigator")

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter("marker_sequence", [31, 30, 29, 28, 27])
        # OpenCV aruco dictionary id (e.g. DICT_5X5_250 is commonly 6)
        self.declare_parameter("aruco_dict_id", 6)             # DICT_5X5_250
        self.declare_parameter("marker_size", 0.15)            # metres – matches your model box width
        self.declare_parameter("standoff_distance", 1.0)       # metres
        self.declare_parameter("approach_distance", 0.5)       # metres – final approach threshold

        # If the rover is already within this distance of the target tag,
        # consider it "visited" and advance to the next target.
        self.declare_parameter("visit_distance", 1.7)

        # Robustness / filtering
        # If true, ignore detections whose IDs are not in allowed_marker_ids.
        # If allowed_marker_ids is empty, marker_sequence is used.
        self.declare_parameter("ignore_unlisted_markers", True)
        # Important: declare as INTEGER_ARRAY explicitly (empty list defaults
        # can be inferred as BYTE_ARRAY in rclpy).
        self.declare_parameter(
            "allowed_marker_ids",
            [0],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER_ARRAY
            ),
        )
        # Reject map-pose updates that jump more than this (meters) vs last pose.
        self.declare_parameter("max_map_pose_jump", 1.0)
        # Simple exponential smoothing factor for tag position in map.
        # 0.0 = keep old, 1.0 = take new.
        self.declare_parameter("marker_pose_alpha", 0.35)

        # Target stability / goal management
        # Require N consistent detections before treating the target as "found"
        # during SCAN/EXPLORE. This prevents sending a bad Nav2 goal while the
        # robot is turning and the pose estimate is still settling.
        # Set to 1 to disable gating.
        self.declare_parameter("min_target_detections_for_nav", 1)
        # Detections are considered consistent if the map pose stays within
        # this radius (meters) between consecutive observations.
        self.declare_parameter("target_pose_stable_radius", 0.35)
        # While navigating to a marker, cancel+reissue the Nav2 goal if the
        # marker's filtered map pose moves by more than this distance.
        self.declare_parameter("nav_goal_retarget_distance", 0.75)
        # Minimum time (seconds) between retargets to avoid cancel spam.
        self.declare_parameter("nav_goal_retarget_min_interval", 1.0)
        # Require at least this many consistent detections before retargeting.
        self.declare_parameter("nav_goal_retarget_min_detections", 2)

        # Nav2 failure handling
        # After a Nav2 ABORTED result, wait this long before sending another
        # marker-nav goal (prevents rapid goal spam loops).
        self.declare_parameter("nav_abort_holdoff", 2.0)
        # If Nav2 aborts but the marker is still visible and within this
        # distance, fall back to APPROACH (direct cmd_vel) instead of Nav2.
        self.declare_parameter("abort_fallback_to_approach", True)
        self.declare_parameter("abort_fallback_max_distance", 3.5)

        # Mark tags in the map by broadcasting a map-fixed TF frame
        # named aruco_map_<id> at the discovered pose.
        self.declare_parameter("broadcast_map_tag_tf", True)
        self.declare_parameter("camera_topic",
                               "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("camera_frame",
                               "zed_left_camera_frame_optical")
        self.declare_parameter("cmd_vel_topic",
                               "rover_controller/cmd_vel")

        # Debug outputs
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "aruco_navigator/debug_image")

        # TF stamping
        # If true, stamp aruco_detected_<id> TF with the image header stamp.
        # If false (default), stamp with node clock time so TF is visible as
        # a "latest" transform in RViz/tf2_echo even when camera timestamps
        # don't align with TF buffer.
        self.declare_parameter("tf_use_image_stamp", False)

        # Use the image header stamp when looking up map->camera TF.
        # This makes the computed marker pose in map consistent and prevents
        # "random" jumps when the robot is moving.
        self.declare_parameter("map_tf_use_image_stamp", True)

        # Treat a detection as "fresh" only for this long (seconds).
        # Prevents driving on stale target pose after the marker is lost.
        self.declare_parameter("target_detection_timeout", 0.75)

        # Optional: OccupancyGrid topic for frontier-style exploration.
        self.declare_parameter("costmap_topic", "/map")

        # Scan rotation
        self.declare_parameter("scan_angular_vel", 0.4)        # rad/s while scanning
        self.declare_parameter("scan_full_rotations", 1.5)     # rotations before giving up

        # Explore
        self.declare_parameter("explore_linear_vel", 0.25)     # m/s while exploring
        self.declare_parameter("explore_angular_vel", 0.3)     # rad/s random turning
        self.declare_parameter("explore_drive_time", 5.0)      # seconds per leg
        self.declare_parameter("explore_turn_time", 3.0)       # seconds per turn
        self.declare_parameter("max_explore_cycles", 20)       # safety limit

        # Camera intrinsics (ZED 2i 720p defaults)
        self.declare_parameter("camera_fx", 528.0)
        self.declare_parameter("camera_fy", 528.0)
        self.declare_parameter("camera_cx", 640.0)
        self.declare_parameter("camera_cy", 360.0)
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])

        # Read parameters
        self.marker_sequence = list(
            self.get_parameter("marker_sequence")
            .get_parameter_value().integer_array_value
        )
        dict_id = (self.get_parameter("aruco_dict_id")
                   .get_parameter_value().integer_value)
        self.marker_size = (self.get_parameter("marker_size")
                            .get_parameter_value().double_value)
        self.standoff = (self.get_parameter("standoff_distance")
                         .get_parameter_value().double_value)
        self.approach_dist = (self.get_parameter("approach_distance")
                              .get_parameter_value().double_value)
        self.visit_distance = (self.get_parameter("visit_distance")
                               .get_parameter_value().double_value)
        self.broadcast_map_tag_tf = (
            self.get_parameter("broadcast_map_tag_tf")
            .get_parameter_value().bool_value
        )
        self.ignore_unlisted_markers = (
            self.get_parameter("ignore_unlisted_markers")
            .get_parameter_value().bool_value
        )
        self.allowed_marker_ids = list(
            self.get_parameter("allowed_marker_ids")
            .get_parameter_value().integer_array_value
        )
        self.max_map_pose_jump = (
            self.get_parameter("max_map_pose_jump")
            .get_parameter_value().double_value
        )
        self.marker_pose_alpha = (
            self.get_parameter("marker_pose_alpha")
            .get_parameter_value().double_value
        )

        self.min_target_detections_for_nav = (
            self.get_parameter("min_target_detections_for_nav")
            .get_parameter_value().integer_value
        )
        self.target_pose_stable_radius = (
            self.get_parameter("target_pose_stable_radius")
            .get_parameter_value().double_value
        )
        self.nav_goal_retarget_distance = (
            self.get_parameter("nav_goal_retarget_distance")
            .get_parameter_value().double_value
        )
        self.nav_goal_retarget_min_interval = (
            self.get_parameter("nav_goal_retarget_min_interval")
            .get_parameter_value().double_value
        )
        self.nav_goal_retarget_min_detections = (
            self.get_parameter("nav_goal_retarget_min_detections")
            .get_parameter_value().integer_value
        )

        self.nav_abort_holdoff = (
            self.get_parameter("nav_abort_holdoff")
            .get_parameter_value().double_value
        )
        self.abort_fallback_to_approach = (
            self.get_parameter("abort_fallback_to_approach")
            .get_parameter_value().bool_value
        )
        self.abort_fallback_max_distance = (
            self.get_parameter("abort_fallback_max_distance")
            .get_parameter_value().double_value
        )

        # Treat sentinel [0] as "unset" unless the user explicitly wants it.
        if self.allowed_marker_ids == [0]:
            self.allowed_marker_ids = []

        if not self.allowed_marker_ids:
            self.allowed_marker_ids = list(self.marker_sequence)
        camera_topic = (self.get_parameter("camera_topic")
                        .get_parameter_value().string_value)
        self.camera_frame = (self.get_parameter("camera_frame")
                             .get_parameter_value().string_value)
        cmd_vel_topic = (self.get_parameter("cmd_vel_topic")
                         .get_parameter_value().string_value)
        self.publish_debug_image = (
            self.get_parameter("publish_debug_image")
            .get_parameter_value().bool_value
        )
        self.debug_image_topic = (
            self.get_parameter("debug_image_topic")
            .get_parameter_value().string_value
        )
        self.tf_use_image_stamp = (
            self.get_parameter("tf_use_image_stamp")
            .get_parameter_value().bool_value
        )
        self.map_tf_use_image_stamp = (
            self.get_parameter("map_tf_use_image_stamp")
            .get_parameter_value().bool_value
        )
        self.target_detection_timeout = (
            self.get_parameter("target_detection_timeout")
            .get_parameter_value().double_value
        )
        costmap_topic = (
            self.get_parameter("costmap_topic")
            .get_parameter_value().string_value
        )
        self.scan_ang_vel = (self.get_parameter("scan_angular_vel")
                             .get_parameter_value().double_value)
        self.scan_rotations = (self.get_parameter("scan_full_rotations")
                               .get_parameter_value().double_value)
        self.explore_lin_vel = (self.get_parameter("explore_linear_vel")
                                .get_parameter_value().double_value)
        self.explore_ang_vel = (self.get_parameter("explore_angular_vel")
                                .get_parameter_value().double_value)
        self.explore_drive_t = (self.get_parameter("explore_drive_time")
                                .get_parameter_value().double_value)
        self.explore_turn_t = (self.get_parameter("explore_turn_time")
                               .get_parameter_value().double_value)
        self.max_explore = (self.get_parameter("max_explore_cycles")
                            .get_parameter_value().integer_value)

        fx = self.get_parameter("camera_fx").get_parameter_value().double_value
        fy = self.get_parameter("camera_fy").get_parameter_value().double_value
        cx = self.get_parameter("camera_cx").get_parameter_value().double_value
        cy = self.get_parameter("camera_cy").get_parameter_value().double_value
        dist = list(self.get_parameter("dist_coeffs")
                    .get_parameter_value().double_array_value)

        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self.dist_coeffs = np.array(dist, dtype=np.float64)

        # ── ArUco detector ──────────────────────────────────────────────
        # OpenCV ArUco API differs across versions/distros.
        # Prefer the newer ArucoDetector when available; otherwise fall back
        # to the functional detectMarkers() API.
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)

        # NOTE: OpenCV 4.6's aruco module can be fragile when passing
        # DetectorParameters into detectMarkers(). We'll still create it for
        # newer ArucoDetector API, but avoid using it with the functional API.
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        # ── CV Bridge ───────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── TF ──────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Publishers ──────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, cmd_vel_topic, 10)
        self.detected_markers_pub = self.create_publisher(
            Int32MultiArray, "aruco_navigator/detected_markers", 10)
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image, self.debug_image_topic, 10)

        # ── Nav2 Action Client ──────────────────────────────────────────
        self.cb_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self.cb_group)

        # ── State variables ─────────────────────────────────────────────
        self.state = State.SCAN_ROTATE
        self.current_goal_idx = 0
        self.target_marker_id = None
        self.navigating = False
        self.goal_handle = None

        self._is_frontier_nav = False
        self._last_marker_nav_status = None

        # Track last marker pose used to compute the current Nav2 standoff goal.
        self._active_marker_pose_for_nav = None  # PoseStamped (map)
        self._last_nav_goal_sent_time = None     # rclpy.time.Time
        self._current_result_future = None
        self._nav_holdoff_until = None           # rclpy.time.Time

        # Detection state
        self.detected_marker_pose_map = None   # PoseStamped in map frame
        self.detected_marker_distance = None   # distance in camera frame
        self._target_last_seen = None          # rclpy.time.Time
        self._target_latest_pose_map = None    # PoseStamped in map frame
        self._target_stable_last_pose_map = None
        self._target_detection_count = 0
        self.all_discovered = {}               # {marker_id: PoseStamped}
        self.visited_markers = set()           # {marker_id}
        self._filtered_marker_pose_map = {}    # {marker_id: PoseStamped}

        # Scan rotate state
        self.scan_start_time = None
        self.scan_accumulated_angle = 0.0

        # Explore state
        self.explore_sub_state = "TURN"        # "TURN" or "DRIVE"
        self.explore_sub_start = None
        self.explore_turn_direction = 1.0
        self.explore_cycles = 0

        # Costmap for frontier detection (optional enhancement)
        self.latest_costmap = None
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, costmap_topic, self.costmap_callback, 10)

        # ── Image Subscriber ────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        self._last_any_detection = self.get_clock().now()
        self._last_tf_warn = self.get_clock().now()

        # ── Main state-machine timer (10 Hz) ────────────────────────────
        self.dt = 0.1
        self.state_timer = self.create_timer(self.dt, self.state_machine_tick)

        self.get_logger().info(
            f"ArUco Navigator initialised (SEARCH MODE). "
            f"Sequence: {self.marker_sequence}")
        self.get_logger().info(
            "Waiting for Nav2 navigate_to_pose action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 action server available!")

        # Start first target
        self._set_next_target()

    # ════════════════════════════════════════════════════════════════════
    #  Image callback – ALWAYS running, detect all visible ArUco markers
    # ════════════════════════════════════════════════════════════════════
    def image_callback(self, msg: Image):
        """
        Process every camera frame. Detect ALL ArUco markers,
        store discoveries, and flag when target is found.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        if frame is None or (hasattr(frame, "size") and frame.size == 0):
            return

        # Make a safe grayscale uint8 contiguous image for OpenCV.
        if frame.ndim == 2:
            gray = frame
        elif frame.ndim == 3 and frame.shape[2] == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        elif frame.ndim == 3 and frame.shape[2] == 4:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        else:
            return

        gray = np.ascontiguousarray(gray)
        if gray.dtype != np.uint8:
            gray = gray.astype(np.uint8, copy=False)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            # Avoid passing DetectorParameters on older OpenCV builds; it has
            # been observed to segfault on some distro builds.
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)

        # Optional debug overlay
        if self.debug_image_pub is not None:
            debug_frame = frame.copy()
            if ids is not None and len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)
                self._last_any_detection = self.get_clock().now()
            else:
                # Occasionally log that nothing is being detected
                quiet_for = (self.get_clock().now() - self._last_any_detection).nanoseconds / 1e9
                if quiet_for > 5.0:
                    self.get_logger().warn(
                        "No ArUco markers detected for >5s. "
                        "Check camera_topic, focus/lighting, dictionary, and tag border.")
                    self._last_any_detection = self.get_clock().now()

            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
                dbg_msg.header = msg.header
                self.debug_image_pub.publish(dbg_msg)
            except Exception as e:
                self.get_logger().warn(f"Debug image publish failed: {e}")

        if ids is None:
            return

        ids_flat = ids.flatten()

        # Publish all detected marker IDs for debugging
        det_msg = Int32MultiArray()
        det_msg.data = [int(i) for i in ids_flat]
        self.detected_markers_pub.publish(det_msg)

        # Process each detected marker
        for i, marker_id in enumerate(ids_flat):
            marker_id = int(marker_id)

            if self.ignore_unlisted_markers and marker_id not in self.allowed_marker_ids:
                # Keep debug overlay + raw detected_markers topic, but ignore
                # unlisted IDs for navigation / TF-map marking.
                continue

            # Prefer the image header's frame_id when available to avoid
            # mismatches between configured and actual camera frames.
            camera_frame = self.camera_frame or msg.header.frame_id
            if not camera_frame:
                continue

            # Estimate pose in camera frame
            obj_points = np.array([
                [-self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0],
            ], dtype=np.float64)

            success, rvec, tvec = cv2.solvePnP(
                obj_points, corners[i], self.camera_matrix, self.dist_coeffs)
            if not success:
                continue

            distance = float(np.linalg.norm(tvec))

            # Build 4×4 transform
            rot_mat, _ = cv2.Rodrigues(rvec)
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = rot_mat
            T_cam_marker[:3, 3] = tvec.flatten()

            # Broadcast detected marker TF
            t = TransformStamped()
            if self.tf_use_image_stamp and (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0):
                t.header.stamp = msg.header.stamp
            else:
                t.header.stamp = self.get_clock().now().to_msg()

            if (self.map_tf_use_image_stamp and
                    (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0)):
                tf_lookup_time = msg.header.stamp
            else:
                tf_lookup_time = rclpy.time.Time()
            t.header.frame_id = camera_frame
            t.child_frame_id = f"aruco_detected_{marker_id}"
            tv = tvec.reshape(-1)
            t.transform.translation.x = float(tv[0])
            t.transform.translation.y = float(tv[1])
            t.transform.translation.z = float(tv[2])
            q = quaternion_from_matrix(T_cam_marker)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

            # If the frame isn't an optical frame, the OpenCV pose axes can be
            # inconsistent with TF conventions, causing "random" map poses.
            # Warn (throttled) so it is easy to spot.
            if "optical" not in camera_frame:
                since = (self.get_clock().now() - self._last_tf_warn).nanoseconds / 1e9
                if since > 10.0:
                    self.get_logger().warn(
                        f"Camera frame '{camera_frame}' does not look like an optical frame. "
                        "If tag poses look wrong, set camera_frame to the *_optical_frame TF.")
                    self._last_tf_warn = self.get_clock().now()

            # Transform to map frame
            try:
                tf_map = self.tf_buffer.lookup_transform(
                    "map", camera_frame, tf_lookup_time,
                    timeout=Duration(seconds=0.5))
            except Exception:
                # Common in sim: camera stamps and TF buffer aren't aligned.
                # Fall back to latest available transform.
                try:
                    tf_map = self.tf_buffer.lookup_transform(
                        "map", camera_frame, rclpy.time.Time(),
                        timeout=Duration(seconds=0.5))
                except Exception:
                    # Throttle warnings so the log isn't spammy.
                    since = (self.get_clock().now() - self._last_tf_warn).nanoseconds / 1e9
                    if since > 5.0:
                        self.get_logger().warn(
                            f"Detected marker {marker_id} in image, but cannot TF 'map' -> '{camera_frame}'. "
                            "Tag will not be considered 'found' until TF is available.")
                        self._last_tf_warn = self.get_clock().now()
                    continue

            pose_cam = PoseStamped()
            pose_cam.header.stamp = t.header.stamp
            pose_cam.header.frame_id = camera_frame
            pose_cam.pose.position.x = float(tv[0])
            pose_cam.pose.position.y = float(tv[1])
            pose_cam.pose.position.z = float(tv[2])
            pose_cam.pose.orientation.x = q[0]
            pose_cam.pose.orientation.y = q[1]
            pose_cam.pose.orientation.z = q[2]
            pose_cam.pose.orientation.w = q[3]

            pose_map = self._transform_pose(pose_cam, tf_map)

            # Filter / smooth pose in map to suppress outliers.
            prev = self._filtered_marker_pose_map.get(marker_id)
            if prev is not None:
                dx = pose_map.pose.position.x - prev.pose.position.x
                dy = pose_map.pose.position.y - prev.pose.position.y
                jump = math.hypot(dx, dy)
                if jump > float(self.max_map_pose_jump):
                    continue

                a = float(self.marker_pose_alpha)
                pose_map.pose.position.x = (a * pose_map.pose.position.x) + ((1.0 - a) * prev.pose.position.x)
                pose_map.pose.position.y = (a * pose_map.pose.position.y) + ((1.0 - a) * prev.pose.position.y)
                pose_map.pose.position.z = (a * pose_map.pose.position.z) + ((1.0 - a) * prev.pose.position.z)

            self._filtered_marker_pose_map[marker_id] = pose_map

            # Store discovery
            self.all_discovered[marker_id] = pose_map

            # Optionally broadcast a map-fixed marker frame so it is clearly
            # visible/"marked" in RViz.
            if self.broadcast_map_tag_tf:
                tag_tf = TransformStamped()
                tag_tf.header.stamp = self.get_clock().now().to_msg()
                tag_tf.header.frame_id = "map"
                tag_tf.child_frame_id = f"aruco_map_{marker_id}"
                tag_tf.transform.translation.x = pose_map.pose.position.x
                tag_tf.transform.translation.y = pose_map.pose.position.y
                tag_tf.transform.translation.z = pose_map.pose.position.z
                tag_tf.transform.rotation = pose_map.pose.orientation
                self.tf_broadcaster.sendTransform(tag_tf)

            # If this is our current target, flag it
            if marker_id == self.target_marker_id:
                self._handle_target_detection(marker_id, pose_map, distance)

    def _handle_target_detection(
        self, marker_id: int, pose_map: PoseStamped, distance: float
    ):
        """
        Update target-detection tracking with optional stability gating.
        A detection is considered navigation-ready once it has enough
        consistent observations.
        """
        now = self.get_clock().now()
        if self._target_last_seen is not None:
            gap = (now - self._target_last_seen).nanoseconds / 1e9
        else:
            gap = float("inf")

        if gap > float(self.target_detection_timeout):
            self._target_detection_count = 0
            self._target_stable_last_pose_map = None

        self._target_last_seen = now
        self.detected_marker_distance = distance
        self._target_latest_pose_map = pose_map

        consistent = True
        if self._target_stable_last_pose_map is not None:
            dx = (
                pose_map.pose.position.x
                - self._target_stable_last_pose_map.pose.position.x
            )
            dy = (
                pose_map.pose.position.y
                - self._target_stable_last_pose_map.pose.position.y
            )
            consistent = (
                math.hypot(dx, dy) <= float(self.target_pose_stable_radius)
            )

        if consistent:
            self._target_detection_count += 1
        else:
            self._target_detection_count = 1

        self._target_stable_last_pose_map = pose_map

        if self._target_detection_count < int(self.min_target_detections_for_nav):
            return

        self.detected_marker_pose_map = pose_map

        self.get_logger().info(
            f"🎯 FOUND target marker {marker_id} at distance "
            f"{distance:.2f}m → map({pose_map.pose.position.x:.2f}, "
            f"{pose_map.pose.position.y:.2f})")

        # If we're already close enough, count it as visited.
        if distance <= float(self.visit_distance):
            self._visit_current_marker(reason=f"within {distance:.2f}m")

    # ════════════════════════════════════════════════════════════════════
    #  Costmap callback (for frontier exploration)
    # ════════════════════════════════════════════════════════════════════
    def costmap_callback(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    # ════════════════════════════════════════════════════════════════════
    #  State Machine (10 Hz)
    # ════════════════════════════════════════════════════════════════════
    def state_machine_tick(self):
        if self.state == State.ALL_DONE:
            return

        if self.state == State.SCAN_ROTATE:
            self._tick_scan_rotate()
        elif self.state == State.EXPLORE_FRONTIER:
            self._tick_explore()
        elif self.state == State.NAVIGATE_TO_MARKER:
            self._tick_navigate()
        elif self.state == State.APPROACH_MARKER:
            self._tick_approach()

    # ────────────────────────────────────────────────────────────────────
    #  SCAN_ROTATE – spin in place, look for target marker
    # ────────────────────────────────────────────────────────────────────
    def _tick_scan_rotate(self):
        # Check if we found the marker while rotating
        if self.detected_marker_pose_map is not None:
            if self._nav_holdoff_until is not None and self.get_clock().now() < self._nav_holdoff_until:
                self.get_logger().info(
                    "Marker detected but Nav2 is in abort holdoff; continuing scan.",
                    throttle_duration_sec=2.0)
                self._publish_cmd_vel(0.0, self.scan_ang_vel)
                self.scan_accumulated_angle += self.scan_ang_vel * self.dt
                return
            self._stop_robot()
            self.get_logger().info(
                f"✅ Marker {self.target_marker_id} detected during scan!")
            # If already close, don't bother planning; just mark visited.
            if (self.detected_marker_distance is not None and
                    self.detected_marker_distance <= float(self.visit_distance)):
                self._visit_current_marker(reason="close during scan")
            else:
                self._transition_to_navigate()
            return

        # Initialise scan
        if self.scan_start_time is None:
            self.scan_start_time = self.get_clock().now()
            self.scan_accumulated_angle = 0.0
            self.get_logger().info(
                f"🔄 Scanning (rotating) to find marker "
                f"{self.target_marker_id}...")

        # Rotate
        self._publish_cmd_vel(0.0, self.scan_ang_vel)
        self.scan_accumulated_angle += self.scan_ang_vel * self.dt

        # Check if we've done enough rotations
        total_needed = self.scan_rotations * 2.0 * math.pi
        if self.scan_accumulated_angle >= total_needed:
            self._stop_robot()
            self.get_logger().info(
                f"🔍 Scan complete — marker {self.target_marker_id} "
                f"NOT found. Switching to EXPLORE.")
            self.scan_start_time = None
            self.scan_accumulated_angle = 0.0
            self._transition_to_explore()

    # ────────────────────────────────────────────────────────────────────
    #  EXPLORE_FRONTIER – drive around looking for the marker
    # ────────────────────────────────────────────────────────────────────
    def _tick_explore(self):
        # Always check if we found the marker while exploring
        if self.detected_marker_pose_map is not None:
            if self._nav_holdoff_until is not None and self.get_clock().now() < self._nav_holdoff_until:
                self.get_logger().info(
                    "Marker detected but Nav2 is in abort holdoff; continuing explore.",
                    throttle_duration_sec=2.0)
                return
            self._stop_robot()
            self.get_logger().info(
                f"✅ Marker {self.target_marker_id} found while exploring!")
            if (self.detected_marker_distance is not None and
                    self.detected_marker_distance <= float(self.visit_distance)):
                self._visit_current_marker(reason="close during explore")
            else:
                self._transition_to_navigate()
            return

        # Safety limit
        if self.explore_cycles >= self.max_explore:
            self.get_logger().warn(
                f"⚠️ Max explore cycles ({self.max_explore}) reached for "
                f"marker {self.target_marker_id}. Skipping.")
            self._stop_robot()
            self._advance_sequence()
            return

        # Try frontier-based goal first
        if self.explore_sub_state == "FRONTIER_NAV":
            # Nav2 is handling it; we just wait
            if not self.navigating:
                # Nav finished without finding marker → go back to turning
                self.explore_sub_state = "TURN"
                self.explore_sub_start = None
            return

        # Sub-state machine: TURN → DRIVE → TURN → ...
        if self.explore_sub_start is None:
            self.explore_sub_start = self.get_clock().now()
            if self.explore_sub_state == "TURN":
                # Try frontier goal from costmap
                frontier = self._find_frontier_goal()
                if frontier is not None:
                    self.get_logger().info(
                        f"🗺️ Frontier goal: ({frontier.pose.position.x:.1f},"
                        f" {frontier.pose.position.y:.1f})")
                    self.explore_sub_state = "FRONTIER_NAV"
                    self._send_nav_goal(frontier, is_frontier=True)
                    return
                else:
                    # Random turn direction
                    self.explore_turn_direction = random.choice([-1.0, 1.0])

        elapsed = ((self.get_clock().now() - self.explore_sub_start)
                   .nanoseconds / 1e9)

        if self.explore_sub_state == "TURN":
            self._publish_cmd_vel(
                0.0, self.explore_ang_vel * self.explore_turn_direction)
            if elapsed >= self.explore_turn_t:
                self.explore_sub_state = "DRIVE"
                self.explore_sub_start = None
                self.explore_cycles += 1
        elif self.explore_sub_state == "DRIVE":
            self._publish_cmd_vel(self.explore_lin_vel, 0.0)
            if elapsed >= self.explore_drive_t:
                self.explore_sub_state = "TURN"
                self.explore_sub_start = None

    # ────────────────────────────────────────────────────────────────────
    #  NAVIGATE_TO_MARKER – Nav2 drives to standoff point
    # ────────────────────────────────────────────────────────────────────
    def _tick_navigate(self):
        # While navigating to a marker, retarget the standoff goal if the
        # marker pose estimate improves/moves (common while rotating).
        if self.navigating and (not self._is_frontier_nav):
            if self._target_last_seen is not None:
                age = (self.get_clock().now() - self._target_last_seen).nanoseconds / 1e9
            else:
                age = float("inf")

            if (
                self._target_latest_pose_map is not None
                and age <= float(self.target_detection_timeout)
                and self._target_detection_count >= int(self.nav_goal_retarget_min_detections)
            ):
                if self._active_marker_pose_for_nav is None:
                    self._active_marker_pose_for_nav = self._target_latest_pose_map

                dx = (
                    self._target_latest_pose_map.pose.position.x
                    - self._active_marker_pose_for_nav.pose.position.x
                )
                dy = (
                    self._target_latest_pose_map.pose.position.y
                    - self._active_marker_pose_for_nav.pose.position.y
                )
                delta = math.hypot(dx, dy)

                can_retarget = True
                if self._last_nav_goal_sent_time is not None:
                    since = (self.get_clock().now() - self._last_nav_goal_sent_time).nanoseconds / 1e9
                    can_retarget = since >= float(self.nav_goal_retarget_min_interval)

                if delta >= float(self.nav_goal_retarget_distance) and can_retarget:
                    self.get_logger().warn(
                        f"Retargeting Nav2 goal: marker pose moved {delta:.2f}m")
                    try:
                        if self.goal_handle is not None:
                            self.goal_handle.cancel_goal_async()
                    except Exception:
                        pass

                    goal = self._compute_standoff_goal(self._target_latest_pose_map)
                    self._active_marker_pose_for_nav = self._target_latest_pose_map
                    self._last_nav_goal_sent_time = self.get_clock().now()
                    self._send_nav_goal(goal, is_frontier=False)

        if not self.navigating:
            # Only enter approach after we reached the standoff goal.
            if self._last_marker_nav_status == 4:  # SUCCEEDED
                self.state = State.APPROACH_MARKER
                self.get_logger().info(
                    f"📐 Entering APPROACH for marker {self.target_marker_id}")
            else:
                # If nav failed/canceled, fall back to re-search.
                self.get_logger().warn(
                    f"Nav ended with status {self._last_marker_nav_status}; "
                    f"returning to SCAN for marker {self.target_marker_id}.")
                self.state = State.SCAN_ROTATE
                self.scan_start_time = None
                self.scan_accumulated_angle = 0.0

    # ────────────────────────────────────────────────────────────────────
    #  APPROACH_MARKER – slow visual servoing toward the marker
    # ────────────────────────────────────────────────────────────────────
    def _tick_approach(self):
        # Require a recent detection for approach; otherwise treat as lost.
        if self._target_last_seen is not None:
            age = (self.get_clock().now() - self._target_last_seen).nanoseconds / 1e9
            if age > float(self.target_detection_timeout):
                self.detected_marker_pose_map = None

        if self.detected_marker_pose_map is None:
            # Lost sight – do a gentle scan
            self._publish_cmd_vel(0.0, 0.3)
            return

        # Get robot pose
        robot_tf = self._lookup_robot_tf(timeout_s=0.5)
        if robot_tf is None:
            return
        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y

        mx = self.detected_marker_pose_map.pose.position.x
        my = self.detected_marker_pose_map.pose.position.y
        dist = math.hypot(mx - rx, my - ry)

        if dist <= self.approach_dist:
            self._stop_robot()
            self.get_logger().info(
                f"✅ ARRIVED at marker {self.target_marker_id}! "
                f"(distance: {dist:.2f}m)")
            self._visit_current_marker(reason=f"arrived {dist:.2f}m")
            return

        # Proportional steering toward marker
        angle_to_marker = math.atan2(my - ry, mx - rx)

        # Get robot yaw
        q = robot_tf.transform.rotation
        _, _, robot_yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)
        angle_error = self._normalize_angle(angle_to_marker - robot_yaw)

        linear = min(0.15, 0.3 * dist)
        angular = 1.5 * angle_error

        # Clamp
        angular = max(-1.0, min(1.0, angular))

        self._publish_cmd_vel(linear, angular)

    # ════════════════════════════════════════════════════════════════════
    #  State transitions
    # ════════════════════════════════════════════════════════════════════
    def _set_next_target(self):
        if self.current_goal_idx >= len(self.marker_sequence):
            self.state = State.ALL_DONE
            self._stop_robot()
            self.get_logger().info(
                "🏁 ALL MARKERS VISITED! Sequence complete.")
            self.get_logger().info(
                f"   Discovered markers: "
                f"{list(self.all_discovered.keys())}")
            return

        self.target_marker_id = self.marker_sequence[self.current_goal_idx]
        self._reset_target_tracking()
        self.scan_start_time = None
        self.scan_accumulated_angle = 0.0
        self.explore_sub_state = "TURN"
        self.explore_sub_start = None
        self.explore_cycles = 0
        self.navigating = False
        self._active_marker_pose_for_nav = None
        self._last_nav_goal_sent_time = None

        # Check if we already discovered this marker while navigating
        if self.target_marker_id in self.all_discovered:
            self.get_logger().info(
                f"🎯 Marker {self.target_marker_id} was already discovered! "
                f"Navigating directly.")
            self.detected_marker_pose_map = (
                self.all_discovered[self.target_marker_id])
            self.state = State.NAVIGATE_TO_MARKER
            self._transition_to_navigate()
        else:
            self.state = State.SCAN_ROTATE
            self.get_logger().info(
                f"🎯 Target: marker {self.target_marker_id} "
                f"({self.current_goal_idx + 1}/"
                f"{len(self.marker_sequence)}) — SEARCHING...")

    def _reset_target_tracking(self):
        self.detected_marker_pose_map = None
        self.detected_marker_distance = None
        self._target_last_seen = None
        self._target_latest_pose_map = None
        self._target_stable_last_pose_map = None
        self._target_detection_count = 0

    def _advance_sequence(self):
        self.current_goal_idx += 1
        self._set_next_target()

    def _visit_current_marker(self, reason: str = ""):
        if self.target_marker_id is None:
            return

        # Cancel any active Nav2 goal so it doesn't keep commanding /cmd_vel
        # while we advance to the next marker.
        if self.goal_handle is not None and self.navigating:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.navigating = False
            self.goal_handle = None
            self._current_result_future = None

        self._stop_robot()
        self.visited_markers.add(int(self.target_marker_id))
        msg = f"✅ VISITED marker {self.target_marker_id}"
        if reason:
            msg += f" ({reason})"
        self.get_logger().info(msg)
        self._advance_sequence()

    def _transition_to_navigate(self):
        self.state = State.NAVIGATE_TO_MARKER
        goal = self._compute_standoff_goal(self.detected_marker_pose_map)
        self._active_marker_pose_for_nav = self.detected_marker_pose_map
        self._last_nav_goal_sent_time = self.get_clock().now()
        self._send_nav_goal(goal, is_frontier=False)

    def _transition_to_explore(self):
        self.state = State.EXPLORE_FRONTIER
        self.explore_sub_state = "TURN"
        self.explore_sub_start = None
        self.explore_cycles = 0

    # ════════════════════════════════════════════════════════════════════
    #  Frontier detection from costmap
    # ════════════════════════════════════════════════════════════════════
    def _find_frontier_goal(self) -> PoseStamped | None:
        """
        Simple frontier detection: find cells on the boundary between
        known-free and unknown space. Pick a random frontier cluster.
        """
        if self.latest_costmap is None:
            return None

        cmap = self.latest_costmap
        w = cmap.info.width
        h = cmap.info.height
        res = cmap.info.resolution
        ox = cmap.info.origin.position.x
        oy = cmap.info.origin.position.y
        data = np.array(cmap.data, dtype=np.int8).reshape((h, w))

        # Free = 0, Unknown = -1, Obstacle >= 50
        free_mask = (data == 0)
        unknown_mask = (data == -1)

        # Frontier: free cell adjacent (4-neighborhood) to unknown.
        # Numpy-only to avoid SciPy as a runtime dependency.
        unknown_up = np.zeros_like(unknown_mask)
        unknown_up[1:, :] = unknown_mask[:-1, :]
        unknown_down = np.zeros_like(unknown_mask)
        unknown_down[:-1, :] = unknown_mask[1:, :]
        unknown_left = np.zeros_like(unknown_mask)
        unknown_left[:, 1:] = unknown_mask[:, :-1]
        unknown_right = np.zeros_like(unknown_mask)
        unknown_right[:, :-1] = unknown_mask[:, 1:]

        unknown_neighbor = unknown_up | unknown_down | unknown_left | unknown_right
        frontier_mask = free_mask & unknown_neighbor

        frontier_coords = np.argwhere(frontier_mask)  # (row, col)
        if len(frontier_coords) == 0:
            return None

        # Get robot position
        rx, ry = self._get_robot_xy(default_xy=(0.0, 0.0), timeout_s=0.5)

        # Convert frontier cells to world coordinates
        world_coords = np.zeros((len(frontier_coords), 2))
        world_coords[:, 0] = frontier_coords[:, 1] * res + ox  # x
        world_coords[:, 1] = frontier_coords[:, 0] * res + oy  # y

        # Cluster frontiers (simple: pick cells at least 2m from robot, 
        # sort by distance, pick one between 3-8m away)
        dists = np.linalg.norm(
            world_coords - np.array([rx, ry]), axis=1)
        valid = (dists > 2.0) & (dists < 10.0)

        if not np.any(valid):
            valid = dists > 1.0
        if not np.any(valid):
            return None

        valid_coords = world_coords[valid]
        valid_dists = dists[valid]

        # Pick a random frontier from the closer half
        sorted_idx = np.argsort(valid_dists)
        half = max(1, len(sorted_idx) // 3)
        chosen_idx = random.choice(sorted_idx[:half])

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(valid_coords[chosen_idx, 0])
        goal.pose.position.y = float(valid_coords[chosen_idx, 1])
        goal.pose.position.z = 0.0

        # Face toward the goal
        yaw = math.atan2(
            goal.pose.position.y - ry, goal.pose.position.x - rx)
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        return goal

    # ════════════════════════════════════════════════════════════════════
    #  Navigation helpers
    # ════════════════════════════════════════════════════════════════════
    def _compute_standoff_goal(self, marker_pose: PoseStamped) -> PoseStamped:
        """Compute a goal 'standoff' metres from the marker, facing it."""
        mx = marker_pose.pose.position.x
        my = marker_pose.pose.position.y

        rx, ry = self._get_robot_xy(default_xy=(0.0, 0.0), timeout_s=0.5)

        dx = rx - mx
        dy = ry - my
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            dx, dy, dist = 1.0, 0.0, 1.0

        goal_x = mx + (dx / dist) * self.standoff
        goal_y = my + (dy / dist) * self.standoff
        yaw = math.atan2(my - goal_y, mx - goal_x)
        q = quaternion_from_euler(0, 0, yaw)

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        return goal

    def _send_nav_goal(self, goal_pose: PoseStamped, is_frontier=False):
        label = "frontier" if is_frontier else f"marker {self.target_marker_id}"
        self.get_logger().info(
            f"📍 Sending Nav2 goal for {label} → "
            f"({goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.navigating = True
        self._is_frontier_nav = is_frontier
        if not is_frontier:
            self._last_marker_nav_status = None
        else:
            self._active_marker_pose_for_nav = None
        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Nav2 goal REJECTED.")
            self.navigating = False
            return
        self.get_logger().info("Nav2 goal ACCEPTED.")
        result_future = self.goal_handle.get_result_async()
        self._current_result_future = result_future
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        # While navigating toward frontier, keep checking for target
        if (self._is_frontier_nav and
                self.detected_marker_pose_map is not None):
            self.get_logger().info(
                f"🎯 Target marker {self.target_marker_id} found "
                f"while exploring! Cancelling frontier nav.")
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            return
        if remaining > 0:
            self.get_logger().info(
                f"  Nav distance remaining: {remaining:.2f}m",
                throttle_duration_sec=5.0)

    def _nav_result_cb(self, future):
        # Ignore results from goals that have been superseded (e.g. retargeted).
        if self._current_result_future is not None and future is not self._current_result_future:
            return

        result = future.result()
        status = result.status

        if self._is_frontier_nav:
            self.navigating = False
            if self.detected_marker_pose_map is not None:
                self.get_logger().info("Frontier nav ended — marker found!")
                self._transition_to_navigate()
            else:
                self.get_logger().info(
                    "Frontier nav ended — marker not yet found. "
                    "Continuing exploration.")
                self.explore_sub_state = "TURN"
                self.explore_sub_start = None
            return

        # Marker navigation result
        self._last_marker_nav_status = status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"✅ Nav2 reached standoff for marker "
                f"{self.target_marker_id}!")
        elif status == 5:  # CANCELED
            if self.detected_marker_pose_map is not None:
                self.get_logger().info("Nav canceled — re-targeting marker.")
            else:
                self.get_logger().warn("Nav canceled.")
        elif status == 6:  # ABORTED
            self.get_logger().error("Nav ABORTED.")

            # Prevent rapid abort->scan->send_goal loops.
            self._nav_holdoff_until = self.get_clock().now() + Duration(seconds=float(self.nav_abort_holdoff))

            # If we can still see the marker and it's relatively close,
            # fall back to direct approach rather than Nav2.
            if (
                bool(self.abort_fallback_to_approach)
                and self.detected_marker_pose_map is not None
                and self.detected_marker_distance is not None
                and float(self.detected_marker_distance) <= float(self.abort_fallback_max_distance)
            ):
                self.get_logger().warn(
                    f"Falling back to APPROACH (marker ~{float(self.detected_marker_distance):.2f}m away)")
                self.state = State.APPROACH_MARKER
                self.navigating = False
                return
        else:
            self.get_logger().warn(f"Nav ended with status: {status}")

        self.navigating = False

    # ════════════════════════════════════════════════════════════════════
    #  Low-level helpers
    # ════════════════════════════════════════════════════════════════════
    def _publish_cmd_vel(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def _stop_robot(self):
        self._publish_cmd_vel(0.0, 0.0)

    def _lookup_robot_tf(self, timeout_s: float = 0.5):
        try:
            return self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(),
                timeout=Duration(seconds=timeout_s))
        except Exception:
            return None

    def _get_robot_xy(self, default_xy=None, timeout_s: float = 0.5):
        robot_tf = self._lookup_robot_tf(timeout_s=timeout_s)
        if robot_tf is None:
            if default_xy is not None:
                return default_xy
            return None, None
        return (
            robot_tf.transform.translation.x,
            robot_tf.transform.translation.y,
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        """Quaternion → (roll, pitch, yaw)."""
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    @staticmethod
    def _transform_pose(
        pose_in: PoseStamped, transform: TransformStamped
    ) -> PoseStamped:
        t = transform.transform.translation
        q_tf = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        p = pose_in.pose.position
        q_pose = [
            pose_in.pose.orientation.x,
            pose_in.pose.orientation.y,
            pose_in.pose.orientation.z,
            pose_in.pose.orientation.w,
        ]

        rot_matrix = quaternion_matrix(q_tf)[:3, :3]
        point = np.array([p.x, p.y, p.z])
        rotated = rot_matrix @ point

        out = PoseStamped()
        out.header.frame_id = transform.header.frame_id
        out.header.stamp = pose_in.header.stamp
        out.pose.position.x = rotated[0] + t.x
        out.pose.position.y = rotated[1] + t.y
        out.pose.position.z = rotated[2] + t.z

        q_out = quaternion_multiply(q_tf, q_pose)
        out.pose.orientation.x = q_out[0]
        out.pose.orientation.y = q_out[1]
        out.pose.orientation.z = q_out[2]
        out.pose.orientation.w = q_out[3]

        return out


def main():
    rclpy.init()
    node = ArucoNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()