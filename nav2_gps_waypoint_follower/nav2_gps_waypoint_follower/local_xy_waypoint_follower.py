import math
import os
import sys
import time
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from nav2_gps_waypoint_follower.utils.gps_utils import quaternion_from_euler


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"YAML at {path} did not parse to a dict")
    return data


def _get_datum_from_dual_ekf_params(dual_ekf_navsat_params_path: str) -> tuple[float, float, float]:
    params = _load_yaml(dual_ekf_navsat_params_path)
    navsat = params.get("navsat_transform", {}).get("ros__parameters", {})

    # Handle both 'datum' and accidental 'datum ' keys.
    datum_key = None
    for key in navsat.keys():
        if isinstance(key, str) and key.strip() == "datum":
            datum_key = key
            break

    if datum_key is None:
        raise KeyError(
            f"No 'datum' found in navsat_transform.ros__parameters in {dual_ekf_navsat_params_path}"
        )

    datum = navsat[datum_key]
    if (
        not isinstance(datum, list)
        or len(datum) != 3
        or not all(isinstance(x, (int, float)) for x in datum)
    ):
        raise ValueError(
            f"Expected datum: [lat, lon, alt], got {datum!r} in {dual_ekf_navsat_params_path}"
        )

    return float(datum[0]), float(datum[1]), float(datum[2])


def _latlon_to_local_enu_m(
    latitude_deg: float,
    longitude_deg: float,
    datum_lat_deg: float,
    datum_lon_deg: float,
) -> tuple[float, float]:
    """Convert WGS84 degrees to local ENU meters using a small-area approximation.

    Returns (east_m, north_m).

    This matches robot_localization/navsat_transform's typical ENU convention:
    - X: east
    - Y: north
    """

    earth_radius_m = 6378137.0  # WGS84 semi-major axis
    deg_to_rad = math.pi / 180.0

    dlat = (latitude_deg - datum_lat_deg) * deg_to_rad
    dlon = (longitude_deg - datum_lon_deg) * deg_to_rad

    lat0 = datum_lat_deg * deg_to_rad

    north_m = dlat * earth_radius_m
    east_m = dlon * earth_radius_m * math.cos(lat0)
    return east_m, north_m


def _parse_waypoints(wps_file_path: str) -> list[dict[str, float]]:
    wps = _load_yaml(wps_file_path)
    raw = wps.get("waypoints")
    if not isinstance(raw, list):
        raise ValueError(f"Expected 'waypoints:' list in {wps_file_path}")

    parsed: list[dict[str, float]] = []
    for i, wp in enumerate(raw):
        if not isinstance(wp, dict):
            raise ValueError(f"Waypoint {i} in {wps_file_path} is not a dict")
        try:
            latitude = float(wp["latitude"])
            longitude = float(wp["longitude"])
            yaw = float(wp.get("yaw", 0.0))
        except Exception as exc:
            raise ValueError(f"Invalid waypoint {i} in {wps_file_path}: {wp!r}") from exc
        parsed.append({"latitude": latitude, "longitude": longitude, "yaw": yaw})

    return parsed


def main() -> None:
    rclpy.init()

    package_share = get_package_share_directory("nav2_gps_waypoint_follower")
    default_wps = os.path.join(package_share, "config", "erc_traverse_wps.yaml")
    default_dual_ekf = os.path.join(package_share, "config", "dual_ekf_navsat_params.yaml")

    wps_path = sys.argv[1] if len(sys.argv) > 1 else default_wps
    dual_ekf_params_path = sys.argv[2] if len(sys.argv) > 2 else default_dual_ekf

    datum_lat, datum_lon, _datum_alt = _get_datum_from_dual_ekf_params(dual_ekf_params_path)
    wps = _parse_waypoints(wps_path)

    navigator = BasicNavigator("basic_navigator")
    navigator.waitUntilNav2Active(localizer="robot_localization")

    now = navigator.get_clock().now().to_msg()
    poses: list[PoseStamped] = []

    for wp in wps:
        east_m, north_m = _latlon_to_local_enu_m(wp["latitude"], wp["longitude"], datum_lat, datum_lon)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = now
        pose.pose.position.x = east_m
        pose.pose.position.y = north_m
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_euler(0.0, 0.0, wp["yaw"])
        poses.append(pose)

    navigator.followWaypoints(poses)
    while not navigator.isTaskComplete():
        time.sleep(0.1)


if __name__ == "__main__":
    main()
