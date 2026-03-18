# Fully Autonomous Rover Simulation
A comprehensive ROS2 Jazzy workspace for simulating and controlling a 4-wheel differential drive rover with advanced navigation, mapping, and terrain analysis capabilities.

## Features

- **Gazebo Simulation** with multiple world environments (Mars terrain, Sonoma raceway)
- **SLAM Mapping** using SLAM Toolbox (2D LiDAR-based)
- **RTAB-Map** for RGB-D based 2D/3D terrain mapping
- **GPS Navigation** with Nav2 waypoint follower (A* and RPP planners)
- **AMCL Localization** for map-based navigation
- **Dual EKF Localization** with NavSat transform for GPS integration
- **ZED Camera Simulation** with depth sensing
- **Joystick Teleoperation** support
- **Twist Mux** for velocity command prioritization (autonomous vs manual control)

---

## System Requirements

- **OS:** Ubuntu 24.04 (Noble)
- **ROS2:** Jazzy Jalisco
- **Gazebo:** Harmonic (gz-sim 8.x)

---

## Dependencies Installation

### Prerequisites

```bash
# Install ROS2 Jazzy (if not already installed)
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Initialize rosdep (if not already done)
sudo rosdep init
rosdep update
```

### Python Packages (not covered by rosdep)

```bash
sudo apt install -y \
    python3-transforms3d \
    python3-numpy \
    python3-opencv \
    python3-pil \
    python3-yaml \
```

---

## Docker (ROS 2 Jazzy)

The workspace includes a full Docker setup at the repository root:

- `Dockerfile` - ROS 2 Jazzy image with rosdep + colcon tooling
- `docker-compose.yml` - dev runtime (host networking, RViz/Gazebo GUI support)
- `docker/entrypoint.sh` - auto-sources ROS and workspace overlays

### Build and Run

From `~/rover_ws`:

```bash
# Allow local X11 GUI apps from Docker (RViz/Gazebo)
xhost +local:docker

# Build image and start shell in container
docker compose run --rm rover_jazzy
```

Inside the container:

```bash
# Build if needed (workspace is mounted from host)
colcon build --symlink-install
source install/setup.bash
```

### Rebuild Image

```bash
docker compose build --no-cache
```

### Auto Push After Host Code Changes

When you change code on your host PC and want that update published to Docker Hub:

```bash
# one-time setup
cp .env.example .env

# edit .env and set your image name, for example:
# DOCKER_IMAGE=your_dockerhub_username/rover_ws:jazzy

# login once
docker login

# rebuild from current host code and push
./docker/push_to_dockerhub.sh
```

The script builds with `docker compose build`, pushes the configured tag, and optionally pushes `latest` if `PUSH_LATEST=1`.

### Notes

- The image runs `rosdep install --from-paths src --ignore-src` during build.
- Default skipped rosdep keys:
    - `ament_python` (non-system rosdep key in this workspace)
- To change skipped keys, edit `ROSDEP_SKIP_KEYS` in `docker-compose.yml`.
- Local `build/`, `install/`, and `log/` folders are excluded from image context via `.dockerignore`.

---

## Build Instructions

```bash
# Clone the workspace (if not already done)
cd ~/rover_ws

# Install rosdep dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc for persistence
echo "source ~/rover_ws/install/setup.bash" >> ~/.bashrc
```

---

## Launch Commands

### Basic Simulation

Launch the rover in Gazebo with joystick teleoperation:

```bash
# Mars Yard world (rocky terrain)
ros2 launch rover_bringup simulated_robot.launch.py world_name:=mars_yard world_extension:=.sdf

# Sonoma raceway world
ros2 launch rover_bringup simulated_robot.launch.py world_name:=sonoma world_extension:=.sdf

# Empty world (default)
ros2 launch rover_bringup simulated_robot.launch.py
```

### Available Worlds

| World Name | Extension | Description |
|------------|-----------|-------------|
| `mars_yard` | `.sdf` | Rocky Mars-like terrain for terrain mapping |
| `sonoma` | `.sdf` | Sonoma raceway - flat terrain for navigation |
| `empty` | `.world` | Empty world (default) |

---

### SLAM Mapping (LiDAR-based)

Create a 2D map using SLAM Toolbox:

```bash
ros2 launch rover_bringup simulated_robot.launch.py world_name:=sonoma world_extension:=.sdf mapping:=true
```

Drive around using joystick to build the map. Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/rover_ws/src/rover_mapping/maps/my_map
```

---

### AMCL Navigation (Map-based)

Navigate using a pre-built map with AMCL localization:

```bash
ros2 launch rover_bringup simulated_robot.launch.py world_name:=sonoma world_extension:=.sdf navigation:=true
```

Set initial pose and goal in RViz2 to navigate autonomously.

---

### GPS Navigation

Navigate using GPS waypoints with Nav2 (A* and RPP planners):

```bash
ros2 launch rover_bringup simulated_robot.launch.py world_name:=sonoma world_extension:=.sdf gps_nav:=true
```

This launches:
- Dual EKF (odom + map frames)
- NavSat transform for GPS → map conversion
- Nav2 with GPS waypoint follower

---

### RTAB-Map Terrain Mapping

RGB-D based 2D and 3D mapping:

```bash
ros2 launch rover_bringup simulated_robot.launch.py world_name:=mars_yard world_extension:=.sdf rtab:=true
```

This launches:
- RTAB-Map SLAM with RGB-D + point cloud
- Terrain processor for traversability analysis
- RViz2 with terrain visualization

**Published Topics:**
- `/rtabmap/map` - 2D occupancy grid
- `/rtabmap/cloud_map` - Accumulated 3D point cloud
- `/terrain/traversability` - Traversability costmap
- `/terrain/slope_map` - Slope analysis map

---

### Combined Mapping + Autonomous Navigation

For full autonomous exploration with terrain mapping:

**Terminal 1:** Launch simulation with GPS navigation
```bash
ros2 launch rover_bringup simulated_robot.launch.py world_name:=sonoma world_extension:=.sdf gps_nav:=true
```

**Terminal 2:** Launch RTAB-Map terrain mapping
```bash
ros2 launch rtabmap_terrain_mapping rtabmap_combined_mapping.launch.py
```

---

## Package Overview

| Package | Description |
|---------|-------------|
| `rover_description` | URDF/Xacro model, Gazebo launch, meshes |
| `rover_controller` | ros2_control config, diff drive, joystick teleop |
| `rover_bringup` | Main launch files combining all subsystems |
| `rover_localization` | EKF configs (local, global, GPS) |
| `rover_mapping` | SLAM Toolbox config and maps |
| `rover_navigation` | Nav2 config, behavior trees, costmaps |
| `rtabmap_terrain_mapping` | RTAB-Map 3D mapping with terrain analysis |
| `nav2_gps_waypoint_follower` | GPS waypoint navigation with dual EKF |
| `explore` | Frontier-based autonomous exploration |
| `pointcloud_to_laserscan` | Convert depth camera to 2D laser scan |
| `rover_utils` | Utility scripts and tools |

---

## Launch Arguments Reference

### `simulated_robot.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use Gazebo simulation time |
| `world_name` | `empty` | World file name (without extension) |
| `world_extension` | `.world` | World file extension (`.world` or `.sdf`) |
| `mapping` | `false` | Enable SLAM mapping mode |
| `navigation` | `false` | Enable AMCL navigation mode |
| `gps_nav` | `false` | Enable GPS navigation mode |
| `rtab` | `false` | Enable RTAB-Map terrain mapping |

### `rtabmap_combined_mapping.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation time |
| `rviz` | `true` | Launch RViz2 visualization |
| `rtabmap_viz` | `false` | Launch RTAB-Map viz (SLAM debugging) |
| `odom_topic` | `/odometry/local` | Odometry source topic |

---

## Troubleshooting

### Controller Manager Not Ready

If controllers fail to spawn, increase the delay or check:
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### TF Timeout Errors

Ensure all nodes use `use_sim_time:=true` in simulation:
```bash
ros2 param get /your_node use_sim_time
```

### GPS Navigation Issues

Verify NavSat transform is working:
```bash
ros2 topic echo /odometry/gps
ros2 topic echo /gps/fix
```

### RTAB-Map Not Accumulating

Check that point cloud topics are publishing:
```bash
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 topic hz /rtabmap/cloud_map
```

---

## Important Topics

### Sensor Data

| Topic | Type | Description |
|-------|------|-------------|
| `/zed/zed_node/rgb/color/rect/image` | Image | ZED RGB camera image |
| `/zed/zed_node/depth/depth_registered` | Image | ZED depth image |
| `/zed/zed_node/point_cloud/cloud_registered` | PointCloud2 | ZED point cloud |
| `/zed/zed_node/rgb/color/rect/image/camera_info` | CameraInfo | Camera intrinsics |
| `/scan` | LaserScan | 2D LiDAR scan |
| `/imu/data` | Imu | IMU sensor data |
| `/gps/fix` | NavSatFix | GPS coordinates |

### Odometry & Localization

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | Odometry | Wheel odometry |
| `/odometry/local` | Odometry | EKF fused odometry (odom frame) |
| `/odometry/global` | Odometry | EKF fused odometry (map frame) |
| `/odometry/gps` | Odometry | GPS-based odometry |

### Navigation

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands to robot |
| `/plan` | Path | Navigation planned path |
| `/local_costmap/costmap` | OccupancyGrid | Local costmap |
| `/global_costmap/costmap` | OccupancyGrid | Global costmap |
| `/goal_pose` | PoseStamped | Navigation goal |

### Mapping

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | OccupancyGrid | SLAM Toolbox 2D map |
| `/rtabmap/map` | OccupancyGrid | RTAB-Map 2D occupancy grid |
| `/rtabmap/cloud_map` | PointCloud2 | RTAB-Map accumulated 3D map |
| `/rtabmap/cloud_ground` | PointCloud2 | Ground points |
| `/rtabmap/cloud_obstacles` | PointCloud2 | Obstacle points |

### Terrain Analysis

| Topic | Type | Description |
|-------|------|-------------|
| `/terrain_map` | OccupancyGrid | Terrain occupancy map |
| `/terrain/traversability` | OccupancyGrid | Traversability costmap |
| `/terrain/slope_map` | OccupancyGrid | Slope analysis map |
| `/terrain/filtered_points` | PointCloud2 | Height-filtered point cloud |

---

## Author

[Abul Hasnat Abdullah](https://github.com/abulhasnat-abdullah)

---

## License

See individual package licenses.
