# RTAB-Map Terrain Mapping

This project implements a terrain mapping solution using RTAB-Map for both 2D and 3D environments, specifically designed to handle uneven terrain. The project is built on ROS 2 and includes both Python and C++ components.

## Project Structure

```
rtabmap_terrain_mapping
├── src
│   ├── rtabmap_terrain_mapping
│   │   ├── __init__.py
│   │   ├── mapping_node.py         # Main ROS 2 node for mapping
│   │   ├── terrain_processor.py     # Logic for processing terrain data
│   │   └── point_cloud_handler.py   # Manages point cloud data
│   └── rtabmap_terrain_mapping_cpp
│       ├── src
│       │   ├── mapping_node.cpp     # C++ implementation of the mapping node
│       │   └── terrain_filter.cpp    # C++ terrain filtering algorithms
│       └── include
│           └── rtabmap_terrain_mapping_cpp
│               ├── mapping_node.hpp  # Header for C++ mapping node
│               └── terrain_filter.hpp # Header for terrain filter
├── launch
│   ├── rtabmap_2d_mapping.launch.py  # Launch file for 2D mapping
│   ├── rtabmap_3d_mapping.launch.py  # Launch file for 3D mapping
│   └── rtabmap_combined_mapping.launch.py # Launch file for combined mapping
├── config
│   ├── rtabmap_params.yaml           # RTAB-Map configuration parameters
│   ├── rgbd_camera.yaml              # RGB-D camera parameters
│   └── lidar_config.yaml             # LiDAR sensor parameters
├── rviz
│   └── rtabmap_visualization.rviz     # RViz configuration for visualization
├── msg
│   └── TerrainInfo.msg               # Custom message type for terrain information
├── srv
│   └── SaveMap.srv                   # Service for saving the generated map
├── package.xml                       # ROS 2 package configuration
├── CMakeLists.txt                   # Build instructions for C++ components
├── setup.py                          # Python package installation
└── README.md                         # Project documentation
```

## Installation

1. Ensure you have ROS 2 installed on your system.
2. Clone this repository into your ROS 2 workspace.
3. Install the required dependencies using:
   ```
   rosdep install -i --from-path src --rosdistro <your_ros_distro> -y
   ```
4. Build the project:
   ```
   colcon build
   ```

## Usage

To launch the mapping nodes, use the following commands:

- For 2D mapping:
  ```
  ros2 launch rtabmap_terrain_mapping rtabmap_2d_mapping.launch.py
  ```

- For 3D mapping:
  ```
  ros2 launch rtabmap_terrain_mapping rtabmap_3d_mapping.launch.py
  ```

- For combined mapping:
  ```
  ros2 launch rtabmap_terrain_mapping rtabmap_combined_mapping.launch.py
  ```

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.