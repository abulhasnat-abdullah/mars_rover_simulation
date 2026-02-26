#!/usr/bin/env python3
"""
Terrain Processor - Point Cloud Terrain Analysis
Processes point cloud data for terrain traversability analysis.
Features:
- Height-based filtering
- Slope/roughness calculation  
- Traversability costmap generation
- Obstacle detection
"""

import rclpy
import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import struct
import math
import numpy as np
from collections import defaultdict


class TerrainProcessor(Node):
    def __init__(self):
        super().__init__('terrain_processor')
        
        # Parameters
        self.declare_parameter('min_height', -0.3)  # Min height relative to robot
        self.declare_parameter('max_height', 0.5)   # Max traversable obstacle height
        self.declare_parameter('resolution', 0.1)   # Grid cell size in meters
        self.declare_parameter('map_size', 20.0)    # Map size in meters (square)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('slope_threshold', 30.0)  # Max slope in degrees
        self.declare_parameter('roughness_threshold', 0.15)  # Max roughness (std dev)
        self.declare_parameter('min_points_per_cell', 3)  # Min points for valid cell
        
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.resolution = self.get_parameter('resolution').value
        self.map_size = self.get_parameter('map_size').value
        self.slope_threshold = self.get_parameter('slope_threshold').value
        self.roughness_threshold = self.get_parameter('roughness_threshold').value
        self.min_points = self.get_parameter('min_points_per_cell').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Grid dimensions
        self.grid_size = int(self.map_size / self.resolution)
        self.origin_offset = self.map_size / 2.0  # Center the map
        
        
        # Subscribers - ZED camera point cloud
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/depth/depth_registered/points',
            self.pointcloud_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        
        # Subscribe to RTAB-Map cloud map for accumulated terrain
        self.rtabmap_cloud_sub = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.rtabmap_cloud_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        
        # Subscribe to RTAB-Map grid as base layer
        self.rtabmap_grid_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.rtabmap_grid_callback,
        )
        
        # Publishers
        self.terrain_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/terrain_map',
        )
        
        self.traversability_pub = self.create_publisher(
            OccupancyGrid,
            '/terrain/traversability',
        )
        
        self.slope_map_pub = self.create_publisher(
            OccupancyGrid,
            '/terrain/slope_map',
        )
        
        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2,
            '/terrain/filtered_points',
            10
        )
        
        # Terrain data storage
        self.terrain_cells = defaultdict(list)  # (x, y) -> list of z values
        self.last_rtabmap_grid = None
        
        # Timer for periodic terrain map updates
        self.create_timer(1.0, self.publish_terrain_maps)
        
        self.get_logger().info('Terrain Processor initialized with analysis')
        self.get_logger().info(f'Grid: {self.grid_size}x{self.grid_size}, resolution: {self.resolution}m')
        self.get_logger().info(f'Height range: [{self.min_height}, {self.max_height}]m')
        self.get_logger().info(f'Slope threshold: {self.slope_threshold}°, Roughness: {self.roughness_threshold}m')
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        gx = int((x + self.origin_offset) / self.resolution)
        gy = int((y + self.origin_offset) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates."""
        x = gx * self.resolution - self.origin_offset
        y = gy * self.resolution - self.origin_offset
        return x, y
    
    def parse_pointcloud2(self, msg: PointCloud2):
        """Parse PointCloud2 message to extract xyz points."""
        points = []
        
        # Find x, y, z field offsets
        field_offsets = {}
        for field in msg.fields:
            field_offsets[field.name] = (field.offset, field.datatype)
        
        if 'x' not in field_offsets or 'y' not in field_offsets or 'z' not in field_offsets:
            return points
        
        x_off, x_type = field_offsets['x']
        y_off, y_type = field_offsets['y']
        z_off, z_type = field_offsets['z']
        
        # Parse points
        point_step = msg.point_step
        data = msg.data
        
        for i in range(0, len(data), point_step):
            try:
                x = struct.unpack_from('f', data, i + x_off)[0]
                y = struct.unpack_from('f', data, i + y_off)[0]
                z = struct.unpack_from('f', data, i + z_off)[0]
                
                # Filter NaN and infinite values
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    points.append((x, y, z))
            except struct.error:
                continue
        
        return points
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming ZED point cloud for terrain analysis."""
        points = self.parse_pointcloud2(msg)
        
        if not points:
            return
        
        # Filter points by height and add to terrain cells
        filtered_points = []
        for x, y, z in points:
            if self.min_height <= z <= self.max_height:
                gx, gy = self.world_to_grid(x, y)
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    self.terrain_cells[(gx, gy)].append(z)
                    filtered_points.append((x, y, z))
        
        # Publish filtered point cloud
        if filtered_points:
            self.publish_filtered_cloud(filtered_points, msg.header)
        
        self.get_logger().debug(f'Processed {len(filtered_points)} terrain points')
    
    def rtabmap_cloud_callback(self, msg: PointCloud2):
        """Process RTAB-Map accumulated cloud map for terrain."""
        points = self.parse_pointcloud2(msg)
        
        if not points:
            return
        
        # Add all valid points to terrain cells
        for x, y, z in points:
            if self.min_height <= z <= self.max_height:
                gx, gy = self.world_to_grid(x, y)
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    self.terrain_cells[(gx, gy)].append(z)
        
        self.get_logger().debug(f'Integrated {len(points)} RTAB-Map points')
    
    def rtabmap_grid_callback(self, msg: OccupancyGrid):
        """Store RTAB-Map grid as base layer."""
        self.last_rtabmap_grid = msg
        # Also forward as terrain_map for basic usage
        self.terrain_grid_pub.publish(msg)
    
    def calculate_cell_metrics(self, z_values):
        """Calculate terrain metrics for a grid cell."""
        if len(z_values) < self.min_points:
            return None, None, None
        
        z_arr = np.array(z_values)
        
        # Height statistics
        z_min = np.min(z_arr)
        z_max = np.max(z_arr)
        z_mean = np.mean(z_arr)
        
        # Roughness = standard deviation of heights
        roughness = np.std(z_arr)
        
        # Height range (step height)
        height_range = z_max - z_min
        
        # Estimate slope from height variation (simplified)
        # Real slope would need neighboring cells
        slope_estimate = math.atan2(height_range, self.resolution) * 180 / math.pi
        
        return roughness, slope_estimate, height_range
    
    def compute_traversability(self, roughness, slope, height_range):
        """Compute traversability cost (0-100, where 0=free, 100=obstacle)."""
        if roughness is None:
            return -1  # Unknown
        
        # Normalize factors to 0-1
        slope_factor = min(slope / self.slope_threshold, 1.0)
        roughness_factor = min(roughness / self.roughness_threshold, 1.0)
        height_factor = min(height_range / self.max_height, 1.0)
        
        # Weighted combination
        cost = (slope_factor * 0.4 + roughness_factor * 0.4 + height_factor * 0.2) * 100
        
        # Threshold for obstacles
        if slope > self.slope_threshold * 1.5 or roughness > self.roughness_threshold * 2:
            cost = 100  # Lethal
        
        return int(min(cost, 100))
    
    def publish_terrain_maps(self):
        """Publish computed terrain analysis maps."""
        if not self.terrain_cells:
            return
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        
        # Initialize grids
        traversability_data = [-1] * (self.grid_size * self.grid_size)  # Unknown
        slope_data = [-1] * (self.grid_size * self.grid_size)
        
        cells_processed = 0
        
        for (gx, gy), z_values in self.terrain_cells.items():
            if len(z_values) < self.min_points:
                continue
            
            roughness, slope, height_range = self.calculate_cell_metrics(z_values)
            
            if roughness is not None:
                idx = gy * self.grid_size + gx
                
                # Traversability cost
                traversability_data[idx] = self.compute_traversability(roughness, slope, height_range)
                
                # Slope map (normalized to 0-100)
                slope_data[idx] = int(min(slope / 90.0 * 100, 100))
                
                cells_processed += 1
        
        # Create and publish traversability map
        trav_grid = OccupancyGrid()
        trav_grid.header = header
        trav_grid.info.resolution = self.resolution
        trav_grid.info.width = self.grid_size
        trav_grid.info.height = self.grid_size
        trav_grid.info.origin.position.x = -self.origin_offset
        trav_grid.info.origin.position.y = -self.origin_offset
        trav_grid.info.origin.position.z = 0.0
        trav_grid.info.origin.orientation.w = 1.0
        trav_grid.data = traversability_data
        self.traversability_pub.publish(trav_grid)
        
        # Create and publish slope map
        slope_grid = OccupancyGrid()
        slope_grid.header = header
        slope_grid.info = trav_grid.info
        slope_grid.data = slope_data
        self.slope_map_pub.publish(slope_grid)
        
        if cells_processed > 0:
            self.get_logger().debug(f'Published terrain maps: {cells_processed} cells analyzed')
    
    def publish_filtered_cloud(self, points, header):
        """Publish filtered point cloud."""
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack point data
        data = bytearray()
        for x, y, z in points:
            data.extend(struct.pack('fff', x, y, z))
        msg.data = bytes(data)
        
        self.filtered_cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()