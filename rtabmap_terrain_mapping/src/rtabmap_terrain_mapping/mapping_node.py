#!/usr/bin/env python3
"""
Main Mapping Node for RTAB-Map Terrain Mapping
Coordinates RTAB-Map with terrain processing for your rover.
"""

import rclpy
import rclpy.qos
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


class MappingNode(Node):
    def __init__(self):
        super().__init__('rtabmap_mapping_coordinator')
        
        # Parameters matching your workspace
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('use_3d_mapping', True)
        
        self.use_3d = self.get_parameter('use_3d_mapping').value
        
        # Services
        self.save_map_srv = self.create_service(
            Trigger,
            'save_terrain_map',
            self.save_map_callback
        )
        
        self.reset_map_srv = self.create_service(
            Empty,
            'reset_terrain_map',
            self.reset_map_callback
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.map_callback,
            10
        )
        
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.cloud_map_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        
        # Publishers
        self.terrain_map_pub = self.create_publisher(
            OccupancyGrid,
            '/terrain_occupancy_grid',
            10
        )
        
        self.get_logger().info('RTAB-Map Mapping Coordinator initialized')
        self.get_logger().info(f'3D mapping: {self.use_3d}')
    
    def save_map_callback(self, request, response):
        """Save the current terrain map."""
        self.get_logger().info('Saving terrain map...')
        response.success = True
        response.message = 'Map save requested'
        return response
    
    def reset_map_callback(self, request, response):
        """Reset the terrain map."""
        self.get_logger().info('Resetting terrain map...')
        return response
    
    def map_callback(self, msg: OccupancyGrid):
        """Process incoming occupancy grid."""
        self.terrain_map_pub.publish(msg)
    
    def cloud_map_callback(self, msg: PointCloud2):
        """Process incoming 3D cloud map."""
        self.get_logger().debug('Received cloud map update')


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()