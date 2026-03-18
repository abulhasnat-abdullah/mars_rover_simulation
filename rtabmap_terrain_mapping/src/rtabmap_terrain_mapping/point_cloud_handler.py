#!/usr/bin/env python3
"""
Point Cloud Handler for RTAB-Map Terrain Mapping
Manages point cloud data transformations and filtering.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudHandler(Node):
    def __init__(self):
        super().__init__('point_cloud_handler')
        
        # Parameters
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('min_points', 10)
        self.declare_parameter('input_topic', '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('output_topic', '/terrain/processed_cloud')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Subscriber
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            10
        )
        
        # Publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )
        
        self.get_logger().info(f'Point Cloud Handler: {input_topic} -> {output_topic}')
    
    def cloud_callback(self, msg: PointCloud2):
        """Process and republish point cloud."""
        # Forward the cloud (add filtering logic here as needed)
        self.cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudHandler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()