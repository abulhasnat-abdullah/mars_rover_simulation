#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        
        # Subscribe to Twist messages
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.twist_callback,
            10
        )
        
        # Publish TwistStamped messages
        self.pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Twist bridge started: /cmd_vel_nav (Twist) -> /cmd_vel (TwistStamped)')

    def twist_callback(self, msg):
        # Convert Twist to TwistStamped
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        
        # Publish
        self.pub.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()