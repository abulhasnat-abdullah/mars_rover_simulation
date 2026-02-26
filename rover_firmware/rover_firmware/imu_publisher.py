#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class ESP32ImuBridge(Node):
    def __init__(self):
        super().__init__("esp32_imu_bridge")
        
        # Serial connection to ESP32
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # ROS2 publisher
        self.imu_pub = self.create_publisher(Imu, "/imu/out", 10)
        
        # Timer for reading serial
        self.timer = self.create_timer(0.01, self.read_serial)
        
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "base_footprint"
        
    def read_serial(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                values = line.split(',')
                
                if len(values) == 6:
                    ax, ay, az, gx, gy, gz = map(int, values)
                    
                    # Convert to proper units
                    self.imu_msg.linear_acceleration.x = ax / 1670.13
                    self.imu_msg.linear_acceleration.y = ay / 1670.13
                    self.imu_msg.linear_acceleration.z = az / 1670.13
                    
                    self.imu_msg.angular_velocity.x = gx / 7509.55
                    self.imu_msg.angular_velocity.y = gy / 7509.55
                    self.imu_msg.angular_velocity.z = gz / 7509.55
                    
                    self.imu_msg.header.stamp = self.get_clock().now().to_msg()
                    self.imu_pub.publish(self.imu_msg)
                    
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

def main():
    rclpy.init()
    node = ESP32ImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()