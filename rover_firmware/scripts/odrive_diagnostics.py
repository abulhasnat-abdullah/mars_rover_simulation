#!/usr/bin/env python3
"""
ODrive Diagnostics Monitor
Monitors ODrive errors, temperatures, and status in real-time

Place in: rover_firmware/scripts/odrive_diagnostics.py
Make executable: chmod +x odrive_diagnostics.py

Usage: ros2 run rover_firmware odrive_diagnostics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import sys

class ODriveDiagnostics(Node):
    def __init__(self):
        super().__init__('odrive_diagnostics')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Store latest data
        self.latest_data = {}
        
        # Error code mappings
        self.axis_errors = {
            0x00000001: "INITIALIZING",
            0x00000002: "SYSTEM_LEVEL",
            0x00000004: "TIMING_ERROR",
            0x00000008: "MISSING_ESTIMATE",
            0x00000010: "BAD_CONFIG",
            0x00000020: "DRV_FAULT",
            0x00000040: "MISSING_INPUT",
            0x00000080: "DC_BUS_OVER_REGEN_CURRENT",
            0x00000100: "DC_BUS_OVER_CURRENT",
            0x00000200: "DC_BUS_UNDERVOLTAGE",
            0x00000400: "DC_BUS_OVERVOLTAGE",
            0x00000800: "PRE_CHARGE_TIMEOUT",
            0x00001000: "BRAKE_DEADTIME_VIOLATION",
            0x00002000: "UNEXPECTED_TIMER_CALLBACK",
        }
        
        self.motor_errors = {
            0x0001: "PHASE_RESISTANCE_OUT_OF_RANGE",
            0x0002: "PHASE_INDUCTANCE_OUT_OF_RANGE",
            0x0004: "ADC_FAILED",
            0x0008: "DRV_FAULT",
            0x0010: "CONTROL_DEADLINE_MISSED",
            0x0020: "NOT_IMPLEMENTED_MOTOR_TYPE",
            0x0040: "BRAKE_CURRENT_OUT_OF_RANGE",
            0x0080: "MODULATION_MAGNITUDE",
            0x0100: "BRAKE_DEADTIME_VIOLATION",
            0x0200: "UNEXPECTED_TIMER_CALLBACK",
            0x0400: "CURRENT_SENSE_SATURATION",
            0x0800: "CURRENT_LIMIT_VIOLATION",
        }
        
        self.encoder_errors = {
            0x0001: "UNSTABLE_GAIN",
            0x0002: "CPR_POLEPAIRS_MISMATCH",
            0x0004: "NO_RESPONSE",
            0x0008: "UNSUPPORTED_ENCODER_MODE",
            0x0010: "ILLEGAL_HALL_STATE",
            0x0020: "INDEX_NOT_FOUND_YET",
            0x0040: "ABS_SPI_TIMEOUT",
            0x0080: "ABS_SPI_COM_FAIL",
            0x0100: "ABS_SPI_NOT_READY",
            0x0200: "HALL_NOT_CALIBRATED_YET",
        }
        
        self.controller_errors = {
            0x01: "OVERSPEED",
            0x02: "INVALID_INPUT_MODE",
            0x04: "UNSTABLE_GAIN",
            0x08: "INVALID_MIRROR_AXIS",
            0x10: "INVALID_LOAD_ENCODER",
            0x20: "INVALID_ESTIMATE",
        }
        
        # Create timer for periodic display
        self.timer = self.create_timer(2.0, self.display_status)
        
        self.get_logger().info("ODrive Diagnostics Monitor Started")
        self.get_logger().info("=" * 80)
    
    def joint_callback(self, msg):
        """Store latest joint state data"""
        for i, name in enumerate(msg.name):
            self.latest_data[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0,
            }
    
    def decode_errors(self, error_code, error_dict):
        """Decode error codes into human-readable strings"""
        errors = []
        for code, name in error_dict.items():
            if int(error_code) & code:
                errors.append(name)
        return errors if errors else ["NO_ERROR"]
    
    def display_status(self):
        """Display current status of all ODrive joints"""
        if not self.latest_data:
            self.get_logger().warn("No joint state data received yet...")
            return
        
        # Clear screen (optional)
        print("\n" * 2)
        print("=" * 80)
        print("ODrive Status Monitor".center(80))
        print("=" * 80)
        
        for joint_name in ['joint_FL', 'joint_FR', 'joint_BL', 'joint_BR']:
            if joint_name not in self.latest_data:
                continue
            
            data = self.latest_data[joint_name]
            
            print(f"\n{joint_name}:")
            print(f"  Position:  {data['position']:>8.3f} rad")
            print(f"  Velocity:  {data['velocity']:>8.3f} rad/s")
            print(f"  Effort:    {data['effort']:>8.3f} Nm")
        
        print("\n" + "=" * 80)
        
        # Additional diagnostics from extended joint states (if available)
        # You can subscribe to custom topics published by the hardware interface
        # for temperatures, errors, etc.
    
    def check_temperatures(self, joint_name, fet_temp, motor_temp):
        """Check if temperatures are within safe limits"""
        warnings = []
        
        if fet_temp > 80.0:
            warnings.append(f"{joint_name} FET temperature HIGH: {fet_temp}°C")
        
        if motor_temp > 100.0:
            warnings.append(f"{joint_name} Motor temperature HIGH: {motor_temp}°C")
        
        return warnings

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ODriveDiagnostics()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()