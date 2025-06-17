#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import time
import struct
import sys

# CAN interface
CAN_INTERFACE = 'can0'

# Motor IDs with ordered keys for consistent display
MOTORS_ORDERED = [
    'shoulder_pitch',      # motor_a (0x141)
    'shoulder_roll',       # motor_b (0x142)
    'forearm_rotation',    # motor_e (0x143)
    'wrist_pitch',         # motor_f (0x144)
    'upper_arm_rotation',  # motor_c (0x145)
    'elbow_pitch',         # motor_d (0x146)
    'wrist_yaw'            # motor_g (0x147)
]

# Motor IDs mapping
MOTORS = {
    'shoulder_pitch': 0x141,      # motor_a
    'shoulder_roll': 0x142,       # motor_b
    'upper_arm_rotation': 0x145,  # motor_c
    'elbow_pitch': 0x146,         # motor_d
    'forearm_rotation': 0x143,    # motor_e
    'wrist_pitch': 0x144,         # motor_f
    'wrist_yaw': 0x147            # motor_g
}

def init_can_bus():
    """Init CAN bus."""
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
        return bus
    except OSError:
        print("Failed to connect to CAN bus")
        sys.exit(1)

def position_control(bus, motor_id, pos_deg, speed_dps=800):
    """Send pos cmd with no prints."""
    speed = int(speed_dps)
    p_int = int(pos_deg * 100)
    data = [0xA4, 0] + list(struct.pack('<H', speed)) + list(struct.pack('<i', p_int))
    try:
        msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        bus.send(msg)
        bus.recv(timeout=0.2)
        return True
    except can.CanError:
        return False

def stop_motor(bus, motor_id):
    """Stop motor with no prints."""
    msg = can.Message(arbitration_id=motor_id, data=[0x81, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError:
        pass

class JointToCanNode(Node):
    def __init__(self):
        super().__init__('joint_to_can')
        
        # Initialize CAN bus
        self.bus = init_can_bus()
        
        # Store last commanded positions
        self.last_positions = {joint: 0.0 for joint in MOTORS}
        self.changed = False  # Track if any position changed
        self.last_print_time = time.time()
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        print('Joint to CAN bridge started')
        print('Motors: ' + ', '.join(MOTORS_ORDERED))
        print('Watching for joint states...')
    
    def joint_state_callback(self, msg):
        self.changed = False
        
        for i, joint_name in enumerate(msg.name):
            if joint_name in MOTORS:
                joint_rad = msg.position[i]
                
                # Convert to degrees with offset for motors
                motor_angle = joint_rad * 57.3 + 180.0
                
                motor_id = MOTORS[joint_name]
                
                # Only command if position changed significantly
                if abs(motor_angle - self.last_positions.get(joint_name, 0)) > 0.5:
                    # Send command to motor
                    success = position_control(self.bus, motor_id, motor_angle)
                    if success:
                        self.last_positions[joint_name] = motor_angle
                        self.changed = True
        
        # Print positions periodically or when they change
        current_time = time.time()
        if self.changed or current_time - self.last_print_time > 1.0:
            self.print_positions()
            self.last_print_time = current_time
    
    def print_positions(self):
        """Print all motor positions in a compact array format."""
        positions = [f"{self.last_positions.get(joint, 0.0):.1f}" for joint in MOTORS_ORDERED]
        print(f"Motor positions: [{', '.join(positions)}]")
    
    def shutdown(self):
        # Stop all motors before shutting down
        print('Stopping all motors')
        for motor_id in MOTORS.values():
            stop_motor(self.bus, motor_id)
        self.bus.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    node = JointToCanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()