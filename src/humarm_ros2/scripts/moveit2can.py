#!/usr/bin/env python3
"""
Bridge node between MoveIt/ROS2 Controllers and CAN-based motors.
Subscribe to trajectory commands from MoveIt and convert them to CAN commands for RMD-X motors.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import can
import struct
import numpy as np
import math
import time
import sys

# CAN interface
CAN_INTERFACE = 'can0'

# Motor IDs mapping each joint to its CAN ID
MOTORS = {
    'shoulder_pitch': 0x141,      # motor_a
    'shoulder_roll': 0x142,       # motor_b
    'upper_arm_rotation': 0x145,  # motor_c
    'elbow_pitch': 0x146,         # motor_d
    'forearm_rotation': 0x143,    # motor_e
    'wrist_pitch': 0x144,         # motor_f
    'wrist_yaw': 0x147            # motor_g
}

class MoveitToCanNode(Node):
    """
    ROS2 Node that bridges between MoveIt trajectory commands and CAN motor control.
    """
    def __init__(self):
        """Initialize the node, subscribers, and CAN connection."""
        super().__init__('moveit_to_can')
        
        # Parameters
        self.declare_parameter('can_interface', CAN_INTERFACE)
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        
        # Store joint positions
        self.joint_positions = {}  # Current joint positions
        self.last_commands = {}    # Last commanded positions to avoid duplicate commands
        
        # Initialize CAN bus
        self.init_can_bus()
        
        # Subscribe to joint state topic to monitor actual robot state
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Subscribe to trajectory commands from MoveIt
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',  # Topic name from your MoveIt controller config
            self.trajectory_callback,
            10)
        
        # Status publisher for diagnostic purposes
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info('=================================')
        self.get_logger().info('MoveIt to CAN bridge initialized')
        self.get_logger().info(f'Using CAN interface: {self.can_interface}')
        self.get_logger().info('Listening for trajectory commands')
        self.get_logger().info('=================================')
        
    def init_can_bus(self):
        """Initialize the CAN bus connection."""
        try:
            self.bus = can.interface.Bus(channel=self.can_interface, interface='socketcan')
            self.get_logger().info(f'Successfully connected to CAN bus: {self.can_interface}')
        except OSError as e:
            self.get_logger().error(f'Failed to connect to CAN bus: {e}')
            self.get_logger().error('Running in simulation mode (no motor commands will be sent)')
            self.bus = None
    
    def position_control(self, motor_id, pos_deg, speed_dps=800):
        """
        Send position control command to motor using RMD protocol.
        
        Args:
            motor_id: CAN ID of the motor
            pos_deg: Target position in degrees
            speed_dps: Maximum speed in degrees per second
            
        Returns:
            bool: True if command was sent successfully, False otherwise
        """
        if self.bus is None:
            self.get_logger().debug(f'Simulating motor {motor_id} move to {pos_deg:.2f}° at {speed_dps} deg/s')
            return False
        
        # Ensure parameters are within valid ranges
        speed = max(0, min(int(speed_dps), 1000))
        p_int = int(pos_deg * 100)  # Position in 0.01 degree increments
        
        # Create the command
        data = [0xA4, 0] + list(struct.pack('<H', speed)) + list(struct.pack('<i', p_int))
        
        try:
            # Create and send CAN message
            msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
            self.bus.send(msg)
            
            # Try to receive response (non-blocking)
            try:
                response = self.bus.recv(timeout=0.05)
                self.get_logger().debug(f'Motor {motor_id} response: {response}')
            except:
                pass
                
            return True
        except can.CanError as e:
            self.get_logger().error(f'CAN error when commanding motor {motor_id}: {e}')
            return False
    
    def stop_motor(self, motor_id):
        """
        Send stop command to a motor.
        
        Args:
            motor_id: CAN ID of the motor
        """
        if self.bus is None:
            return
            
        try:
            msg = can.Message(arbitration_id=motor_id, data=[0x81, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
            self.bus.send(msg)
            self.get_logger().info(f'Sent stop command to motor {motor_id}')
        except can.CanError as e:
            self.get_logger().error(f'Error stopping motor {motor_id}: {e}')
    
    def joint_state_callback(self, msg):
        """
        Callback for joint state messages - tracks the current robot state.
        
        Args:
            msg: JointState message
        """
        for i, name in enumerate(msg.name):
            if name in MOTORS:
                self.joint_positions[name] = msg.position[i]
    
    def trajectory_callback(self, msg):
        """
        Callback for trajectory command messages from MoveIt.
        
        Args:
            msg: JointTrajectory message
        """
        if not msg.points:
            self.get_logger().warn("Received empty trajectory")
            return
        
        # Get the final point in the trajectory
        # For a more advanced implementation, you could execute the entire trajectory
        point = msg.points[-1]
        
        self.get_logger().info("Received trajectory command")
        
        # Process each joint in the trajectory
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in MOTORS:
                # Get target position in radians
                joint_position = point.positions[i]
                
                # Convert to degrees with offset for motor
                # RMD motors typically use 0-360° with 180° as center
                motor_angle = joint_position * 57.3 + 180.0  # rad to deg conversion
                
                # Get corresponding motor ID
                motor_id = MOTORS[joint_name]
                
                # Only send command if position changed significantly
                if (joint_name not in self.last_commands or 
                        abs(motor_angle - self.last_commands.get(joint_name, 0)) > 0.5):
                    
                    self.get_logger().info(f'Moving {joint_name} to {motor_angle:.2f}° (from {joint_position:.4f} rad)')
                    
                    # Send the command to the motor
                    success = self.position_control(motor_id, motor_angle)
                    if success:
                        self.last_commands[joint_name] = motor_angle
                else:
                    self.get_logger().debug(f'Skipping {joint_name} command (change too small)')
            else:
                self.get_logger().warn(f'Trajectory contains unknown joint: {joint_name}')
    
    def publish_status(self):
        """Periodically publish node status for monitoring purposes."""
        if not self.joint_positions:
            return
            
        # Log the current state of all joints
        status_str = "Current joint positions: "
        for joint_name, position in self.joint_positions.items():
            motor_angle = position * 57.3 + 180.0  # rad to deg
            status_str += f"{joint_name}={motor_angle:.1f}° "
            
        self.get_logger().info(status_str)
        
        # Check CAN bus status
        if self.bus is None:
            self.get_logger().warn("Not connected to CAN bus - running in simulation mode")
            # Try to reinitialize CAN bus
            self.init_can_bus()
    
    def execute_trajectory(self, trajectory, joint_names):
        """
        Execute a full trajectory with time-based interpolation.
        
        Args:
            trajectory: List of JointTrajectoryPoint objects
            joint_names: List of joint names corresponding to trajectory points
        """
        if not trajectory:
            return
            
        # Get start time
        start_time = self.get_clock().now()
        
        # Execute each trajectory point at the appropriate time
        for point in trajectory:
            # Calculate time to wait
            target_time = start_time + Duration(seconds=point.time_from_start.sec,
                                              nanoseconds=point.time_from_start.nanosec)
            current_time = self.get_clock().now()
            
            # Wait until the appropriate time
            if target_time > current_time:
                time_to_wait = (target_time.nanoseconds - current_time.nanoseconds) / 1e9
                if time_to_wait > 0:
                    time.sleep(time_to_wait)
            
            # Send commands for this trajectory point
            for i, joint_name in enumerate(joint_names):
                if joint_name in MOTORS:
                    joint_position = point.positions[i]
                    motor_angle = joint_position * 57.3 + 180.0
                    motor_id = MOTORS[joint_name]
                    
                    # Send command to motor
                    self.position_control(motor_id, motor_angle)
    
    def stop_all_motors(self):
        """Stop all motors and clean up."""
        self.get_logger().info("Stopping all motors")
        for joint_name, motor_id in MOTORS.items():
            self.stop_motor(motor_id)
            
        if self.bus:
            try:
                self.bus.shutdown()
                self.get_logger().info("CAN bus shut down")
            except Exception as e:
                self.get_logger().error(f"Error shutting down CAN bus: {e}")

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = MoveitToCanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
        node.get_logger().error(f"Error details: {type(e).__name__}: {e}")
    finally:
        node.stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()