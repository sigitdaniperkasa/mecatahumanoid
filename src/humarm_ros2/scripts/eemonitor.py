#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from std_msgs.msg import ColorRGBA
import math

class EndEffectorMonitor(Node):
    def __init__(self):
        super().__init__('end_effector_monitor')
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Base frame and end-effector frame
        self.base_frame = 'base_link'
        self.ee_frame = 'hand_link'
        
        # Publisher for the position marker
        self.marker_pub = self.create_publisher(Marker, 'end_effector_marker', 10)
        
        # Interactive marker server for setting positions
        self.marker_server = InteractiveMarkerServer(self, 'end_effector_control')
        
        # Create timer for updating position
        self.create_timer(0.1, self.update_position)
        
        # Create the interactive marker when node starts
        self.create_interactive_marker()
        
        self.get_logger().info('End-Effector Monitor started')
        self.get_logger().info('Use interactive marker to set desired positions')
    
    def update_position(self):
        """Update and display the current end-effector position."""
        try:
            # Get the transform from base to end-effector
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time())
            
            # Extract position
            pos = transform.transform.translation
            
            # Log position at a lower rate (every ~1 second)
            if not hasattr(self, 'last_log_time') or self.get_clock().now().to_msg().sec - self.last_log_time > 1:
                self.get_logger().info(f'End-effector position: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] meters')
                self.last_log_time = self.get_clock().now().to_msg().sec
            
            # Create position marker
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'end_effector_position'
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            # Position the text just above the end-effector
            marker.pose.position.x = pos.x
            marker.pose.position.y = pos.y
            marker.pose.position.z = pos.z + 0.05  # Slightly above the end-effector
            
            marker.pose.orientation.w = 1.0
            
            # Format position as text
            marker.text = f'[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]'
            
            # Set marker scale and color
            marker.scale.z = 0.05  # Text size
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            # Publish the marker
            self.marker_pub.publish(marker)
            
            # Update the interactive marker position if it exists
            try:
                int_marker = self.marker_server.get("goal_marker")
                # Only update if not being dragged
                if int_marker and not hasattr(self, 'is_dragging') or not self.is_dragging:
                    int_marker.pose.position.x = pos.x
                    int_marker.pose.position.y = pos.y
                    int_marker.pose.position.z = pos.z
                    
                    self.marker_server.insert(int_marker, feedback_callback=self.process_feedback)
                    self.marker_server.applyChanges()
            except KeyError:
                pass
            
        except TransformException as e:
            self.get_logger().warn(f'Could not get transform: {e}')
    
    def create_interactive_marker(self):
        """Create an interactive marker for setting goal positions."""
        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.base_frame
        int_marker.name = 'goal_marker'
        int_marker.description = 'Goal Position'
        
        # Set initial position (default if we don't know end-effector position yet)
        int_marker.pose.position.x = 0.3
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.3
        
        # Create a sphere marker
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.6
        
        # Create control with marker
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        # X axis control
        control = InteractiveMarkerControl()
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)
        
        # Y axis control
        control = InteractiveMarkerControl()
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)
        
        # Z axis control
        control = InteractiveMarkerControl()
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        int_marker.controls.append(control)
        
        # Add to server
        self.marker_server.insert(int_marker, feedback_callback=self.process_feedback)
        self.marker_server.applyChanges()
    
    def process_feedback(self, feedback):
        """Process feedback from interactive marker."""
        if feedback.event_type == feedback.MOUSE_DOWN:
            self.is_dragging = True
            
        elif feedback.event_type == feedback.MOUSE_UP:
            self.is_dragging = False
            
            # Get the position from the marker
            pos = feedback.pose.position
            
            # Log the goal position
            self.get_logger().info(f'Goal position set: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]')
            
            # Create a goal marker
            self.display_goal_marker(pos)
    
    def display_goal_marker(self, position):
        """Display a marker at the goal position."""
        # Create position marker
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal_position'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position the text
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z + 0.1  # Above the end-effector
        
        marker.pose.orientation.w = 1.0
        
        # Format position as text
        marker.text = f'GOAL: [{position.x:.3f}, {position.y:.3f}, {position.z:.3f}]'
        
        # Set marker scale and color
        marker.scale.z = 0.05  # Text size
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Make it persist for a while
        marker.lifetime.sec = 15
        
        # Publish the marker
        self.marker_pub.publish(marker)
        
        # Also publish a line connecting current position to goal
        self.publish_path_marker(position)
    
    def publish_path_marker(self, goal_position):
        """Publish a line marker from current end-effector position to goal."""
        try:
            # Get current end-effector position
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time())
            
            current_pos = transform.transform.translation
            
            # Create line marker
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'goal_path'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Add points for the line
            p1 = Point()
            p1.x, p1.y, p1.z = current_pos.x, current_pos.y, current_pos.z
            
            p2 = Point()
            p2.x, p2.y, p2.z = goal_position.x, goal_position.y, goal_position.z
            
            marker.points = [p1, p2]
            
            # Set line properties
            marker.scale.x = 0.01  # Line width
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            # Make it persist for a while
            marker.lifetime.sec = 15
            
            # Publish the marker
            self.marker_pub.publish(marker)
            
            # Calculate distance
            dx = goal_position.x - current_pos.x
            dy = goal_position.y - current_pos.y
            dz = goal_position.z - current_pos.z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            self.get_logger().info(f'Distance to goal: {distance:.3f} meters')
            
        except TransformException:
            self.get_logger().warn('Could not get current transform for path marker')

def main(args=None):
    rclpy.init(args=args)
    
    node = EndEffectorMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()