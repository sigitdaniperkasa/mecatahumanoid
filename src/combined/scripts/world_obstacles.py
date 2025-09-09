#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

class WorldObstacles(Node):
    def __init__(self):
        super().__init__('world_obstacles')
        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.send_markers)

    def send_markers(self):
        markers = []
        positions = [
            (2.0, 1.5),
            (3.5, -1.0),
            (1.0, -2.0),
            (4.0, 2.0),
            (2.5, 0.0)
        ]

        for i, (x, y) in enumerate(positions):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.5
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 1.0
            m.color.r = 0.2
            m.color.g = 0.6
            m.color.b = 0.2
            m.color.a = 1.0
            m.lifetime = Duration(sec=0)  # tetap muncul selamanya
            self.pub.publish(m)

def main():
    rclpy.init()
    rclpy.spin(WorldObstacles())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
