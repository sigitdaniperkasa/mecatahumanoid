#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random

class FakeLidar(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'  # Sesuai URDF!

        msg.angle_min = -math.pi / 2
        msg.angle_max = math.pi / 2
        msg.angle_increment = math.pi / 180
        msg.range_min = 0.12
        msg.range_max = 3.5

        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [random.uniform(0.3, 3.0) for _ in range(num_readings)]
        msg.intensities = [1.0] * num_readings

        self.pub.publish(msg)
        print("📡 fake_lidar mem-publish ke /scan")

def main():
    rclpy.init()
    node = FakeLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

