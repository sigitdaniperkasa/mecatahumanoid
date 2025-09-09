#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math, time, random

class FakeGPS(Node):
    def __init__(self):
        super().__init__('fake_gps')
        self.pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(1.0, self.publish_gps)  # 1 Hz
        self.start_time = time.time()

    def publish_gps(self):
        t = time.time() - self.start_time

        # Simulasi lintasan (bisa diganti logika yang lebih realistis)
        base_lat = -6.8915
        base_lon = 107.6107
        lat = base_lat + 0.0001 * math.sin(t / 10.0)
        lon = base_lon + 0.0001 * math.cos(t / 10.0)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 738.0  # misalnya ketinggian Bandung
        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # GPS

        self.pub.publish(msg)
        self.get_logger().info(f"GPS: {lat:.6f}, {lon:.6f}")

def main():
    rclpy.init()
    rclpy.spin(FakeGPS())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
