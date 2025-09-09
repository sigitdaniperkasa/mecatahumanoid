#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import random
import math
import time

class FakeIMU(Node):
    def __init__(self):
        super().__init__('fake_imu')
        self.pub = self.create_publisher(Imu, 'imu', 10)
        self.timer = self.create_timer(0.1, self.publish_imu)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulasi data acak
        msg.linear_acceleration.x = random.uniform(-0.2, 0.2)
        msg.linear_acceleration.y = random.uniform(-0.2, 0.2)
        msg.linear_acceleration.z = 9.8  # gravitasi

        msg.angular_velocity.x = random.uniform(-0.01, 0.01)
        msg.angular_velocity.y = random.uniform(-0.01, 0.01)
        msg.angular_velocity.z = random.uniform(-0.1, 0.1)

        # Orientasi sebagai quaternion (tetap untuk sekarang)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
        msg.orientation = q

        self.pub.publish(msg)
        print("📡 fake_imu mem-publish ke /imu")

def main():
    rclpy.init()
    node = FakeIMU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
