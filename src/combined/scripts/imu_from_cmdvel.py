#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class IMUFromCmdVel(Node):
    def __init__(self):
        super().__init__('imu_from_cmdvel')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.pub = self.create_publisher(Imu, 'imu', 10)

        self.yaw = 0.0
        self.prev_time = self.get_clock().now()
        self.twist = Twist()

        self.timer = self.create_timer(0.05, self.publish_imu)

    def on_cmd_vel(self, msg):
        self.twist = msg

    def publish_imu(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        angular_z = self.twist.angular.z
        linear_x = self.twist.linear.x

        self.yaw += angular_z * dt
        q = self.quaternion_from_yaw(self.yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation = q
        imu_msg.angular_velocity.z = angular_z
        imu_msg.linear_acceleration.x = linear_x * 2.0

        self.pub.publish(imu_msg)

    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

def main():
    rclpy.init()
    node = IMUFromCmdVel()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
