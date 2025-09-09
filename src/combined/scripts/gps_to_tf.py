#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class GPSToTF(Node):
    def __init__(self):
        super().__init__('gps_to_tf')
        self.sub = self.create_subscription(NavSatFix, 'gps/fix', self.callback, 10)
        self.br = TransformBroadcaster(self)

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'gps_link'

        # Konversi GPS lat/lon ke koordinat lokal dummy
        # Di sini kita anggap pusat (0,0) = latitude -6.8915, longitude 107.6107
        base_lat = -6.8915
        base_lon = 107.6107

        scale = 111000.0  # kira-kira meter per derajat
        dx = (msg.longitude - base_lon) * scale
        dy = (msg.latitude  - base_lat) * scale

        t.transform.translation.x = dx
        t.transform.translation.y = dy
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # no rotation

        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(GPSToTF())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
