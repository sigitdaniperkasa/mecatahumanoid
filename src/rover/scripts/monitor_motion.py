#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

def yaw_from_quat_z_w(z, w):
    # planar yaw dari quaternion (z, w)
    return math.atan2(2.0*w*z, 1.0 - 2.0*z*z)

def compass_heading(deg):
    # 16-arah kompas
    dirs = ['N','NNE','NE','ENE','E','ESE','SE','SSE',
            'S','SSW','SW','WSW','W','WNW','NW','NNW']
    idx = int((deg + 11.25) // 22.5) % 16
    return dirs[idx]

class MotionMonitor(Node):
    def __init__(self):
        super().__init__('motion_monitor')
        # baca command kecepatan
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)
        # baca TF pose
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

        self.v = 0.0         # linear.x sekarang
        self.w = 0.0         # angular.z sekarang
        self.prev_v = 0.0    # untuk hitung akselerasi
        self.prev_t = time.monotonic()

        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

    def on_twist(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def loop(self):
        now = time.monotonic()
        dt = now - self.prev_t
        if dt <= 0.0:
            dt = 1e-6

        # akselerasi dari delta v / dt
        a = (self.v - self.prev_v) / dt
        self.prev_v = self.v
        self.prev_t = now

        # arah gerak berdasarkan ambang kecil
        if   self.v > 0.02: move = 'forward'
        elif self.v < -0.02: move = 'backward'
        else: move = 'stopped'

        if   self.w > 0.05: turn = 'turning left'
        elif self.w < -0.05: turn = 'turning right'
        else: turn = 'not turning'

        # pose dari TF map->base_link
        x = y = yaw_deg = None
        head_txt = 'N/A'
        try:
            tf = self.buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.rotation.z
            wq = tf.transform.rotation.w
            yaw = yaw_from_quat_z_w(z, wq)
            yaw_deg = (math.degrees(yaw) + 360.0) % 360.0
            head_txt = f"{yaw_deg:6.1f}° {compass_heading(yaw_deg)}"
        except Exception:
            pass  # belum ada TF? tidak apa-apa, tampilkan N/A

        # tampilan HUD di terminal
        print("\033[2J\033[H", end="")  # clear + home
        print("=== Rover Motion Monitor ===")
        print("Source: /cmd_vel  |  TF: map -> base_link")
        print("")
        print(f"linear v  : {self.v:+.3f} m/s")
        print(f"angular w : {self.w:+.3f} rad/s")
        print(f"accel     : {a:+.3f} m/s^2")
        print(f"move      : {move:>8}   |   {turn}")
        if x is not None and yaw_deg is not None:
            print(f"pose (x,y): ({x:+.2f}, {y:+.2f}) m")
            print(f"heading   : {head_txt}")
        else:
            print("pose/heading: N/A (waiting for TF 'map->base_link')")
        print("\nPress Ctrl+C to exit.")

def main():
    rclpy.init()
    node = MotionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
