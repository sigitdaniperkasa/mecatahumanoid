#!/usr/bin/env python3
import math, time, curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

# ====== TUNABLES ======
LIN_MAX   = 2.3     # m/s  (top speed)
ANG_MAX   = 2.8     # rad/s
LIN_ACCEL = 4.0     # m/s^2  (berapa cepat naik saat ditahan)
ANG_ACCEL = 5.0     # rad/s^2
RATE_HZ   = 60.0    # loop rate
HOLD_GRACE = 0.20   # detik: waktu toleransi "tombol masih dianggap ditekan"

HELP_TEXT = """
Arrow keys to drive:
  ↑ : forward   ↓ : backward   ← : turn left   → : turn right
Hold = speed increases; release = instant stop (after a tiny 0.2s grace).
Press 'q' to quit.
"""

class TeleopRViz(Node):
    def __init__(self, stdscr):
        super().__init__('teleop_rviz')
        self.stdscr = stdscr
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.br = TransformBroadcaster(self)

        # state pose & speed
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.last = time.monotonic()

        # state hold & direction
        self.hold_fwd  = 0.0     # akumulasi waktu tahan untuk linear
        self.hold_turn = 0.0     # akumulasi waktu tahan untuk angular
        self.lin_dir   = 0       # -1, 0, +1
        self.ang_dir   = 0       # -1, 0, +1
        self.last_lin_press = 0.0
        self.last_ang_press = 0.0

        self.timer = self.create_timer(1.0 / RATE_HZ, self.loop)

        # setup curses
        self.stdscr.nodelay(True); self.stdscr.keypad(True)
        curses.cbreak(); curses.noecho()
        self.draw_help()

    def draw_help(self):
        self.stdscr.clear()
        for i, line in enumerate(HELP_TEXT.strip().splitlines()):
            self.stdscr.addstr(i, 0, line)
        self.stdscr.refresh()

    def read_keys(self):
        keys = set()
        while True:
            k = self.stdscr.getch()
            if k == -1: break
            keys.add(k)
        return keys

    def loop(self):
        now = time.monotonic(); dt = now - self.last; self.last = now
        keys = self.read_keys()

        # update direction & last press time jika ada event
        if curses.KEY_UP in keys:
            self.lin_dir = +1
            self.last_lin_press = now
        elif curses.KEY_DOWN in keys:
            self.lin_dir = -1
            self.last_lin_press = now

        if curses.KEY_LEFT in keys:
            self.ang_dir = +1
            self.last_ang_press = now
        elif curses.KEY_RIGHT in keys:
            self.ang_dir = -1
            self.last_ang_press = now

        if ord('q') in keys:
            rclpy.shutdown()
            return

        # apakah masih dianggap "menekan" berdasarkan grace window?
        pressing_lin = (now - self.last_lin_press) <= HOLD_GRACE and self.lin_dir != 0
        pressing_ang = (now - self.last_ang_press) <= HOLD_GRACE and self.ang_dir != 0

        # akumulasi waktu tahan (kalau tidak menekan, reset)
        self.hold_fwd  = (self.hold_fwd + dt) if pressing_lin else 0.0
        self.hold_turn = (self.hold_turn + dt) if pressing_ang else 0.0

        # target speed tumbuh dengan waktu tahan sampai MAX
        want_v = 0.0; want_w = 0.0
        if pressing_lin:
            want_v = self.lin_dir * min(LIN_ACCEL * self.hold_fwd, LIN_MAX)
        if pressing_ang:
            want_w = self.ang_dir * min(ANG_ACCEL * self.hold_turn, ANG_MAX)

        # smoothing ringan supaya enak, tapi tetap instant stop kalau lewat grace
        alpha = 0.5
        self.v = alpha * want_v + (1 - alpha) * self.v
        self.w = alpha * want_w + (1 - alpha) * self.w

        if not pressing_lin:
            self.v = 0.0
            self.lin_dir = 0
        if not pressing_ang:
            self.w = 0.0
            self.ang_dir = 0

        # integrasi pose (buat TF)
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        # publish cmd_vel
        tw = Twist(); tw.linear.x = self.v; tw.angular.z = self.w
        self.pub.publish(tw)

        # broadcast TF map->base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        cy = math.cos(self.yaw * 0.5); sy = math.sin(self.yaw * 0.5)
        t.transform.rotation.z = sy; t.transform.rotation.w = cy
        self.br.sendTransform(t)

        # HUD
        self.stdscr.addstr(4, 0, f"v={self.v:+.2f} m/s  w={self.w:+.2f} rad/s  x={self.x:+.2f} y={self.y:+.2f} yaw={self.yaw:+.2f}   ")
        self.stdscr.refresh()

def main():
    rclpy.init()
    try:
        curses.wrapper(lambda stdscr: rclpy.spin(TeleopRViz(stdscr)))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

