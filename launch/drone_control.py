#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import tty
import termios
import threading
import math

MSG = """
╔══════════════════════════════════════════╗
║           DRONE CONTROL PANEL           ║
╠══════════════════════════════════════════╣
║  MOVEMENT:                              ║
║  i       : Forward                      ║
║  ,       : Backward                     ║
║  j       : Left                         ║
║  l       : Right                        ║
║  t       : Up   ⬆️                      ║
║  b       : Down ⬇️                      ║
║  u / o   : Rotate Left / Right          ║
║  k       : STOP                         ║
╠══════════════════════════════════════════╣
║  SPEED:                                 ║
║  w       : Increase Speed  (+10%)       ║
║  x       : Decrease Speed  (-10%)       ║
╠══════════════════════════════════════════╣
║  H       : Return to Home  🏠           ║
║  q       : Quit                         ║
╚══════════════════════════════════════════╝
"""

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/drone/odom',
                                            self.odom_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.speed = 0.5
        self.turn  = 0.5
        self.returning = False
        self.create_timer(0.1, self.return_loop)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def return_loop(self):
        if not self.returning:
            return
        cmd = Twist()
        ex = 0.0 - self.x
        ey = 0.0 - self.y
        ez = 0.0 - self.z
        dist = math.sqrt(ex**2 + ey**2 + ez**2)
        if dist < 0.15:
            print('\n✅ Drone has landed at home position!')
            self.returning = False
            self.pub.publish(Twist())
        else:
            kp = 0.5
            cmd.linear.x = max(-0.5, min(0.5, kp * ex))
            cmd.linear.y = max(-0.5, min(0.5, kp * ey))
            cmd.linear.z = max(-0.5, min(0.5, kp * ez))
            self.pub.publish(cmd)

    def send(self, vx, vy, vz, wz):
        cmd = Twist()
        cmd.linear.x  = vx
        cmd.linear.y  = vy
        cmd.linear.z  = vz
        cmd.angular.z = wz
        self.pub.publish(cmd)

    def print_status(self):
        print(f'\r  Speed: {self.speed:.2f} m/s | '
              f'Turn: {self.turn:.2f} rad/s | '
              f'Pos -> x:{self.x:.2f} y:{self.y:.2f} z:{self.z:.2f}    ',
              end='', flush=True)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = DroneControl()
    settings = termios.tcgetattr(sys.stdin)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(MSG)
    print(f'  Initial Speed: {node.speed:.2f} m/s\n')

    try:
        while True:
            key = get_key(settings)

            if key == 'q':
                print('\n\nStopping drone...')
                node.send(0.0, 0.0, 0.0, 0.0)
                break

            elif key == 'w':
                node.speed = min(2.0, round(node.speed + 0.1, 2))
                node.turn  = min(2.0, round(node.turn  + 0.1, 2))
                print(f'\n  ⬆ Speed increased: {node.speed:.2f} m/s')

            elif key == 'x':
                node.speed = max(0.1, round(node.speed - 0.1, 2))
                node.turn  = max(0.1, round(node.turn  - 0.1, 2))
                print(f'\n  ⬇ Speed decreased: {node.speed:.2f} m/s')

            elif key == 'H':
                node.returning = True
                print('\n  🏠 Returning to home...')

            elif key == 'k':
                node.returning = False
                node.send(0.0, 0.0, 0.0, 0.0)
                print('\n  ⏹  Stopped')

            elif key == 'i':
                node.returning = False
                node.send(node.speed, 0.0, 0.0, 0.0)
            elif key == ',':
                node.returning = False
                node.send(-node.speed, 0.0, 0.0, 0.0)
            elif key == 'j':
                node.returning = False
                node.send(0.0, node.speed, 0.0, 0.0)
            elif key == 'l':
                node.returning = False
                node.send(0.0, -node.speed, 0.0, 0.0)
            elif key == 't':
                node.returning = False
                node.send(0.0, 0.0, node.speed, 0.0)
            elif key == 'b':
                node.returning = False
                node.send(0.0, 0.0, -node.speed, 0.0)
            elif key == 'u':
                node.returning = False
                node.send(0.0, 0.0, 0.0, node.turn)
            elif key == 'o':
                node.returning = False
                node.send(0.0, 0.0, 0.0, -node.turn)

            node.print_status()

    except Exception as e:
        print(f'\nError: {e}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.send(0.0, 0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
