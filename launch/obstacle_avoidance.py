#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/drone/scan', self.scan_cb, 10)

        # Parameters
        self.safe_distance   = 1.5   # متر - مسافة الأمان
        self.danger_distance = 0.8   # متر - مسافة الخطر
        self.speed           = 0.4   # سرعة التقدم
        self.active          = False # تجنب العوائق مفعّل؟

        # State
        self.regions = {
            'front': 999.0,
            'front_left': 999.0,
            'front_right': 999.0,
            'left': 999.0,
            'right': 999.0,
            'back': 999.0,
        }

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('✅ Obstacle Avoidance Node Started!')

    def scan_cb(self, msg):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        def safe_min(start, end):
            vals = [r for r in ranges[start:end]
                    if not math.isnan(r) and not math.isinf(r) and r > 0.0]
            return min(vals) if vals else 999.0

        # تقسيم الـ Lidar إلى 6 مناطق
        step = n // 6
        self.regions['front']       = safe_min(0,          step)
        self.regions['front_left']  = safe_min(step,       step*2)
        self.regions['left']        = safe_min(step*2,     step*3)
        self.regions['back']        = safe_min(step*3,     step*4)
        self.regions['right']       = safe_min(step*4,     step*5)
        self.regions['front_right'] = safe_min(step*5,     n)

    def control_loop(self):
        if not self.active:
            return

        cmd = Twist()
        f  = self.regions['front']
        fl = self.regions['front_left']
        fr = self.regions['front_right']
        l  = self.regions['left']
        r  = self.regions['right']

        # ===== منطق تجنب العوائق =====

        # ✅ الطريق آمن - تقدم للأمام
        if f > self.safe_distance and fl > self.safe_distance and fr > self.safe_distance:
            cmd.linear.x = self.speed
            cmd.angular.z = 0.0
            self._log('✅ Clear - Moving Forward')

        # ⚠️ عائق في المقدمة فقط - دوران
        elif f < self.safe_distance and fl > self.safe_distance and fr > self.safe_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self._log('⚠️  Obstacle Front - Turning Left')

        # ⚠️ عائق يسار المقدمة - انعطف يمين
        elif f > self.safe_distance and fl < self.safe_distance:
            cmd.linear.x = self.speed * 0.5
            cmd.angular.z = -0.4
            self._log('⚠️  Obstacle Front-Left - Turning Right')

        # ⚠️ عائق يمين المقدمة - انعطف يسار
        elif f > self.safe_distance and fr < self.safe_distance:
            cmd.linear.x = self.speed * 0.5
            cmd.angular.z = 0.4
            self._log('⚠️  Obstacle Front-Right - Turning Left')

        # 🔴 خطر - عائق قريب جداً في المقدمة
        elif f < self.danger_distance:
            cmd.linear.x = -self.speed * 0.5
            cmd.angular.z = 0.5
            self._log('🔴 DANGER - Backing Up!')

        # ⚠️ عائق يسار ويمين المقدمة - تراجع
        elif fl < self.safe_distance and fr < self.safe_distance:
            if l > r:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.5
                self._log('⚠️  Both sides blocked - Turning Right')
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
                self._log('⚠️  Both sides blocked - Turning Left')

        else:
            cmd.linear.x = self.speed * 0.3
            cmd.angular.z = 0.3
            self._log('🔄 Navigating...')

        self.cmd_pub.publish(cmd)

    def _log(self, msg):
        print(f'\r  {msg} | '
              f'F:{self.regions["front"]:.2f} '
              f'FL:{self.regions["front_left"]:.2f} '
              f'FR:{self.regions["front_right"]:.2f} '
              f'L:{self.regions["left"]:.2f} '
              f'R:{self.regions["right"]:.2f}    ',
              end='', flush=True)

def main():
    rclpy.init()
    node = ObstacleAvoidance()

    print("""
╔══════════════════════════════════════════╗
║       OBSTACLE AVOIDANCE SYSTEM         ║
╠══════════════════════════════════════════╣
║  Press ENTER : Toggle ON/OFF            ║
║  Press q     : Quit                     ║
╚══════════════════════════════════════════╝
    """)

    import threading, sys, tty, termios

    settings = termios.tcgetattr(sys.stdin)

    def get_key():
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            key = get_key()
            if key == 'q':
                print('\n\nStopping...')
                node.active = False
                node.cmd_pub.publish(Twist())
                break
            elif key == '\r' or key == '\n' or key == ' ':
                node.active = not node.active
                status = '🟢 ON' if node.active else '🔴 OFF'
                print(f'\n  Obstacle Avoidance: {status}')
    except Exception as e:
        print(f'\nError: {e}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.cmd_pub.publish(Twist())
        rclpy.shutdown()

if __name__ == '__main__':
    main()
