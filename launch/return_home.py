#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class ReturnHome(Node):
    def __init__(self):
        super().__init__('return_home')
        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/drone/odom',
                                            self.odom_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.returning = False
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('اضغط ENTER للعودة للمركز...')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def control_loop(self):
        if not self.returning:
            return

        cmd = Twist()
        target_x, target_y, target_z = 0.0, 0.0, 1.0
        tolerance = 0.15

        ex = target_x - self.x
        ey = target_y - self.y
        ez = target_z - self.z

        dist = math.sqrt(ex**2 + ey**2 + ez**2)

        if dist < tolerance:
            self.get_logger().info('✅ وصل للمركز!')
            self.returning = False
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
        else:
            kp = 0.5
            cmd.linear.x = kp * ex
            cmd.linear.y = kp * ey
            cmd.linear.z = kp * ez
            # تحديد السرعة القصوى
            cmd.linear.x = max(-0.5, min(0.5, cmd.linear.x))
            cmd.linear.y = max(-0.5, min(0.5, cmd.linear.y))
            cmd.linear.z = max(-0.5, min(0.5, cmd.linear.z))

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = ReturnHome()

    import threading
    def wait_input():
        while rclpy.ok():
            input()
            node.returning = True
            node.get_logger().info('🏠 العودة للمركز...')

    t = threading.Thread(target=wait_input, daemon=True)
    t.start()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
