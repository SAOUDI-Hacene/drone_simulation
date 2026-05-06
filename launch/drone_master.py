#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, tty, termios, threading, math

MSG = """
╔══════════════════════════════════════════════════╗
║            DRONE MASTER CONTROL                 ║
╠══════════════════════════════════════════════════╣
║  t : Up        b : Down                         ║
║  i : Forward   , : Backward                     ║
║  j : Left      l : Right                        ║
║  u : Rotate L  o : Rotate R                     ║
║  k : STOP                                       ║
╠══════════════════════════════════════════════════╣
║  w : Speed Up  x : Speed Down                   ║
╠══════════════════════════════════════════════════╣
║  a : Safety Shield ON/OFF                       ║
║  H : Return Home                                ║
║  q : Quit                                       ║
╚══════════════════════════════════════════════════╝
"""

# ── Limites des murs ──────────────────────────────
WALL_MAX  = 9.0
WALL_SLOW = 8.0
Z_MAX     = 4.5

# ── Obstacles : position (x, y) + rayon de sécurité ──
# Chaque obstacle = (x, y, rayon_stop, rayon_slow)
OBSTACLES = [
    ( 4.0,  4.0, 1.8, 2.8),   # box_1   rouge
    (-4.0,  4.0, 1.4, 2.4),   # box_2   orange
    ( 4.0, -4.0, 1.6, 2.6),   # box_3   violet
    (-4.0, -4.0, 1.6, 2.6),   # box_4   jaune
    ( 0.0,  7.0, 1.6, 2.6),   # box_5   bleu-violet
    ( 0.0, -7.0, 1.6, 2.6),   # box_6   bleu ciel
    ( 6.0,  0.0, 1.4, 2.4),   # cyl_1   bleu
    (-6.0,  0.0, 1.4, 2.4),   # cyl_2   vert-bleu
    ( 7.0, -6.0, 1.2, 2.2),   # cyl_3
    (-7.0,  6.0, 1.2, 2.2),   # cyl_4
    ( 5.0,  5.0, 1.4, 2.4),   # sphere_1
    (-5.0, -5.0, 1.3, 2.3),   # sphere_2
    (-5.0,  5.0, 1.5, 2.5),   # sphere_3
    ( 5.0, -5.0, 1.3, 2.3),   # sphere_4
]


class DroneMaster(Node):
    def __init__(self):
        super().__init__('drone_master')
        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.create_subscription(Odometry, '/drone/odom', self.odom_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.speed  = 0.5
        self.turn   = 0.5
        self.safety = False
        self.homing = False
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_wz = 0.0
        self.create_timer(0.1, self.loop)

    def odom_cb(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.z = float(msg.pose.pose.position.z)

    def loop(self):
        if self.homing:
            self._do_homing()
        else:
            self._send()

    # ── Calcul distance au plus proche obstacle ───────
    def _nearest_obstacle(self):
        """Retourne (distance, ox, oy, r_stop, r_slow) de l'obstacle le plus proche"""
        best = None
        best_dist = 999.0
        for (ox, oy, r_stop, r_slow) in OBSTACLES:
            d = math.sqrt((self.x - ox)**2 + (self.y - oy)**2)
            if d < best_dist:
                best_dist = d
                best = (d, ox, oy, r_stop, r_slow)
        return best

    def _check_obstacles(self, vx, vy):
        """
        Même logique que les murs :
        - dist < r_stop  → STOP dans la direction de l'obstacle
        - dist < r_slow  → ralentissement progressif
        """
        result = self._nearest_obstacle()
        if result is None:
            return float(vx), float(vy), '✅ Clear'

        dist, ox, oy, r_stop, r_slow = result

        if dist > r_slow:
            return float(vx), float(vy), '✅ Clear'

        # Direction vers l'obstacle (vecteur normalisé)
        dx = ox - self.x   # positif = obstacle à droite/avant
        dy = oy - self.y
        norm = math.sqrt(dx**2 + dy**2)
        if norm < 0.001:
            return float(vx), float(vy), '✅ Clear'

        dx /= norm
        dy /= norm

        # Produit scalaire : est-ce qu'on se dirige vers l'obstacle ?
        # dot > 0 → on fonce vers lui
        dot_x = vx * dx + vy * dy   # composante vers obstacle en X global
        # Simplification : on bloque si on se rapproche

        if dist < r_stop:
            # STOP : bloquer le mouvement vers l'obstacle
            # Projeter la vitesse sur la direction obstacle
            proj = vx * dx + vy * dy
            if proj > 0:
                vx -= proj * dx
                vy -= proj * dy
            info = f'🛑 OBS STOP! {dist:.1f}m'

        elif dist < r_slow:
            # Ralentissement progressif
            ratio = (dist - r_stop) / (r_slow - r_stop)
            ratio = max(0.1, float(ratio))
            proj = vx * dx + vy * dy
            if proj > 0:
                vx -= proj * dx * (1.0 - ratio)
                vy -= proj * dy * (1.0 - ratio)
            info = f'⚠️  OBS {dist:.1f}m'
        else:
            info = '✅ Clear'

        return float(vx), float(vy), info

    def _check_walls(self, vx, vy, vz):
        wall_info = ''

        # X axis
        if self.x >= WALL_MAX and vx > 0:
            vx = 0.0
            wall_info = f'🧱 WALL EAST x={self.x:.1f}'
        elif self.x <= -WALL_MAX and vx < 0:
            vx = 0.0
            wall_info = f'🧱 WALL WEST x={self.x:.1f}'
        elif self.x >= WALL_SLOW and vx > 0:
            ratio = (WALL_MAX - self.x) / (WALL_MAX - WALL_SLOW)
            vx *= max(0.1, float(ratio))
            wall_info = f'⚠️  Near East wall'
        elif self.x <= -WALL_SLOW and vx < 0:
            ratio = (WALL_MAX + self.x) / (WALL_MAX - WALL_SLOW)
            vx *= max(0.1, float(ratio))
            wall_info = f'⚠️  Near West wall'

        # Y axis
        if self.y >= WALL_MAX and vy > 0:
            vy = 0.0
            wall_info = f'🧱 WALL NORTH y={self.y:.1f}'
        elif self.y <= -WALL_MAX and vy < 0:
            vy = 0.0
            wall_info = f'🧱 WALL SOUTH y={self.y:.1f}'
        elif self.y >= WALL_SLOW and vy > 0:
            ratio = (WALL_MAX - self.y) / (WALL_MAX - WALL_SLOW)
            vy *= max(0.1, float(ratio))
            wall_info = f'⚠️  Near North wall'
        elif self.y <= -WALL_SLOW and vy < 0:
            ratio = (WALL_MAX + self.y) / (WALL_MAX - WALL_SLOW)
            vy *= max(0.1, float(ratio))
            wall_info = f'⚠️  Near South wall'

        # Z axis
        if self.z >= Z_MAX and vz > 0:
            vz = 0.0
            wall_info = f'🧱 MAX HEIGHT z={self.z:.1f}'

        return float(vx), float(vy), float(vz), wall_info

    def _send(self):
        vx = float(self.cmd_vx)
        vy = float(self.cmd_vy)
        vz = float(self.cmd_vz)
        wz = float(self.cmd_wz)
        info = ''

        if self.safety:
            vx, vy, info = self._check_obstacles(vx, vy)

        vx, vy, vz, wall_info = self._check_walls(vx, vy, vz)
        if wall_info:
            info = wall_info

        out = Twist()
        out.linear.x  = float(vx)
        out.linear.y  = float(vy)
        out.linear.z  = float(vz)
        out.angular.z = float(wz)
        self.pub.publish(out)
        self._status(info)

    def _do_homing(self):
        ex = 0.0 - self.x
        ey = 0.0 - self.y
        ez = 0.0 - self.z
        dist = math.sqrt(ex**2 + ey**2 + ez**2)
        if dist < 0.2:
            self.homing = False
            self.pub.publish(Twist())
            self._status('✅ Landed at Home!')
            return
        kp = 0.4
        vx = max(-0.4, min(0.4, kp * ex))
        vy = max(-0.4, min(0.4, kp * ey))
        vz = max(-0.4, min(0.4, kp * ez))
        if self.x >= WALL_MAX  and vx > 0: vx = 0.0
        if self.x <= -WALL_MAX and vx < 0: vx = 0.0
        if self.y >= WALL_MAX  and vy > 0: vy = 0.0
        if self.y <= -WALL_MAX and vy < 0: vy = 0.0
        out = Twist()
        out.linear.x = float(vx)
        out.linear.y = float(vy)
        out.linear.z = float(vz)
        self.pub.publish(out)
        self._status(f'🏠 Homing... dist={dist:.2f}m')

    def set_cmd(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        self.cmd_vx = float(vx)
        self.cmd_vy = float(vy)
        self.cmd_vz = float(vz)
        self.cmd_wz = float(wz)

    def stop_all(self):
        self.set_cmd()
        self.homing = False
        self.pub.publish(Twist())

    def _status(self, info=''):
        sh  = '🛡️ ON ' if self.safety else '⬜ OFF'
        hm  = '🏠' if self.homing else '  '
        pos = f'x:{self.x:.1f} y:{self.y:.1f} z:{self.z:.1f}'
        result = self._nearest_obstacle()
        nd = result[0] if result else 999.0
        print(f'\r  [SHIELD:{sh}] {hm} [Spd:{self.speed:.1f}] [{pos}] Nearest:{nd:.1f}m | {info}        ',
              end='', flush=True)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node     = DroneMaster()
    settings = termios.tcgetattr(sys.stdin)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(MSG)
    print('  ✅ Wall + Obstacle protection by GPS coordinates')
    print('  ✅ No Lidar needed — works always!\n')

    try:
        while True:
            key = get_key(settings)
            if key == 'q':
                print('\n  Shutting down...')
                node.stop_all()
                break
            elif key == 'w':
                node.speed = min(2.0, round(node.speed + 0.1, 2))
                node.turn  = min(2.0, round(node.turn  + 0.1, 2))
                print(f'\n  ⬆  Speed: {node.speed:.1f} m/s')
            elif key == 'x':
                node.speed = max(0.1, round(node.speed - 0.1, 2))
                node.turn  = max(0.1, round(node.turn  - 0.1, 2))
                print(f'\n  ⬇  Speed: {node.speed:.1f} m/s')
            elif key == 'a':
                node.safety = not node.safety
                st = '🛡️  ON' if node.safety else '🔴 OFF'
                print(f'\n  Safety Shield: {st}')
            elif key == 'H':
                node.homing = True
                node.set_cmd()
                print('\n  🏠 Returning to Home...')
            elif key == 'k':
                node.stop_all()
                print('\n  ⏹  STOPPED')
            elif not node.homing:
                s = node.speed
                t = node.turn
                if   key == 't': node.set_cmd(vz= s)
                elif key == 'b': node.set_cmd(vz=-s)
                elif key == 'i': node.set_cmd(vx= s)
                elif key == ',': node.set_cmd(vx=-s)
                elif key == 'j': node.set_cmd(vy= s)
                elif key == 'l': node.set_cmd(vy=-s)
                elif key == 'u': node.set_cmd(wz= t)
                elif key == 'o': node.set_cmd(wz=-t)
    except Exception as e:
        print(f'\n  Error: {e}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop_all()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
