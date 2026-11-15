#!/usr/bin/env python3
"""Test automatique de l'odométrie en rotation (simulation Gazebo).

Fait pivoter le robot sur place, puis compare la rotation vue par
l'odométrie (/rovexbot_controller/odom) à la rotation réelle dans Gazebo
(ign model -p).

Usage (simulation lancée, clavier/manette au repos) :
    python3 test_rotation.py            # 180 deg à 0.5 rad/s
    python3 test_rotation.py 90 0.3     # angle (deg) et vitesse (rad/s)
"""
import math
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

MODEL = "rovexbot"
CMD_TOPIC = "/rovexbot_controller/cmd_vel"
ODOM_TOPIC = "/rovexbot_controller/odom"


def parse_gazebo_yaw(text):
    """Extrait le yaw (rad) de la sortie de `ign model -m <modele> -p`."""
    triples = []
    for group in re.findall(r"\[([^\[\]]*)\]", text):
        try:
            values = [float(v) for v in group.split()]
        except ValueError:
            continue
        if len(values) == 3:
            triples.append(values)
    if len(triples) < 2:
        raise RuntimeError("Pose Gazebo introuvable dans :\n" + text)
    return triples[-1][2]


def gazebo_yaw():
    out = subprocess.run(
        ["ign", "model", "-m", MODEL, "-p"],
        capture_output=True, text=True, timeout=15,
    )
    return parse_gazebo_yaw(out.stdout)


class RotationTest(Node):
    def __init__(self):
        super().__init__(
            "test_rotation",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.last_yaw = None
        self.total = 0.0  # yaw odométrie cumulé (sans saut à +/-180 deg)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.last_yaw is not None:
            d = yaw - self.last_yaw
            d = (d + math.pi) % (2.0 * math.pi) - math.pi
            self.total += d
        self.last_yaw = yaw

    def send(self, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.angular.z = wz
        self.pub.publish(msg)

    def spin_for(self, seconds, wz=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if wz is not None:
                self.send(wz)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    angle_deg = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    target = math.radians(abs(angle_deg))

    rclpy.init()
    node = RotationTest()
    try:
        # Attente de l'odométrie
        t0 = time.monotonic()
        while node.last_yaw is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - t0 > 10.0:
                print("ERREUR : pas de message sur " + ODOM_TOPIC
                      + " (simulation lancée ?)")
                return 1

        node.spin_for(1.0, wz=0.0)          # robot à l'arrêt
        odom_start = node.total
        gz_start = gazebo_yaw()

        print(f"Rotation de {angle_deg:.0f} deg à {speed:.2f} rad/s ...")
        t0 = time.monotonic()
        limit = target / speed * 3.0 + 10.0
        while abs(node.total - odom_start) < target:
            node.send(speed)
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - t0 > limit:
                print("ERREUR : le robot ne tourne pas (délai dépassé).")
                node.spin_for(1.0, wz=0.0)
                return 1

        node.spin_for(3.0, wz=0.0)          # arrêt et stabilisation
        d_odom = node.total - odom_start
        d_gz = gazebo_yaw() - gz_start
        # Même nombre de tours que l'odométrie
        d_gz += 2.0 * math.pi * round((d_odom - d_gz) / (2.0 * math.pi))

        ecart = (d_odom / d_gz - 1.0) * 100.0 if abs(d_gz) > 1e-6 else float("nan")
        print("")
        print(f"  Odométrie : {math.degrees(d_odom):8.2f} deg")
        print(f"  Gazebo    : {math.degrees(d_gz):8.2f} deg")
        print(f"  Ecart     : {ecart:+8.2f} %")
        print("  -> " + ("OK (moins de 2 %)" if abs(ecart) < 2.0
                        else "odométrie à corriger"))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
