#!/usr/bin/env python3
"""Test automatique de l'odométrie en ligne droite (simulation Gazebo).

Fait avancer le robot en ligne droite, puis compare la distance vue par
l'odométrie (/rovexbot_controller/odom) à la distance réellement parcourue
dans Gazebo (ign model -p).

Usage (simulation lancée, clavier/manette au repos, espace libre devant) :
    python3 test_translation.py             # 2 m à 0.3 m/s
    python3 test_translation.py 1.0 0.2     # distance (m) et vitesse (m/s)
    python3 test_translation.py 1.0 -0.2    # vitesse négative : marche arrière
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


def parse_gazebo_pose(text):
    """Extrait (x, y, yaw) de la sortie de `ign model -m <modele> -p`."""
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
    xyz, rpy = triples[-2], triples[-1]
    return xyz[0], xyz[1], rpy[2]


def gazebo_pose():
    out = subprocess.run(
        ["ign", "model", "-m", MODEL, "-p"],
        capture_output=True, text=True, timeout=15,
    )
    return parse_gazebo_pose(out.stdout)


class TranslationTest(Node):
    def __init__(self):
        super().__init__(
            "test_translation",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.x = None
        self.y = None

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def send(self, vx):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = vx
        self.pub.publish(msg)

    def spin_for(self, seconds, vx=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if vx is not None:
                self.send(vx)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    target = abs(float(sys.argv[1])) if len(sys.argv) > 1 else 2.0
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    if speed == 0.0:
        print("ERREUR : la vitesse ne peut pas être nulle.")
        return 1

    rclpy.init()
    node = TranslationTest()
    try:
        # Attente de l'odométrie
        t0 = time.monotonic()
        while node.x is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - t0 > 10.0:
                print("ERREUR : pas de message sur " + ODOM_TOPIC
                      + " (simulation lancée ?)")
                return 1

        node.spin_for(1.0, vx=0.0)          # robot à l'arrêt
        ox0, oy0 = node.x, node.y
        gx0, gy0, gyaw0 = gazebo_pose()

        sens = "avant" if speed > 0 else "arrière"
        print(f"Ligne droite de {target:.2f} m à {abs(speed):.2f} m/s "
              f"(marche {sens}) ...")
        t0 = time.monotonic()
        limit = target / abs(speed) * 3.0 + 10.0
        while math.hypot(node.x - ox0, node.y - oy0) < target:
            node.send(speed)
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - t0 > limit:
                print("ERREUR : le robot n'avance pas (délai dépassé, "
                      "obstacle ?).")
                node.spin_for(1.0, vx=0.0)
                return 1

        node.spin_for(3.0, vx=0.0)          # arrêt et stabilisation
        d_odom = math.hypot(node.x - ox0, node.y - oy0)
        gx1, gy1, gyaw1 = gazebo_pose()
        d_gz = math.hypot(gx1 - gx0, gy1 - gy0)
        dyaw = (gyaw1 - gyaw0 + math.pi) % (2.0 * math.pi) - math.pi

        ecart = (d_odom / d_gz - 1.0) * 100.0 if d_gz > 1e-6 else float("nan")
        print("")
        print(f"  Odométrie : {d_odom:8.4f} m")
        print(f"  Gazebo    : {d_gz:8.4f} m")
        print(f"  Ecart     : {ecart:+8.2f} %")
        print(f"  Déviation de cap (Gazebo) : {math.degrees(dyaw):+.2f} deg")
        print("  -> " + ("OK (moins de 2 %)" if abs(ecart) < 2.0
                        else "odométrie à corriger"))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
