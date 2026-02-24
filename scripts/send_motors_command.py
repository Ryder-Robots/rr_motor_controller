#!/usr/bin/env python3
"""Send geometry_msgs/msg/Twist commands to /cmd_vel to test the motor ECU.

Maneuvers publish continuously at PUBLISH_HZ for the calculated duration, then
send a zero Twist to stop the robot. This ensures commands do not expire before
the maneuver completes (the ECU discards commands older than ttl_ns).

Arc maneuvers (11, 12) derive angular velocity as:
    omega = theta * linear_x / arc_length
so the robot turns exactly 45 degrees while travelling 300 mm along the arc.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

PUBLISH_HZ = 10  # republish rate (Hz) while a maneuver is running

# ---------------------------------------------------------------------------
# Maneuver table: (description, linear_x m/s, angular_z rad/s, duration_s)
# ---------------------------------------------------------------------------
_V1, _V2, _V3 = 0.70, 0.86, 0.95
_DIST = 0.3        # 300 mm in metres
_ROT_W = 0.75      # rotation angular velocity (rad/s)
_ARC_THETA = math.pi / 4          # 45 degrees in radians
_ARC_W = _ARC_THETA * _V2 / _DIST  # omega for 300 mm / 45 deg arc at _V2

MANEUVERS: dict[int, tuple[str, float, float, float]] = {
    1:  ("Move 300 mm forward  @ 0.70 m/s",  +_V1,   0.0,          _DIST / _V1),
    2:  ("Move 300 mm backward @ 0.70 m/s",  -_V1,   0.0,          _DIST / _V1),
    3:  ("Move 300 mm forward  @ 0.86 m/s",  +_V2,   0.0,          _DIST / _V2),
    4:  ("Move 300 mm backward @ 0.86 m/s",  -_V2,   0.0,          _DIST / _V2),
    5:  ("Move 300 mm forward  @ 0.95 m/s",  +_V3,   0.0,          _DIST / _V3),
    6:  ("Move 300 mm backward @ 0.95 m/s",  -_V3,   0.0,          _DIST / _V3),
    7:  ("Rotate CW  45°  @ 0.75 rad/s",      0.0,  -_ROT_W,       _ARC_THETA / _ROT_W),
    8:  ("Rotate CCW 45°  @ 0.75 rad/s",      0.0,  +_ROT_W,       _ARC_THETA / _ROT_W),
    9:  ("Rotate CW  360° @ 0.75 rad/s",      0.0,  -_ROT_W,       2.0 * math.pi / _ROT_W),
    10: ("Rotate CCW 360° @ 0.75 rad/s",      0.0,  +_ROT_W,       2.0 * math.pi / _ROT_W),
    11: ("Arc CW  300 mm / 45° @ 0.86 m/s",  +_V2,  -_ARC_W,       _DIST / _V2),
    12: ("Arc CCW 300 mm / 45° @ 0.86 m/s",  +_V2,  +_ARC_W,       _DIST / _V2),
}


def _run_maneuver(node: Node, publisher, linear_x: float, angular_z: float, duration: float) -> None:
    move = Twist()
    move.linear.x = linear_x
    move.angular.z = angular_z

    stop = Twist()  # all-zero halts the motors

    deadline = time.monotonic() + duration
    period = 1.0 / PUBLISH_HZ

    while time.monotonic() < deadline:
        publisher.publish(move)
        rclpy.spin_once(node, timeout_sec=period)

    publisher.publish(stop)
    rclpy.spin_once(node, timeout_sec=0.1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Send Twist commands to /cmd_vel to test the motor ECU.')
    parser.add_argument(
        '--maneuver', type=int, choices=range(1, 13), metavar='N',
        help='Maneuver number 1–12 (omit to list available maneuvers and exit)')
    args = parser.parse_args()

    if args.maneuver is None:
        print('Available maneuvers:')
        for n, (desc, lx, az, t) in MANEUVERS.items():
            print(f'  {n:2d}.  {desc}')
        return

    desc, linear_x, angular_z, duration = MANEUVERS[args.maneuver]

    rclpy.init()
    node = Node('send_cmd_vel')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    node.get_logger().info('Waiting for subscriber on /cmd_vel...')
    while publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.get_logger().info(
        f'Running maneuver {args.maneuver}: {desc}  ({duration:.3f} s)')
    _run_maneuver(node, publisher, linear_x, angular_z, duration)
    node.get_logger().info('Maneuver complete.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
