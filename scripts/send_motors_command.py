#!/usr/bin/env python3
"""Send a rr_interfaces/msg/Motors message to the /motors_command topic."""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rr_interfaces.msg import Motor, Motors


def main():
    parser = argparse.ArgumentParser(description='Send a Motors command to motor index 0.')
    parser.add_argument('--direction', type=str, default='forward',
                        choices=['forward', 'reverse'],
                        help='Motor direction (default: forward)')
    parser.add_argument('--velocity', type=float, default=0,
                        help='PWM duty cycle value (default: 0)')
    args = parser.parse_args()

    rclpy.init()
    node = Node('motors_command_sender')

    # Match the SensorDataQoS used by the subscriber
    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )

    publisher = node.create_publisher(Motors, '/motors_command', qos)

    msg = Motors()
    motor = Motor()
    motor.direction = args.direction == 'forward'
    motor.velocity = args.velocity

    # Motor at index 0
    msg.motors = [motor]

    # Wait for subscriber to be discovered
    node.get_logger().info('Waiting for subscriber...')
    while publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.5)

    # Publish a few times to ensure delivery over BEST_EFFORT QoS.
    for _ in range(3):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(
        f'Published Motors message: direction={args.direction}, velocity={args.velocity}'
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
