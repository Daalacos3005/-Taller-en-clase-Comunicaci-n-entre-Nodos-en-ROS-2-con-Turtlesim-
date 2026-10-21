#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

    def pose_callback(self, msg: Pose):
        self.get_logger().info(
            f'x: {msg.x:.2f} | y: {msg.y:.2f} | '
            f'theta: {msg.theta:.2f} rad ({math.degrees(msg.theta):.1f} deg)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
