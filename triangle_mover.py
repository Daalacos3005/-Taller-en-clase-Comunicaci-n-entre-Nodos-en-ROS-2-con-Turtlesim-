#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TriangleMover(Node):
    def __init__(self):
        super().__init__('triangle_mover')

        self.pose = None
        self.vertices = []
        self.vertex_index = 0
        self.state = 'turn'
        self.pause_start = None

        self.L = 3.0              # lado del triangulo
        self.angle_tol = 0.01     # rad
        self.dist_tol = 0.05      # unidades
        self.pause_s = 0.3

        self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(0.05, self.control_loop)

    def update_pose(self, msg: Pose):
        self.pose = msg

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def control_loop(self):
        if self.pose is None:
            return

        # Vertices del triangulo equilatero a partir de la posicion inicial
        if not self.vertices:
            x0, y0 = self.pose.x, self.pose.y
            self.vertices = [
                (x0, y0),
                (x0 + self.L, y0),
                (x0 + self.L / 2.0, y0 + self.L * math.sqrt(3) / 2.0),
            ]
            self.vertex_index = 1
            self.state = 'turn'
            return

        tx, ty = self.vertices[self.vertex_index]
        dx, dy = tx - self.pose.x, ty - self.pose.y
        angle_diff = self.normalize_angle(math.atan2(dy, dx) - self.pose.theta)
        dist = math.hypot(dx, dy)

        msg = Twist()

        if self.state == 'turn':
            if abs(angle_diff) > self.angle_tol:
                msg.angular.z = max(-2.0, min(2.0, 3.0 * angle_diff))
            else:
                self.state = 'pause'
                self.pause_start = self.get_clock().now()

        elif self.state == 'pause':
            elapsed = (self.get_clock().now() - self.pause_start).nanoseconds / 1e9
            if elapsed > self.pause_s:
                self.state = 'move'

        elif self.state == 'move':
            if dist > self.dist_tol:
                msg.linear.x = min(1.5, max(0.3, 1.5 * dist))
                msg.angular.z = 2.0 * angle_diff  # correccion fina de rumbo
            else:
                self.state = 'turn'
                self.vertex_index = (self.vertex_index + 1) % 3

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TriangleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
