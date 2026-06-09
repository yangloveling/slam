#!/usr/bin/env python3
"""Publish a simple target point from the first occupied costmap cell."""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point


class DynamicTarget(Node):
    """Convert local costmap obstacle cells into a target point topic."""

    def __init__(self):
        """Create publishers and subscriptions."""
        super().__init__('dynamic_target')

        self.pub = self.create_publisher(Point, '/target_pos', 10)

        self.sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.callback,
            10
        )

        self.get_logger().info('dynamic_target started.')
        self.get_logger().info('Subscribe: /local_costmap/costmap')
        self.get_logger().info('Publish: /target_pos')

    def callback(self, msg: OccupancyGrid):
        """Publish the first occupied costmap cell as a point target."""
        obstacle_points = []

        width = msg.info.width
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        for idx, value in enumerate(msg.data):
            if value > 0:
                x = (idx % width) * resolution + origin_x
                y = (idx // width) * resolution + origin_y
                obstacle_points.append((x, y))

        if obstacle_points:
            target_x, target_y = obstacle_points[0]

            msg_out = Point()
            msg_out.x = target_x
            msg_out.y = target_y
            msg_out.z = 0.0

            self.pub.publish(msg_out)


def main(args=None):
    """Run the dynamic target node."""
    rclpy.init(args=args)

    node = DynamicTarget()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
