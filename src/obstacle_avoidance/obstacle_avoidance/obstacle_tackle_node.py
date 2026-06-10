#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # distances (meters)
        self.emergency_distance = 0.50
        self.warning_distance = 1.00

        self.get_logger().info("Obstacle Avoidance Started")

    def clean_ranges(self, ranges, range_max):

        cleaned = []

        for r in ranges:

            if math.isinf(r):
                cleaned.append(range_max)

            elif math.isnan(r):
                cleaned.append(range_max)

            else:
                cleaned.append(r)

        return cleaned

    def scan_callback(self, msg):

        ranges = self.clean_ranges(msg.ranges, msg.range_max)

        total = len(ranges)

        # 360 beam lidar assumptions
        front = min(ranges[0:20] + ranges[-20:])

        left_front = min(ranges[20:90])
        left = min(ranges[90:180])

        right_front = min(ranges[270:340])
        right = min(ranges[180:270])

        cmd = Twist()

        # -------------------------------------------------
        # Emergency situation
        # -------------------------------------------------
        if front < self.emergency_distance:

            cmd.linear.x = 0.0

            if left_front > right_front:
                cmd.angular.z = 1.2
            else:
                cmd.angular.z = -1.2

        # -------------------------------------------------
        # Obstacle detected ahead
        # -------------------------------------------------
        elif front < self.warning_distance:

            cmd.linear.x = 0.08

            if left_front > right_front:
                cmd.angular.z = 0.8
            else:
                cmd.angular.z = -0.8

        # -------------------------------------------------
        # Free path
        # -------------------------------------------------
        else:

            cmd.linear.x = 0.25

            # keep away from side walls

            if left < 0.50:
                cmd.angular.z = -0.3

            elif right < 0.50:
                cmd.angular.z = 0.3

            else:
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()