import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math


class StatusMonitorNode(Node):
    def __init__(self):
        super().__init__('status_monitor_node')

        # ── Subscriber: receives status from avoidance node ──
        self.status_subscriber = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10
        )

        # ── Subscriber: receives odometry for position tracking ──
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_status   = 'Waiting...'
        self.robot_x          = 0.0
        self.robot_y          = 0.0
        self.robot_yaw        = 0.0
        self.distance_traveled = 0.0
        self.prev_x           = None
        self.prev_y           = None

        # Print dashboard every 1 second
        self.dashboard_timer = self.create_timer(1.0, self.print_dashboard)

        self.get_logger().info('Status Monitor Node started!')

    def status_callback(self, msg):
        self.current_status = msg.data

    def odom_callback(self, msg):
        self.robot_x = round(msg.pose.pose.position.x, 2)
        self.robot_y = round(msg.pose.pose.position.y, 2)

        # Calculate yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = round(math.degrees(math.atan2(siny, cosy)), 1)

        # Accumulate distance traveled
        if self.prev_x is not None and self.prev_y is not None:
            dx = self.robot_x - self.prev_x
            dy = self.robot_y - self.prev_y
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)

        self.prev_x = self.robot_x
        self.prev_y = self.robot_y

    def print_dashboard(self):
        print('\n' + '='*55)
        print('         ROBOT STATUS DASHBOARD                ')
        print('='*55)
        print(f'  Status   : {self.current_status[:50]}')
        print(f'  Position : X={self.robot_x}m  Y={self.robot_y}m')
        print(f'  Heading  : {self.robot_yaw}°')
        print(f'  Distance : {round(self.distance_traveled, 2)}m traveled')
        print('='*55)


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()