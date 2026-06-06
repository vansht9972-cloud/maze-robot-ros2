import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.cmd_publisher = self.create_publisher(Twist, '/robot_cmd', 10)

        self.feedback_subscriber = self.create_subscription(
            String,
            '/robot_feedback',
            self.feedback_callback,
            10
        )

        # Publish at 10Hz for smooth Gazebo motion
        self.timer = self.create_timer(0.1, self.publish_command)
        self.get_logger().info('Controller node started')

    def publish_command(self):
        msg = Twist()
        msg.linear.x = 0.3    # forward speed (m/s)
        msg.angular.z = 0.5   # turning rate (rad/s) — circle radius = 0.3/0.5 = 0.6m
        self.cmd_publisher.publish(msg)
        self.get_logger().info(
            f'Publishing: linear={msg.linear.x} angular={msg.angular.z}'
        )

    def feedback_callback(self, msg):
        self.get_logger().info(f'Feedback: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        node.cmd_publisher.publish(stop_msg)
        node.get_logger().info('Robot stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()