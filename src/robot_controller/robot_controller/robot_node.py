import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/robot_cmd',
            self.command_callback,
            10
        )

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_publisher = self.create_publisher(String, '/robot_feedback', 10)

        # Keep-alive timer at 10Hz — republishes last command continuously
        self.last_msg = Twist()
        self.timer = self.create_timer(0.1, self.keep_alive)

        self.get_logger().info('Robot node started, waiting for commands...')

    def keep_alive(self):
        # Continuously send last received command to Gazebo at 10Hz
        self.vel_publisher.publish(self.last_msg)

    def command_callback(self, msg):
        # Store and immediately forward
        self.last_msg = msg
        self.vel_publisher.publish(msg)

        # Build feedback
        linear  = round(msg.linear.x, 2)
        angular = round(msg.angular.z, 2)

        if msg.linear.x > 0.0 and msg.angular.z == 0.0:
            status = 'MOVING FORWARD'
        elif msg.linear.x < 0.0 and msg.angular.z == 0.0:
            status = 'MOVING BACKWARD'
        elif msg.linear.x > 0.0 and msg.angular.z > 0.0:
            status = 'CIRCULAR MOTION LEFT'
        elif msg.linear.x > 0.0 and msg.angular.z < 0.0:
            status = 'CIRCULAR MOTION RIGHT'
        elif msg.linear.x == 0.0 and msg.angular.z > 0.0:
            status = 'SPINNING LEFT'
        elif msg.linear.x == 0.0 and msg.angular.z < 0.0:
            status = 'SPINNING RIGHT'
        else:
            status = 'STOPPED'

        feedback = String()
        feedback.data = f'[{status}] linear={linear} m/s | angular={angular} rad/s'
        self.feedback_publisher.publish(feedback)
        self.get_logger().info(f'Executed: {feedback.data}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        node.vel_publisher.publish(stop_msg)
        node.get_logger().info('Robot node shutting down — robot stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()