import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class RobotNode(Node):
    def __init__(self):
        super().__init__('manual_robot_node')

        # Subscriber: receives commands from controller_node
        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/robot_cmd',
            self.command_callback,
            10
        )

        # Publisher: drives the robot in Gazebo
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher: sends feedback back to controller_node
        self.feedback_publisher = self.create_publisher(String, '/robot_feedback', 10)

        # Keep-alive timer — republishes last command at 10Hz
        # so Gazebo keeps responding continuously
        self.last_msg = Twist()
        self.timer = self.create_timer(0.1, self.keep_alive)

        self.get_logger().info('Robot node started, waiting for commands...')

    def keep_alive(self):
        self.vel_publisher.publish(self.last_msg)

    def command_callback(self, msg):
        # Store latest command for keep-alive republishing
        self.last_msg = msg

        # Forward immediately to Gazebo
        self.vel_publisher.publish(msg)

        # Determine status from received values
        linear  = round(msg.linear.x, 2)
        angular = round(msg.angular.z, 2)

        if msg.linear.x > 0.0 and msg.angular.z == 0.0:
            status = 'MOVING FORWARD'
        elif msg.linear.x < 0.0 and msg.angular.z == 0.0:
            status = 'MOVING BACKWARD'
        elif msg.linear.x == 0.0 and msg.angular.z > 0.0:
            status = 'TURNING LEFT'
        elif msg.linear.x == 0.0 and msg.angular.z < 0.0:
            status = 'TURNING RIGHT'
        elif msg.linear.x == 0.0 and msg.angular.z == 0.0:
            status = 'STOPPED'
        else:
            status = 'MOVING'

        # Build and publish feedback
        feedback = String()
        feedback.data = (
            f'[{status}] linear={linear} m/s | angular={angular} rad/s'
        )
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