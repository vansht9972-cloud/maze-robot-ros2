import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading


class ControllerNode(Node):
    def __init__(self):
        super().__init__('manual_controller_node')

        self.cmd_publisher = self.create_publisher(Twist, '/robot_cmd', 10)

        self.feedback_subscriber = self.create_subscription(
            String,
            '/robot_feedback',
            self.feedback_callback,
            10
        )

        self.get_logger().info('Controller node started')
        self.print_menu()

        # Run user input in a separate thread so it doesn't block ROS2 spinning
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()

    def print_menu(self):
        print('\n=============================')
        print('   ROBOT CONTROLLER MENU    ')
        print('=============================')
        print('  W — Move Forward          ')
        print('  S — Move Backward         ')
        print('  A — Turn Left             ')
        print('  D — Turn Right            ')
        print('  X — Stop                  ')
        print('  Q — Quit                  ')
        print('=============================\n')

    def user_input_loop(self):
        while rclpy.ok():
            try:
                key = input('Enter command: ').strip().lower()
                msg = Twist()

                if key == 'w':
                    msg.linear.x = 0.5
                    msg.angular.z = 0.0
                    print('Command: MOVE FORWARD')

                elif key == 's':
                    msg.linear.x = -0.5
                    msg.angular.z = 0.0
                    print('Command: MOVE BACKWARD')

                elif key == 'a':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.5
                    print('Command: TURN LEFT')

                elif key == 'd':
                    msg.linear.x = 0.0
                    msg.angular.z = -0.5
                    print('Command: TURN RIGHT')

                elif key == 'x':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    print('Command: STOP')

                elif key == 'q':
                    print('Quitting controller...')
                    # Send stop before quitting
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.cmd_publisher.publish(msg)
                    rclpy.shutdown()
                    break

                else:
                    print('Invalid key! Use W/A/S/D/X/Q')
                    continue

                self.cmd_publisher.publish(msg)

            except EOFError:
                break

    def feedback_callback(self, msg):
        print(f'[FEEDBACK] {msg.data}')


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
        print('Robot stopped. Controller shut down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()