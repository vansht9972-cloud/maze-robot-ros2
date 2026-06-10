import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.vel_publisher    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)

        # ── Parameters ──
        self.forward_speed  = 0.2
        self.turn_speed     = 0.7

        self.front_stop     = 0.7    # stop if front < this
        self.diag_stop      = 0.55   # stop if diagonal < this
        self.side_warn      = 0.45   # steer if side < this
        self.side_critical  = 0.28   # hard turn if side < this
        self.slow_dist      = 1.1    # slow down if front < this

        # ── Escape state ──
        self.state             = 'forward'
        self.escape_counter    = 0
        self.escape_duration   = 35
        self.escape_turn_dir   = -1.0   # -1=right, +1=left (chosen smartly)

        # ── Oscillation detection ──
        self.last_states       = []     # rolling history of last 10 states
        self.history_size      = 10

        # ── Keep-alive at 10Hz ──
        self.last_cmd = Twist()
        self.timer    = self.create_timer(0.1, self.keep_alive)

        self.get_logger().info('Obstacle Avoidance Node started!')

    def keep_alive(self):
        self.vel_publisher.publish(self.last_cmd)

    def get_min(self, ranges, start, end):
        total = len(ranges)
        if start < end:
            zone = ranges[start:end]
        else:
            zone = ranges[start:total] + ranges[0:end]
        valid = [
            r for r in zone
            if not math.isnan(r) and not math.isinf(r) and r > 0.05
        ]
        return min(valid) if valid else 10.0

    def is_oscillating(self):
        """Detect if robot is stuck oscillating between two states."""
        if len(self.last_states) < self.history_size:
            return False
        recent = self.last_states[-self.history_size:]
        # Check if alternating between critical_left and critical_right
        critical_states = [
            s for s in recent
            if 'CRITICAL' in s or 'SIDE' in s
        ]
        return len(critical_states) >= 8

    def update_history(self, state_name):
        self.last_states.append(state_name)
        if len(self.last_states) > 20:
            self.last_states.pop(0)

    def choose_escape_direction(self, min_left, min_right, min_back_left, min_back_right):
        """Choose escape turn direction based on where most space is."""
        left_space  = min_left  + min_back_left
        right_space = min_right + min_back_right
        if left_space >= right_space:
            return 1.0   # turn left while backing
        else:
            return -1.0  # turn right while backing

    def scan_callback(self, scan_msg):
        ranges = list(scan_msg.ranges)

        # ════════════════════════════════════
        # 7-ZONE LIDAR
        # ════════════════════════════════════
        #
        #   FRONT:         355°→5°    (pure front, very narrow)
        #   FRONT-LEFT:    5°→75°     (diagonal left)
        #   FRONT-RIGHT:   285°→355°  (diagonal right)
        #   LEFT:          75°→160°   (pure left)
        #   RIGHT:         200°→285°  (pure right)
        #   BACK-LEFT:     160°→200°  (behind left — for escape direction)
        #   BACK:          170°→190°  (pure back — is there space to back up?)

        min_front       = self.get_min(ranges, 355, 360)
        min_front       = min(min_front, self.get_min(ranges, 0, 5))

        min_front_left  = self.get_min(ranges, 5,   75)
        min_front_right = self.get_min(ranges, 285, 355)

        min_left        = self.get_min(ranges, 75,  160)
        min_right       = self.get_min(ranges, 200, 285)

        min_back_left   = self.get_min(ranges, 160, 185)
        min_back_right  = self.get_min(ranges, 175, 200)
        min_back        = self.get_min(ranges, 170, 190)

        msg    = Twist()
        status = ''

        # ════════════════════════════════════
        # STATE MACHINE
        # ════════════════════════════════════

        # ── ESCAPE STATE ──
        if self.state == 'escape':
            if self.escape_counter > 0:

                if min_back < 0.3:
                    # Can't back up — wall behind! Turn in place instead
                    msg.linear.x  = 0.0
                    msg.angular.z = self.turn_speed * self.escape_turn_dir
                    status = f'ESCAPE — spinning | back={min_back:.2f} counter={self.escape_counter}'
                else:
                    # Back up while turning toward open space
                    msg.linear.x  = -0.15
                    msg.angular.z = self.turn_speed * 0.5 * self.escape_turn_dir
                    status = f'ESCAPE — backing | back={min_back:.2f} counter={self.escape_counter}'

                self.escape_counter -= 1

            else:
                self.state = 'forward'
                self.last_states.clear()
                status = 'ESCAPE COMPLETE — resuming forward'

        # ── OSCILLATION DETECTED: force different escape ──
        elif self.is_oscillating():
            # Pick escape direction toward most open space
            self.escape_turn_dir = self.choose_escape_direction(
                min_left, min_right, min_back_left, min_back_right
            )
            self.state          = 'escape'
            self.escape_counter = self.escape_duration
            self.last_states.clear()
            msg.linear.x        = -0.15
            msg.angular.z       = self.turn_speed * 0.5 * self.escape_turn_dir
            status = (
                f'OSCILLATION DETECTED — ESCAPE '
                f'{"LEFT" if self.escape_turn_dir > 0 else "RIGHT"} | '
                f'back={min_back:.2f}'
            )

        # ── COMPLETELY TRAPPED ──
        elif (min_front       < self.front_stop and
              min_front_left  < self.diag_stop and
              min_front_right < self.diag_stop):
            self.escape_turn_dir = self.choose_escape_direction(
                min_left, min_right, min_back_left, min_back_right
            )
            self.state          = 'escape'
            self.escape_counter = self.escape_duration
            msg.linear.x        = -0.15
            msg.angular.z       = self.turn_speed * 0.5 * self.escape_turn_dir
            status = (
                f'TRAPPED — ESCAPE {"LEFT" if self.escape_turn_dir > 0 else "RIGHT"} | '
                f'F:{min_front:.2f} FL:{min_front_left:.2f} FR:{min_front_right:.2f}'
            )
            self.update_history('TRAPPED')

        # ── CRITICAL SIDE: stop all forward motion ──
        elif min_right < self.side_critical and min_left < self.side_critical:
            # Both sides critical — choose turn toward most space ahead
            if min_front_left >= min_front_right:
                msg.linear.x  = 0.0
                msg.angular.z = self.turn_speed
                status = f'BOTH CRITICAL — TURN LEFT | L:{min_left:.2f} R:{min_right:.2f}'
            else:
                msg.linear.x  = 0.0
                msg.angular.z = -self.turn_speed
                status = f'BOTH CRITICAL — TURN RIGHT | L:{min_left:.2f} R:{min_right:.2f}'
            self.update_history('BOTH_CRITICAL')

        elif min_right < self.side_critical:
            msg.linear.x  = 0.0
            msg.angular.z = self.turn_speed   # turn left away from right wall
            status = f'CRITICAL RIGHT ({min_right:.2f}m) — TURN LEFT HARD'
            self.update_history('CRITICAL_RIGHT')

        elif min_left < self.side_critical:
            msg.linear.x  = 0.0
            msg.angular.z = -self.turn_speed  # turn right away from left wall
            status = f'CRITICAL LEFT ({min_left:.2f}m) — TURN RIGHT HARD'
            self.update_history('CRITICAL_LEFT')

        # ── FRONT BLOCKED ──
        elif min_front < self.front_stop:
            msg.linear.x = 0.0
            left_score   = min_front_left + min_left
            right_score  = min_front_right + min_right
            if left_score >= right_score:
                msg.angular.z = self.turn_speed
                status = f'FRONT ({min_front:.2f}m) — TURN LEFT | scores L:{left_score:.1f} R:{right_score:.1f}'
            else:
                msg.angular.z = -self.turn_speed
                status = f'FRONT ({min_front:.2f}m) — TURN RIGHT | scores L:{left_score:.1f} R:{right_score:.1f}'
            self.update_history('FRONT')

        # ── DIAGONAL LEFT BLOCKED ──
        elif min_front_left < self.diag_stop:
            msg.linear.x  = self.forward_speed * 0.4
            msg.angular.z = -self.turn_speed * 0.6
            status = f'DIAG LEFT ({min_front_left:.2f}m) — STEER RIGHT'
            self.update_history('DIAG_LEFT')

        # ── DIAGONAL RIGHT BLOCKED ──
        elif min_front_right < self.diag_stop:
            msg.linear.x  = self.forward_speed * 0.4
            msg.angular.z = self.turn_speed * 0.6
            status = f'DIAG RIGHT ({min_front_right:.2f}m) — STEER LEFT'
            self.update_history('DIAG_RIGHT')

        # ── SIDE WARNING: gentle steer ──
        elif min_right < self.side_warn:
            msg.linear.x  = self.forward_speed * 0.6
            msg.angular.z = self.turn_speed * 0.4
            status = f'SIDE RIGHT ({min_right:.2f}m) — STEER LEFT'
            self.update_history('SIDE_RIGHT')

        elif min_left < self.side_warn:
            msg.linear.x  = self.forward_speed * 0.6
            msg.angular.z = -self.turn_speed * 0.4
            status = f'SIDE LEFT ({min_left:.2f}m) — STEER RIGHT'
            self.update_history('SIDE_LEFT')

        # ── SLOW DOWN ──
        elif min_front < self.slow_dist:
            msg.linear.x  = self.forward_speed * 0.5
            msg.angular.z = 0.0
            status = f'SLOWING | F:{min_front:.2f} FL:{min_front_left:.2f} FR:{min_front_right:.2f}'
            self.update_history('SLOW')

        # ── PATH CLEAR ──
        else:
            msg.linear.x  = self.forward_speed
            msg.angular.z = 0.0
            status = (
                f'PATH CLEAR | '
                f'F:{min_front:.2f} FL:{min_front_left:.2f} FR:{min_front_right:.2f} '
                f'L:{min_left:.2f} R:{min_right:.2f}'
            )
            self.update_history('CLEAR')

        # ── Publish ──
        self.last_cmd = msg
        self.vel_publisher.publish(msg)

        feedback      = String()
        feedback.data = status
        self.status_publisher.publish(feedback)
        self.get_logger().info(status)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.vel_publisher.publish(stop)
        node.get_logger().info('Robot stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()