#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

enum class RobotState {
    FORWARD,
    AVOIDING
};

class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance() : Node("obstacle_avoidance_node"), current_state_(RobotState::FORWARD), chosen_turn_direction_(0.0)
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "State-Machine Obstacle Avoidance Started.");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        float current_angle = scan_msg->angle_min;
        
        float min_front_dist = scan_msg->range_max;
        float min_left_dist = scan_msg->range_max;
        float min_right_dist = scan_msg->range_max;
        bool front_obstacle = false;

        // Process LiDAR Arrays Geometrically
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            float range = scan_msg->ranges[i];
            float angle = atan2(sin(current_angle), cos(current_angle));
            current_angle += scan_msg->angle_increment;

            if (range <= scan_msg->range_min || range >= scan_msg->range_max || std::isnan(range))
                continue;

            // Wide Front Arc for tight corridors (-35 to +35 degrees)
            if (angle >= -35.0 * M_PI / 180.0 && angle <= 35.0 * M_PI / 180.0)
            {
                if (range <= OBSTACLE_THRESHOLD)
                {
                    front_obstacle = true;
                    if (range < min_front_dist) min_front_dist = range;
                }
            }
            // Side tracking
            else if (angle > 35.0 * M_PI / 180.0 && angle <= 110.0 * M_PI / 180.0)
            {
                if (range < min_left_dist) min_left_dist = range;
            }
            else if (angle >= -110.0 * M_PI / 180.0 && angle < -35.0 * M_PI / 180.0)
            {
                if (range < min_right_dist) min_right_dist = range;
            }
        }

        // State Machine Logic
        switch (current_state_)
        {
            case RobotState::FORWARD:
                if (front_obstacle)
                {
                    // Transition to Avoidance state and lock in a choice
                    current_state_ = RobotState::AVOIDING;
                    twist_msg.linear.x = 0.0;

                    // Lock turning direction memory so it does not oscillate
                    if (min_left_dist >= min_right_dist) {
                        chosen_turn_direction_ = ROTATION_SPEED;  // Left
                        RCLCPP_WARN(this->get_logger(), "Obstacle at %.2fm. Turn choice locked: LEFT", min_front_dist);
                    } else {
                        chosen_turn_direction_ = -ROTATION_SPEED; // Right
                        RCLCPP_WARN(this->get_logger(), "Obstacle at %.2fm. Turn choice locked: RIGHT", min_front_dist);
                    }
                    twist_msg.angular.z = chosen_turn_direction_;
                }
                else
                {
                    // Move forward cleanly
                    twist_msg.linear.x = FORWARD_SPEED;
                    twist_msg.angular.z = 0.0;
                }
                break;

            case RobotState::AVOIDING:
                // If a path opens up that is wider than our threshold, switch back to forward
                if (!front_obstacle)
                {
                    RCLCPP_INFO(this->get_logger(), "Path cleared! Resuming forward cruise.");
                    current_state_ = RobotState::FORWARD;
                    twist_msg.linear.x = FORWARD_SPEED;
                    twist_msg.angular.z = 0.0;
                }
                else
                {
                    // Maintain the locked turn direction regardless of side updates
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = chosen_turn_direction_;
                }
                break;
        }

        cmd_vel_pub_->publish(twist_msg);
    }

    // Parameters optimized for tight maze profiles
    const float OBSTACLE_THRESHOLD = 0.75; // Lowered slightly to clear low doorways
    const float FORWARD_SPEED = 0.4;       
    const float ROTATION_SPEED = 0.5;      // Faster snap-turns to clear wide footprints

    RobotState current_state_;
    float chosen_turn_direction_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}