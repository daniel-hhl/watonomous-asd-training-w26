#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"),
    control_(robot::ControlCore(
        this->get_logger(),
        this->declare_parameter<double>("lookahead_distance", 1.0),
        this->declare_parameter<double>("goal_tolerance", 0.2),
        this->declare_parameter<double>("linear_speed", 0.5)
    ))
{
  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop()
{
    geometry_msgs::msg::Twist stop{};

    // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "No path or odom available: path=%s odom=%s poses=%zu",
            current_path_ ? "yes" : "NO",
            robot_odom_ ? "yes" : "NO", 
            current_path_ ? current_path_->poses.size() : 0);
        cmd_vel_pub_->publish(stop);
        return;
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Control running: %zu poses in path", current_path_->poses.size());
    
    // Find the lookahead point
    auto lookahead_point = control_.findLookaheadPoint(current_path_, robot_odom_);
    if (!lookahead_point) {
        RCLCPP_WARN(this->get_logger(), "No lookahead point found!");
        cmd_vel_pub_->publish(stop);
        return;
    }
    
    // Compute velocity command
    auto cmd_vel = control_.computeVelocity(*lookahead_point, robot_odom_);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Publishing cmd_vel: linear=%.2f angular=%.2f", 
        cmd_vel.linear.x, cmd_vel.angular.z);
    
    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
