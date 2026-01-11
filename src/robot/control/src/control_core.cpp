#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}


double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, 
                                    const geometry_msgs::msg::Point& b)
{
  // TODO: Implement distance calculation between two points
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat)
{
  // TODO: Implement quaternion to yaw conversion
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  

/*
#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger),
    lookahead_distance_(1.0),
    goal_tolerance_(0.1),
    linear_speed_(0.5)
{}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path::SharedPtr& path,
    const nav_msgs::msg::Odometry::SharedPtr& odom)
{
    if (!path || path->poses.empty() || !odom) {
        return std::nullopt;
    }
    
    const auto& robot_pos = odom->pose.pose.position;
    
    // Find the closest point on the path to the robot
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < path->poses.size(); ++i) {
        double dist = computeDistance(robot_pos, path->poses[i].pose.position);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    // Search forward from the closest point for the lookahead point
    for (size_t i = closest_idx; i < path->poses.size(); ++i) {
        double dist = computeDistance(robot_pos, path->poses[i].pose.position);
        
        if (dist >= lookahead_distance_) {
            return path->poses[i];
        }
    }
    
    // If no point is beyond the lookahead distance, return the last point
    return path->poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target,
    const nav_msgs::msg::Odometry::SharedPtr& odom)
{
    geometry_msgs::msg::Twist cmd_vel;
    
    if (!odom) {
        return cmd_vel;
    }
    
    // Get robot's current position and orientation
    const auto& robot_pos = odom->pose.pose.position;
    double robot_yaw = extractYaw(odom->pose.pose.orientation);
    
    // Calculate the angle to the target point
    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;
    double target_angle = std::atan2(dy, dx);
    
    // Calculate the angular error
    double angle_error = target_angle - robot_yaw;
    
    // Normalize angle to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
    
    // Check if goal is reached
    double distance_to_goal = computeDistance(robot_pos, target.pose.position);
    if (distance_to_goal < goal_tolerance_) {
        RCLCPP_INFO(logger_, "Goal reached!");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return cmd_vel;
    }
    
    // Compute velocity commands
    cmd_vel.linear.x = linear_speed_;
    
    // Pure pursuit curvature calculation
    double curvature = 2.0 * std::sin(angle_error) / lookahead_distance_;
    cmd_vel.angular.z = curvature * linear_speed_;
    
    return cmd_vel;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, 
                                    const geometry_msgs::msg::Point& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat)
{
    // Convert quaternion to yaw angle
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}
*/