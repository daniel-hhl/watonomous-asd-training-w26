#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

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