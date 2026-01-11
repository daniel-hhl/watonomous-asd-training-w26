#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <cmath>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);
  
  private:
    rclcpp::Logger logger_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};

} 

#endif 

/*
#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <cmath>

namespace robot
{
class ControlCore {
public:
    ControlCore(const rclcpp::Logger& logger);
    
    // Pure Pursuit methods
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path::SharedPtr& path,
        const nav_msgs::msg::Odometry::SharedPtr& odom);
    
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped& target,
        const nav_msgs::msg::Odometry::SharedPtr& odom);
    
    // Utility methods
    double computeDistance(const geometry_msgs::msg::Point& a, 
                          const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);
    
    // Setters for parameters
    void setLookaheadDistance(double distance) { lookahead_distance_ = distance; }
    void setGoalTolerance(double tolerance) { goal_tolerance_ = tolerance; }
    void setLinearSpeed(double speed) { linear_speed_ = speed; }

private:
    rclcpp::Logger logger_;
    
    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};
} 
#endif
*/
