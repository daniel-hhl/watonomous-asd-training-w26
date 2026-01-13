#include "planner_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), state_(State::WAITING_FOR_GOAL), core_() {
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/move_base_simple/goal", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
 
  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
 
  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;

    // Log map information when received
    RCLCPP_INFO(this->get_logger(), 
      "Received costmap: %dx%d cells, resolution %.3f m/cell", 
      msg->info.width, msg->info.height, msg->info.resolution);

    core_.setMap(current_map_.data,
      current_map_.info.width,
      current_map_.info.height,
      current_map_.info.resolution,
      current_map_.info.origin.position.x,
      current_map_.info.origin.position.y);
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      planPath();
    }
  }
 
void PlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

        // Log goal reception
        RCLCPP_INFO(this->get_logger(), "Received new goal: (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);

        planPath();
}
 
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
}    
 
void PlannerNode::timerCallback() {
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            if (goalReached()) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                state_ = State::WAITING_FOR_GOAL;
                goal_received_ = false;
            } else {
                // comment out if terminal is filled
                RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
                planPath();
            }
        }
}
 
bool PlannerNode::goalReached() {
        double dx = goal_.pose.position.x - robot_pose_.position.x;
        double dy = goal_.pose.position.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}
 
void PlannerNode::planPath() {
        if (!goal_received_ || current_map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
            return;
        }

        // Log start and goal positions before planning
        RCLCPP_INFO(this->get_logger(), "Planning from (%.2f, %.2f) to (%.2f, %.2f)", robot_pose_.position.x, robot_pose_.position.y,
          goal_.pose.position.x, goal_.pose.position.y);
 
        std::vector<std::pair<double,double>> points;
        bool ok = core_.plan(
          robot_pose_.position.x, robot_pose_.position.y,
          goal_.pose.position.x, goal_.pose.position.y,
          points);

        if (!ok) {
          RCLCPP_WARN(this->get_logger(), "A* failed to find a path (start/goal may be in obstacle or unreachable)");
          return;
        }
        
        // Log success
        RCLCPP_INFO(this->get_logger(), "A* succeeded with %zu waypoints", points.size());

        // A* Implementation (pseudo-code)
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";
 
        // Compute path using A* on current_map_
        // Fill path.poses with the resulting waypoints.
        for (auto &p : points) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header = path.header;
          pose.pose.position.x = p.first;
          pose.pose.position.y = p.second;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.w = 1.0;
          path.poses.push_back(pose);
        }
 
        path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
