#include "planner_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <cmath>

PlannerNode::PlannerNode()
: Node("planner_node"),  // Name matches launch file
  state_(State::WAITING_FOR_GOAL),
  goal_received_(false),
  timeout_sec_(10.0)  // 10 second timeout for replanning
{
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer - 1Hz is sufficient for checking goal reached
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "PlannerNode initialized.");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  // Only replan if actively pursuing a goal
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    RCLCPP_INFO(this->get_logger(), "Map updated. Replanning...");
    planPath();
  }
}
 
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();  // CRITICAL: Reset timeout timer

  RCLCPP_INFO(this->get_logger(), "New goal received (%.2f, %.2f).",
              goal_.point.x, goal_.point.y);

  planPath();
}
 
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
}    
 
void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      goal_received_ = false;
      return;
    }

    // Check timeout for replanning
    const double elapsed = (this->now() - goal_start_time_).seconds();
    if (elapsed > timeout_sec_)
    {
      RCLCPP_WARN(this->get_logger(), "Timeout reached. Replanning...");
      planPath();
      goal_start_time_ = this->now();  // Reset timer after replanning
    }
  }
}
 
bool PlannerNode::goalReached() {
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::hypot(dx, dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::logStartGoalOccupancy() const
{
  if (current_map_.data.empty()) return;

  const auto &info = current_map_.info;
  
  // Lambda to convert world coords to grid indices
  auto toIdx = [&](double x, double y) {
    int mx = static_cast<int>((x - info.origin.position.x) / info.resolution);
    int my = static_cast<int>((y - info.origin.position.y) / info.resolution);
    return std::pair<int,int>(mx,my);
  };
  
  auto [sx, sy] = toIdx(robot_pose_.position.x, robot_pose_.position.y);
  auto [gx, gy] = toIdx(goal_.point.x, goal_.point.y);
  
  const int W = static_cast<int>(info.width);
  const int H = static_cast<int>(info.height);
  auto inb = [&](int x, int y){ return x>=0 && y>=0 && x<W && y<H; };

  // Get occupancy values (127 if out of bounds)
  int sOcc = inb(sx,sy) ? static_cast<int>(current_map_.data[sy*W + sx]) : 127;
  int gOcc = inb(gx,gy) ? static_cast<int>(current_map_.data[gy*W + gx]) : 127;

  RCLCPP_INFO(this->get_logger(),
    "Start idx=(%d,%d) occ=%d | Goal idx=(%d,%d) occ=%d",
    sx, sy, sOcc, gx, gy, gOcc);
}
 
void PlannerNode::planPath()
{
  if (!goal_received_ || current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  // Log occupancy values at start/goal to diagnose failures
  logStartGoalOccupancy();

  // Run A* with nearest-free relocation built in
  nav_msgs::msg::Path path = planner_core::aStarSearch(
      current_map_, 
      robot_pose_, 
      goal_.point, 
      /*occ_thresh=*/50  // Cells with occupancy >= 50 are obstacles
  );

  // Set proper timestamp and frame
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = current_map_.header.frame_id;

  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "Published path with %zu poses.", path.poses.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
