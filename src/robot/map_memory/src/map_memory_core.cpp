#include "map_memory_core.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
: logger_(logger)
{
}

void MapMemoryCore::configure(float resolution,
                              uint32_t width,
                              uint32_t height,
                              const geometry_msgs::msg::Pose& origin)
{
  global_map_.info.resolution = resolution;
  global_map_.info.width = width;
  global_map_.info.height = height;
  global_map_.info.origin = origin;

  global_map_.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), -1); // unknown
  global_map_configured_ = true;

  RCLCPP_INFO(logger_, "Global map configured: %ux%u @ %.3fm/cell", width, height, resolution);
}

void MapMemoryCore::setLatestCostmap(const nav_msgs::msg::OccupancyGrid& costmap)
{
  latest_costmap_ = costmap;
  have_costmap_ = true;

  // If we haven't configured the global map yet, initialize fixed world-centered grid
  if (!global_map_configured_) {
    // CHANGED: Create fixed 30m × 30m map (300 cells × 0.1m resolution) centered at world origin
    global_map_.info.resolution = 0.1;
    global_map_.info.width = 300;
    global_map_.info.height = 300;
    global_map_.info.origin.position.x = -15.0;  // Center at (0,0) in world
    global_map_.info.origin.position.y = -15.0;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;  // Identity quaternion
    global_map_.data.assign(300 * 300, 0);  // CHANGED: Initialize to 0 (free), not -1 (unknown)
    global_map_configured_ = true;

    RCLCPP_INFO(logger_, "Initialized fixed 300×300 global map centered at world origin");
  }
}

void MapMemoryCore::updateOdom(const nav_msgs::msg::Odometry& odom)
{
  current_x_ = odom.pose.pose.position.x;
  current_y_ = odom.pose.pose.position.y;

  // Extract yaw from quaternion
  tf2::Quaternion q(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_yaw_);  // Extract roll, pitch, yaw

  have_odom_ = true;

  // First time: set baseline
  if (!have_last_update_pose_) {
    last_update_x_ = current_x_;
    last_update_y_ = current_y_;
    have_last_update_pose_ = true;
    should_update_map_ = true; // allow first integration
    return;
  }

  const double dist = distance2D(current_x_, current_y_, last_update_x_, last_update_y_);
  if (dist >= distance_threshold_m_) {
    should_update_map_ = true;
  }
}

bool MapMemoryCore::maybeUpdateGlobalMap()
{
  if (!global_map_configured_ || !have_costmap_ || !have_odom_) {
    return false;
  }

  if (!should_update_map_) {
    return false;
  }

  // For now: naive integration (assumes costmap grid aligns with global map grid)
  integrateLatestCostmap();

  // Commit update position + reset gate
  last_update_x_ = current_x_;
  last_update_y_ = current_y_;
  should_update_map_ = false;

  return true;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMapMsg(const rclcpp::Time& stamp,
                                                           const std::string& frame_id) const
{
  auto out = global_map_;
  out.header.stamp = stamp;
  out.header.frame_id = frame_id;
  return out;
}

double MapMemoryCore::distance2D(double x0, double y0, double x1, double y1) const
{
  const double dx = x0 - x1;
  const double dy = y0 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

void MapMemoryCore::integrateLatestCostmap()
{
  const double cos_yaw = std::cos(current_yaw_);
  const double sin_yaw = std::sin(current_yaw_);
  
  const auto& costmap_info = latest_costmap_.info;
  const auto& global_info = global_map_.info;
  
  // Costmap origin in its own frame (usually centered on robot or offset)
  const double costmap_origin_x = costmap_info.origin.position.x;
  const double costmap_origin_y = costmap_info.origin.position.y;
  
  // Iterate through every cell in the costmap
  for (uint32_t cy = 0; cy < costmap_info.height; ++cy) {
    for (uint32_t cx = 0; cx < costmap_info.width; ++cx) {
      const size_t costmap_idx = cy * costmap_info.width + cx;
      const int8_t value = latest_costmap_.data[costmap_idx];
      
      // Skip unknown cells
      if (value == -1) {
        continue;
      }
      
      // Assume costmap is centered on robot, not using costmap origin
      const double local_x = (static_cast<double>(cx) - costmap_info.width / 2.0) * costmap_info.resolution;
      const double local_y = (static_cast<double>(cy) - costmap_info.height / 2.0) * costmap_info.resolution;
      
      // Transform to global frame using robot pose
      const double global_x = current_x_ + (local_x * cos_yaw - local_y * sin_yaw);
      const double global_y = current_y_ + (local_x * sin_yaw + local_y * cos_yaw);
      
      // Convert global (x, y) to global map cell indices
      const double dx = global_x - global_info.origin.position.x;
      const double dy = global_y - global_info.origin.position.y;
      const int gx = static_cast<int>(dx / global_info.resolution);
      const int gy = static_cast<int>(dy / global_info.resolution);
      
      // Bounds check
      if (gx < 0 || gx >= static_cast<int>(global_info.width) ||
          gy < 0 || gy >= static_cast<int>(global_info.height)) {
        continue;
      }
      
      // Use max to keep obstacles (don't let free space overwrite obstacles)
      const size_t global_idx = gy * global_info.width + gx;
      global_map_.data[global_idx] = std::max(global_map_.data[global_idx], value);
    }
  }
  
  RCLCPP_INFO(logger_, "Integrated costmap at pose (%.2f, %.2f, %.2f rad)", 
              current_x_, current_y_, current_yaw_);
}

} // namespace robot
