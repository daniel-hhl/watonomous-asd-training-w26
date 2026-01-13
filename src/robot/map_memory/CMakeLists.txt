#include "map_memory_core.hpp"

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

  // If we haven't configured the global map yet, bootstrap it from first costmap.
  if (!global_map_configured_) {
    global_map_ = costmap;
    // Make global start unknown everywhere (memory map should “fill in” over time)
    global_map_.data.assign(costmap.data.size(), -1);
    global_map_configured_ = true;

    RCLCPP_INFO(logger_, "Bootstrapped global map from first costmap: %ux%u @ %.3fm/cell",
                global_map_.info.width, global_map_.info.height, global_map_.info.resolution);
  }
}

void MapMemoryCore::updateOdom(const nav_msgs::msg::Odometry& odom)
{
  current_x_ = odom.pose.pose.position.x;
  current_y_ = odom.pose.pose.position.y;
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
  // Safety: if sizes differ, don't crash — just warn and skip.
  if (latest_costmap_.info.width != global_map_.info.width ||
      latest_costmap_.info.height != global_map_.info.height ||
      latest_costmap_.data.size() != global_map_.data.size())
  {
    RCLCPP_WARN(logger_, "Costmap size mismatch; skipping integration.");
    return;
  }

  // Merge rule from assignment:
  // - if new is known (0..100): overwrite
  // - if new is unknown (-1): keep old
  for (size_t i = 0; i < latest_costmap_.data.size(); ++i) {
    const int8_t new_val = latest_costmap_.data[i];
    if (new_val != -1) {
      global_map_.data[i] = new_val;
    }
  }

  RCLCPP_INFO(logger_, "Integrated latest costmap into global map.");
}

} // namespace robot
