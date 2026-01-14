#include "costmap_core.hpp"

#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger)
{
  // leave config to configure()
}

void CostmapCore::configure(float resolution, uint32_t width, uint32_t height,
                            const geometry_msgs::msg::Pose& origin)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  origin_ = origin;

  grid_.assign(static_cast<size_t>(width_) * height_, 0);
}

void CostmapCore::reset(int8_t default_value)
{
  if (grid_.empty()) {
    grid_.assign(static_cast<size_t>(width_) * height_, default_value);
  } else {
    std::fill(grid_.begin(), grid_.end(), default_value);
  }
}

void CostmapCore::updateFromScan(const sensor_msgs::msg::LaserScan& scan)
{
  // Simple pipeline: clear → mark obstacles → inflate
  reset(0);

  // Step 3 + 4: convert scan points to grid cells and mark occupied
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];

    // ignore invalid ranges
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      continue;
    }

    const float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;

    // x/y in the lidar frame (meters)
    const float x = r * std::cos(angle);
    const float y = r * std::sin(angle);

    // Convert meters -> grid index using origin + resolution
    // Here we assume origin_.position is the (0,0) of the grid in the same frame as the scan.
    // If your sim uses TF frames, you’d normally transform (x,y) into the costmap frame first.
    const int gx = static_cast<int>(std::floor((x - origin_.position.x) / resolution_));
    const int gy = static_cast<int>(std::floor((y - origin_.position.y) / resolution_));

    if (inBounds(gx, gy)) {
      markObstacleCell(static_cast<uint32_t>(gx), static_cast<uint32_t>(gy), 100);
    }
  }

  // Step 5: inflate
  inflateObstacles(/*inflation_radius_m=*/0.5f, /*max_cost=*/100);
}

nav_msgs::msg::OccupancyGrid
CostmapCore::toOccupancyGridMsg(const rclcpp::Time& stamp, const std::string& frame_id) const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin = origin_;

  // OccupancyGrid is 1D row-major; our grid_ already stores it that way.
  msg.data = grid_;
  return msg;
}

void CostmapCore::markObstacleCell(uint32_t x, uint32_t y, int8_t value)
{
  const size_t i = idx(x, y);
  if (i < grid_.size()) {
    grid_[i] = std::max(grid_[i], value);
  }
}

void CostmapCore::inflateObstacles(float inflation_radius_m, int8_t max_cost)
{
  if (inflation_radius_m <= 0.0f) return;

  const int radius_cells = static_cast<int>(std::ceil(inflation_radius_m / resolution_));
  if (radius_cells <= 0) return;

  // Copy original so we inflate from “true obstacles”, not from already-inflated costs
  const std::vector<int8_t> original = grid_;

  for (uint32_t y = 0; y < height_; ++y) {
    for (uint32_t x = 0; x < width_; ++x) {

      // Only inflate around obstacle cells (value >= 100 here)
      if (original[idx(x, y)] < 100) continue;

      const int cx = static_cast<int>(x);
      const int cy = static_cast<int>(y);

      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int nx = cx + dx;
          const int ny = cy + dy;

          if (!inBounds(nx, ny)) continue;

          const float dist_cells = std::sqrt(static_cast<float>(dx * dx + dy * dy));
          const float dist_m = dist_cells * resolution_;
          if (dist_m > inflation_radius_m) continue;

          const float factor = 1.0f - (dist_m / inflation_radius_m);
          const int8_t cost = static_cast<int8_t>(std::round(max_cost * factor));

          const size_t ni = idx(static_cast<uint32_t>(nx), static_cast<uint32_t>(ny));
          grid_[ni] = std::max(grid_[ni], cost);
        }
      }
    }
  }
}

}  // namespace robot
