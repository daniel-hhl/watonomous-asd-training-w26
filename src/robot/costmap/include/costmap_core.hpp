#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>
#include <cstdint>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"


namespace robot
{

class CostmapCore {
public:
  explicit CostmapCore(const rclcpp::Logger& logger);

  // --- init costmap ---
  void configure(float resolution, uint32_t width, uint32_t height, const geometry_msgs::msg::Pose& origin);

  void reset(int8_t default_value = 0);

  // --- update from lidar ---
  void updateFromScan(const sensor_msgs::msg::LaserScan& scan);

  // Build an OccupancyGrid message (Step 6)
  nav_msgs::msg::OccupancyGrid toOccupancyGridMsg(const rclcpp::Time& stamp, const std::string& frame_id) const;

private:
  // 2D -> 1D indexing helper
  inline size_t idx(uint32_t x, uint32_t y) const { return static_cast<size_t>(y) * width_ + x; }
  inline bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && (uint32_t)x < width_ && (uint32_t)y < height_; }

  void markObstacleCell(uint32_t x, uint32_t y, int8_t value = 100);
  void inflateObstacles(float inflation_radius_m, int8_t max_cost = 100);

private:
  rclcpp::Logger logger_;

  // Costmap parameters
  float resolution_{0.1f};
  uint32_t width_{100};
  uint32_t height_{100};
  geometry_msgs::msg::Pose origin_;

  // Interpret as a 2D grid via idx(x,y).
  std::vector<int8_t> grid_;
};

}  // namespace robot

#endif