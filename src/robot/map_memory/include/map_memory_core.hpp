#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger& logger);

  // Configure global-map shape (you can hardcode defaults if you want)
  void configure(float resolution,
                 uint32_t width,
                 uint32_t height,
                 const geometry_msgs::msg::Pose& origin);

  // Inputs (called by node callbacks)
  void setLatestCostmap(const nav_msgs::msg::OccupancyGrid& costmap);
  void updateOdom(const nav_msgs::msg::Odometry& odom);

  // Called by node timer: returns true if global map was updated
  bool maybeUpdateGlobalMap();

  // Output (node publishes this)
  nav_msgs::msg::OccupancyGrid getGlobalMapMsg(const rclcpp::Time& stamp,
                                               const std::string& frame_id) const;

private:
  // Helpers
  double distance2D(double x0, double y0, double x1, double y1) const;

  // Merge rule from assignment:
  // if new cell known -> overwrite, if unknown (-1) -> keep old
  void integrateLatestCostmap();

private:
  rclcpp::Logger logger_;

  // --- global map state ---
  nav_msgs::msg::OccupancyGrid global_map_;
  bool global_map_configured_{false};

  // --- latest inputs ---
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool have_costmap_{false};

  bool have_odom_{false};
  double current_x_{0.0};
  double current_y_{0.0};

  // --- update gating ---
  double last_update_x_{0.0};
  double last_update_y_{0.0};
  bool have_last_update_pose_{false};

  double distance_threshold_m_{1.5};  // assignment mentions 1.5m (your screenshot)
  bool should_update_map_{false};
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_
