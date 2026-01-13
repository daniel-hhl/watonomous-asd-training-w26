#include <chrono>
#include <memory>

#include "costmap_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_(robot::CostmapCore(this->get_logger()))
{
  // Publisher
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // Subscriber
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1)
  );

  // tune the defaults later
  geometry_msgs::msg::Pose origin;
  origin.position.x = 0.0;
  origin.position.y = 0.0;
  origin.position.z = 0.0;
  origin.orientation.x = 0.0;
  origin.orientation.y = 0.0;
  origin.orientation.z = 0.0;
  origin.orientation.w = 1.0;

  // Example: 10m x 10m at 0.1m resolution => 100 x 100 cells
  costmap_.configure(/*resolution=*/0.1f, /*width=*/100, /*height=*/100, origin);
  costmap_.reset(0);

  RCLCPP_INFO(this->get_logger(), "Costmap node started: subscribing /lidar, publishing /costmap");
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Step 3/4/5 happen in core (convert -> mark -> inflate)
  costmap_.updateFromScan(*msg);

  // Step 6: publish OccupancyGrid
  auto out = costmap_.toOccupancyGridMsg(this->now(), costmap_frame_id_);
  costmap_pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
