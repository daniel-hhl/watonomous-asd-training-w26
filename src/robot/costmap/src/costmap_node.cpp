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

  // Define a 30m x 30m grid centered at (0,0) with 0.1m resolution
  constexpr float RES = 0.1f;
  constexpr uint32_t W = 300;
  constexpr uint32_t H = 300;

  geometry_msgs::msg::Pose origin;
  origin.position.x = - (W * RES) / 2.0f;  // -15.0
  origin.position.y = - (H * RES) / 2.0f;  // -15.0
  origin.position.z = 0.0;
  origin.orientation.x = 0.0;
  origin.orientation.y = 0.0;
  origin.orientation.z = 0.0;
  origin.orientation.w = 1.0;

  costmap_.configure(RES, W, H, origin);
  costmap_.reset(0);

  RCLCPP_INFO(this->get_logger(), "Costmap node started: subscribing /lidar, publishing /costmap");
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  costmap_.updateFromScan(*msg);

  auto out = costmap_.toOccupancyGridMsg(msg->header.stamp, msg->header.frame_id);
  costmap_pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
