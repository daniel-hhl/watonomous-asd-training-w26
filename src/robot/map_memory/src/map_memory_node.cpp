#include "map_memory_node.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  // Publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  // Timer throttles updates (optimization per assignment)
  timer_ = this->create_wall_timer(
    1s,
    std::bind(&MapMemoryNode::onTimer, this)
  );

  RCLCPP_INFO(this->get_logger(), "MapMemoryNode started: sub /costmap, sub /odom/filtered, pub /map");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_memory_.setLatestCostmap(*msg);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  map_memory_.updateOdom(*msg);
}

void MapMemoryNode::onTimer()
{
  if (map_memory_.maybeUpdateGlobalMap()) {
    auto out = map_memory_.getGlobalMapMsg(this->now(), map_frame_id_);
    map_pub_->publish(out);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
