#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(get_logger(), loadParams()) {
  // Initialize the constructs and their parameters
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

robot::CostmapCore::Params CostmapNode::loadParams() {
  robot::CostmapCore::Params p;
  p.resolution = this->declare_parameter("resolution", p.resolution);
  p.size_m = this->declare_parameter("size_m", p.size_m);
  p.inflation_radius = this->declare_parameter("inflation_radius", p.inflation_radius);
  p.max_cost = this->declare_parameter("max_cost", p.max_cost);
  return p;
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_pub_->publish(costmap_.buildCostmap(*scan));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
