#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(get_logger(), loadParams()) {
  distance_threshold_ = this->declare_parameter("distance_threshold", 1.5);
  double update_period = this->declare_parameter("update_period", 1.0);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(update_period), std::bind(&MapMemoryNode::updateMap, this));
}

robot::MapMemoryCore::Params MapMemoryNode::loadParams() {
  robot::MapMemoryCore::Params p;
  p.resolution = this->declare_parameter("resolution", p.resolution);
  p.size_m = this->declare_parameter("size_m", p.size_m);
  p.frame_id = this->declare_parameter("frame_id", p.frame_id);
  return p;
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto& p = msg->pose.pose;
  double x = p.position.x;
  double y = p.position.y;

  robot::TimedPose sample;
  sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
  sample.pose.x = x;
  sample.pose.y = y;
  sample.pose.yaw = robot::MapMemoryCore::yawFromQuaternion(
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  odom_history_.push_back(sample);
  while (odom_history_.size() > 1 && sample.stamp - odom_history_.front().stamp > 5.0) {
    odom_history_.pop_front();
  }

  // First odometry always triggers an update so the map has something in it right away
  if (!have_last_) {
    have_last_ = true;
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
    return;
  }
  double distance = std::hypot(x - last_x_, y - last_y_);
  if (distance >= distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_received_ && !odom_history_.empty()) {
    // Place the costmap using where the robot was when the scan was taken, not where it is now
    double stamp = rclcpp::Time(latest_costmap_.header.stamp).seconds();
    robot::Pose2D pose = robot::MapMemoryCore::interpolatePose(odom_history_, stamp);
    map_memory_.integrateCostmap(latest_costmap_, pose);
    should_update_map_ = false;
  }

  // Publish every tick, even when nothing changed, so late subscribers (planner, Foxglove)
  // get a map without the robot having to move first
  nav_msgs::msg::OccupancyGrid map = map_memory_.map();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
