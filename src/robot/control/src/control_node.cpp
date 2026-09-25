#include "control_node.hpp"

#include <chrono>
#include <memory>

ControlNode::ControlNode(): Node("control"), control_(get_logger(), loadParams()) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  double period = this->declare_parameter("control_period", 0.1);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period), [this]() { controlLoop(); });
}

robot::ControlCore::Params ControlNode::loadParams() {
  robot::ControlCore::Params p;
  p.lookahead_distance = this->declare_parameter("lookahead_distance", p.lookahead_distance);
  p.goal_tolerance = this->declare_parameter("goal_tolerance", p.goal_tolerance);
  p.linear_speed = this->declare_parameter("linear_speed", p.linear_speed);
  p.max_angular_speed = this->declare_parameter("max_angular_speed", p.max_angular_speed);
  p.turn_in_place_angle = this->declare_parameter("turn_in_place_angle", p.turn_in_place_angle);
  p.slow_down_distance = this->declare_parameter("slow_down_distance", p.slow_down_distance);
  return p;
}

void ControlNode::controlLoop() {
  if (!robot_odom_) {
    return;
  }

  std::optional<geometry_msgs::msg::Twist> cmd;
  if (current_path_) {
    cmd = control_.computeCommand(*current_path_, *robot_odom_);
  }

  if (cmd) {
    cmd_vel_pub_->publish(*cmd);
    moving_ = true;
  } else if (moving_) {
    // Path ran out, was cancelled, or we arrived
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    moving_ = false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
