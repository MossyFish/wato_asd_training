#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

PlannerNode::PlannerNode() : Node("planner"), planner_(get_logger(), loadParams()) {
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.5);
  replan_timeout_ = this->declare_parameter("replan_timeout", 3.0);
  double timer_period = this->declare_parameter("timer_period", 0.5);
  last_plan_time_ = this->now();

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period), std::bind(&PlannerNode::timerCallback, this));
}

robot::PlannerCore::Params PlannerNode::loadParams() {
  robot::PlannerCore::Params p;
  p.lethal_cost = this->declare_parameter("lethal_cost", p.lethal_cost);
  p.cost_weight = this->declare_parameter("cost_weight", p.cost_weight);
  p.goal_search_radius = this->declare_parameter("goal_search_radius", p.goal_search_radius);
  return p;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Map memory republishes constantly, only treat it as news when the cells actually differ
  if (!have_map_ || msg->data != current_map_.data) {
    map_changed_ = true;
  }
  current_map_ = *msg;
  have_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL && map_changed_) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }
  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    // An empty path tells the controller to stop
    nav_msgs::msg::Path empty;
    empty.header.stamp = this->now();
    empty.header.frame_id = current_map_.header.frame_id;
    path_pub_->publish(empty);
    return;
  }
  // Plan again if it has been a while (a stuck robot or a failed plan gets retried here)
  if ((this->now() - last_plan_time_).seconds() > replan_timeout_) {
    RCLCPP_INFO(this->get_logger(), "Replanning due to timeout");
    planPath();
  }
}

bool PlannerNode::goalReached() const {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  last_plan_time_ = this->now();
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing %s", have_map_ ? "odometry" : "map");
    return;
  }
  map_changed_ = false;

  nav_msgs::msg::Path path = planner_.planPath(
    current_map_, robot_pose_.position.x, robot_pose_.position.y, goal_.point.x, goal_.point.y);
  path.header.stamp = this->now();
  // An empty path (no route) also gets published, so the robot stops instead of following an old plan
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
