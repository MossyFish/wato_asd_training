#include "control_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger, const Params& params)
  : logger_(logger), params_(params) {}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot) const {
  if (path.poses.empty()) {
    return std::nullopt;
  }
  // Closest waypoint, so points the robot already passed are never picked
  size_t closest = 0;
  double best = computeDistance(path.poses[0].pose.position, robot);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    double d = computeDistance(path.poses[i].pose.position, robot);
    if (d < best) {
      best = d;
      closest = i;
    }
  }
  for (size_t i = closest; i < path.poses.size(); ++i) {
    if (computeDistance(path.poses[i].pose.position, robot) >= params_.lookahead_distance) {
      return path.poses[i];
    }
  }
  // Path ends inside the lookahead circle, so aim straight at the goal
  return path.poses.back();
}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeCommand(
  const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom) const {
  if (path.poses.empty()) {
    return std::nullopt;
  }
  const auto& robot = odom.pose.pose.position;
  double dist_to_goal = computeDistance(path.poses.back().pose.position, robot);
  if (dist_to_goal < params_.goal_tolerance) {
    return std::nullopt;
  }

  auto target = findLookaheadPoint(path, robot);
  if (!target) {
    return std::nullopt;
  }

  // Lookahead point expressed in the robot's own frame (x forward, y left)
  double yaw = extractYaw(odom.pose.pose.orientation);
  double dx = target->pose.position.x - robot.x;
  double dy = target->pose.position.y - robot.y;
  double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  double alpha = std::atan2(local_y, local_x);  // heading error to the lookahead point
  double L = std::hypot(local_x, local_y);

  geometry_msgs::msg::Twist cmd;
  if (std::abs(alpha) > params_.turn_in_place_angle || L < 1e-6) {
    // Target is way off to the side or behind: an arc would be huge, so face it first
    cmd.angular.z = std::copysign(params_.max_angular_speed, alpha);
    return cmd;
  }

  // Slow down for sharp turns and when arriving at the goal
  double v = params_.linear_speed * std::cos(alpha);
  if (dist_to_goal < params_.slow_down_distance) {
    v = std::min(v, params_.linear_speed * (dist_to_goal / params_.slow_down_distance));
  }
  v = std::max(v, 0.2);

  // Pure pursuit: the circle through the robot and the lookahead point has curvature 2*y / L^2
  double curvature = 2.0 * local_y / (L * L);
  cmd.linear.x = v;
  cmd.angular.z = std::clamp(v * curvature, -params_.max_angular_speed, params_.max_angular_speed);
  return cmd;
}

}  
