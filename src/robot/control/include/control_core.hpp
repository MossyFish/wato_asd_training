#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore {
  public:
    struct Params {
      double lookahead_distance = 1.5;   // meters
      double goal_tolerance = 0.3;       // meters, stop when this close to the end of the path
      double linear_speed = 1.0;         // m/s cruising speed
      double max_angular_speed = 1.5;    // rad/s
      double turn_in_place_angle = 0.9;  // rad, past this heading error the robot stops and rotates
      double slow_down_distance = 2.0;   // meters from goal where the robot starts slowing
    };

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger, const Params& params);

    // Next velocity command, or nullopt when there is nothing to follow (no path, or goal reached)
    std::optional<geometry_msgs::msg::Twist> computeCommand(
      const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom) const;

    static double extractYaw(const geometry_msgs::msg::Quaternion& quat);
    static double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);

  private:
    // First point on the path (past the closest one) that is at least lookahead_distance away
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot) const;

    rclcpp::Logger logger_;
    Params params_;
};

}  

#endif 
