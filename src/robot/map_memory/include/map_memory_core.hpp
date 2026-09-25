#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <deque>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

// Robot pose in the map frame, taken from odometry
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// Odometry sample kept so a costmap can be placed using the pose from when its scan was taken
struct TimedPose {
  double stamp;  // seconds
  Pose2D pose;
};

class MapMemoryCore {
  public:
    struct Params {
      double resolution = 0.2;     // meters per cell, coarser than the costmap
      double size_m = 40.0;        // side length of the square global map, centered on the origin
      std::string frame_id = "sim_world";
    };

    MapMemoryCore(const rclcpp::Logger& logger, const Params& params);

    // Stitches a robot-centered costmap into the global map
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, const Pose2D& robot_pose);

    // Pose at a given time, interpolated between the two odometry samples around it
    static Pose2D interpolatePose(const std::deque<TimedPose>& history, double stamp);

    static double yawFromQuaternion(double x, double y, double z, double w);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}  

#endif  
