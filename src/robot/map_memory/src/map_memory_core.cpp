#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger, const Params& params)
  : logger_(logger) {
  int cells = static_cast<int>(std::round(params.size_m / params.resolution));
  map_.header.frame_id = params.frame_id;
  map_.info.resolution = params.resolution;
  map_.info.width = cells;
  map_.info.height = cells;
  map_.info.origin.position.x = -params.size_m / 2.0;
  map_.info.origin.position.y = -params.size_m / 2.0;
  map_.info.origin.orientation.w = 1.0;
  // Nothing seen yet, so everything is free until a costmap says otherwise
  map_.data.assign(static_cast<size_t>(cells) * cells, 0);
}

double MapMemoryCore::yawFromQuaternion(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

Pose2D MapMemoryCore::interpolatePose(const std::deque<TimedPose>& history, double stamp) {
  if (history.empty()) {
    return Pose2D();
  }
  // Clock mismatch or a very old scan: just use the freshest pose rather than guessing
  if (stamp >= history.back().stamp || stamp < history.front().stamp - 0.5) {
    return history.back().pose;
  }
  if (stamp <= history.front().stamp) {
    return history.front().pose;
  }
  for (size_t i = 1; i < history.size(); ++i) {
    if (history[i].stamp >= stamp) {
      const TimedPose& a = history[i - 1];
      const TimedPose& b = history[i];
      double span = b.stamp - a.stamp;
      double t = span > 1e-9 ? (stamp - a.stamp) / span : 0.0;
      Pose2D out;
      out.x = a.pose.x + t * (b.pose.x - a.pose.x);
      out.y = a.pose.y + t * (b.pose.y - a.pose.y);
      // Blend the shortest way around so we don't spin through +-pi
      double dyaw = std::atan2(std::sin(b.pose.yaw - a.pose.yaw), std::cos(b.pose.yaw - a.pose.yaw));
      out.yaw = a.pose.yaw + t * dyaw;
      return out;
    }
  }
  return history.back().pose;
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, const Pose2D& robot_pose) {
  const double c = std::cos(robot_pose.yaw);
  const double s = std::sin(robot_pose.yaw);
  const double cm_res = costmap.info.resolution;
  const double map_res = map_.info.resolution;
  const int map_w = static_cast<int>(map_.info.width);
  const int map_h = static_cast<int>(map_.info.height);
  const int cm_w = static_cast<int>(costmap.info.width);
  const int cm_h = static_cast<int>(costmap.info.height);

  // Walk the (finer) costmap cells and drop each one into the (coarser) map cell it lands in.
  // Going costmap -> map this way means no map cell inside the scan area gets skipped.
  for (int j = 0; j < cm_h; ++j) {
    for (int i = 0; i < cm_w; ++i) {
      int8_t value = costmap.data[static_cast<size_t>(j) * cm_w + i];
      // Free/unknown cells carry no new information for a static world
      if (value <= 0) {
        continue;
      }
      // Cell center in the robot frame, then rotate + translate into the map frame
      double lx = costmap.info.origin.position.x + (i + 0.5) * cm_res;
      double ly = costmap.info.origin.position.y + (j + 0.5) * cm_res;
      double wx = robot_pose.x + c * lx - s * ly;
      double wy = robot_pose.y + s * lx + c * ly;

      int mx = static_cast<int>(std::floor((wx - map_.info.origin.position.x) / map_res));
      int my = static_cast<int>(std::floor((wy - map_.info.origin.position.y) / map_res));
      if (mx < 0 || mx >= map_w || my < 0 || my >= map_h) {
        continue;
      }
      int8_t& cell = map_.data[static_cast<size_t>(my) * map_w + mx];
      // Keep the highest cost ever seen. The world is static, so a cell that looked
      // dangerous from one angle stays dangerous even if another scan couldn't see the obstacle.
      cell = std::max(cell, value);
    }
  }
}

}
