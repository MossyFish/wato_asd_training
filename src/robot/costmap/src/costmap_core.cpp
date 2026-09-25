#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger, const Params& params)
  : logger_(logger), params_(params) {
  cells_ = static_cast<int>(std::round(params_.size_m / params_.resolution));
  grid_.assign(static_cast<size_t>(cells_) * cells_, 0);

  // The inflation footprint is the same for every obstacle cell, so build it once
  const int radius_cells = static_cast<int>(std::ceil(params_.inflation_radius / params_.resolution));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      double distance = std::hypot(dx, dy) * params_.resolution;
      if (distance > params_.inflation_radius) {
        continue;
      }
      int cost = static_cast<int>(params_.max_cost * (1.0 - distance / params_.inflation_radius));
      if (cost > 0) {
        kernel_.push_back({dx, dy, static_cast<int8_t>(cost)});
      }
    }
  }
  RCLCPP_INFO(logger_, "Costmap: %dx%d cells at %.2f m, inflation radius %.2f m",
              cells_, cells_, params_.resolution, params_.inflation_radius);
}

void CostmapCore::initializeCostmap() {
  std::fill(grid_.begin(), grid_.end(), static_cast<int8_t>(0));
  obstacles_.clear();
}

// The grid is centered on the sensor, so the sensor sits in the middle cell
bool CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) const {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  x_grid = static_cast<int>(std::floor(x / params_.resolution)) + cells_ / 2;
  y_grid = static_cast<int>(std::floor(y / params_.resolution)) + cells_ / 2;
  return x_grid >= 0 && x_grid < cells_ && y_grid >= 0 && y_grid < cells_;
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  int8_t& cell = grid_[static_cast<size_t>(y_grid) * cells_ + x_grid];
  if (cell != params_.max_cost) {
    cell = static_cast<int8_t>(params_.max_cost);
    obstacles_.emplace_back(x_grid, y_grid);
  }
}

void CostmapCore::inflateObstacles() {
  for (const auto& [ox, oy] : obstacles_) {
    for (const auto& k : kernel_) {
      int x = ox + k.dx;
      int y = oy + k.dy;
      if (x < 0 || x >= cells_ || y < 0 || y >= cells_) {
        continue;
      }
      int8_t& cell = grid_[static_cast<size_t>(y) * cells_ + x];
      if (k.cost > cell) {
        cell = k.cost;
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::buildCostmap(const sensor_msgs::msg::LaserScan& scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    // inf / nan mean the beam hit nothing
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) {
      continue;
    }
    double angle = scan.angle_min + i * scan.angle_increment;
    int x_grid, y_grid;
    if (convertToGrid(range, angle, x_grid, y_grid)) {
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = scan.header.stamp;
  msg.header.frame_id = scan.header.frame_id.empty() ? "robot/chassis/lidar" : scan.header.frame_id;
  msg.info.resolution = params_.resolution;
  msg.info.width = cells_;
  msg.info.height = cells_;
  // Middle cell is the sensor, so shift the origin back by half the grid
  msg.info.origin.position.x = -(cells_ / 2) * params_.resolution;
  msg.info.origin.position.y = -(cells_ / 2) * params_.resolution;
  msg.info.origin.orientation.w = 1.0;
  msg.data = grid_;
  return msg;
}

}
