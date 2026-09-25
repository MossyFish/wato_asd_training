#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    struct Params {
      double resolution = 0.1;        // meters per cell
      double size_m = 30.0;           // side length of the square grid, centered on the sensor
      double inflation_radius = 2.0;  // meters
      int max_cost = 100;             // cost of an occupied cell
    };

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    CostmapCore(const rclcpp::Logger& logger, const Params& params);

    // Turns one laser scan into a robot-centered, inflated costmap
    nav_msgs::msg::OccupancyGrid buildCostmap(const sensor_msgs::msg::LaserScan& scan);

  private:
    struct KernelCell {
      int dx;
      int dy;
      int8_t cost;
    };

    void initializeCostmap();
    bool convertToGrid(double range, double angle, int& x_grid, int& y_grid) const;
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();

    rclcpp::Logger logger_;
    Params params_;
    int cells_;  // grid is cells_ x cells_
    std::vector<int8_t> grid_;
    std::vector<std::pair<int, int>> obstacles_;
    std::vector<KernelCell> kernel_;  // precomputed inflation footprint
};

}  

#endif  
