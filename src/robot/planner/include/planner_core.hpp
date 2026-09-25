#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    struct Params {
      int lethal_cost = 50;             // cells at or above this cost are treated as walls
      double cost_weight = 4.0;         // how strongly A* is pushed away from costly cells
      double goal_search_radius = 4.0;  // meters to look for a free cell if the goal is blocked
    };

    PlannerCore(const rclcpp::Logger& logger, const Params& params);

    // A* from (sx, sy) to (gx, gy), both in the map frame. Empty path if there is no route.
    nav_msgs::msg::Path planPath(
      const nav_msgs::msg::OccupancyGrid& map,
      double sx, double sy, double gx, double gy) const;

  private:
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y, CellIndex& out) const;
    void gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx, double& x, double& y) const;
    bool inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx) const;
    bool findNearestFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& goal, CellIndex& out) const;

    rclcpp::Logger logger_;
    Params params_;
};

}  

#endif  
