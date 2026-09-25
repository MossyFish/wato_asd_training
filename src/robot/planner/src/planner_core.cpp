#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger, const Params& params)
: logger_(logger), params_(params) {}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y, CellIndex& out) const {
  const double res = map.info.resolution;
  out.x = static_cast<int>(std::floor((x - map.info.origin.position.x) / res));
  out.y = static_cast<int>(std::floor((y - map.info.origin.position.y) / res));
  return inBounds(map, out);
}

void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx, double& x, double& y) const {
  const double res = map.info.resolution;
  x = map.info.origin.position.x + (idx.x + 0.5) * res;
  y = map.info.origin.position.y + (idx.y + 0.5) * res;
}

bool PlannerCore::inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx) const {
  return idx.x >= 0 && idx.y >= 0 &&
         idx.x < static_cast<int>(map.info.width) && idx.y < static_cast<int>(map.info.height);
}

// Expanding square rings around the goal until a traversable cell shows up
bool PlannerCore::findNearestFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& goal, CellIndex& out) const {
  const int w = static_cast<int>(map.info.width);
  const int max_r = static_cast<int>(params_.goal_search_radius / map.info.resolution);
  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int r = 0; r <= max_r && !(found && r > std::ceil(best) + 1); ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;
        }
        CellIndex c(goal.x + dx, goal.y + dy);
        if (!inBounds(map, c) || map.data[static_cast<size_t>(c.y) * w + c.x] >= params_.lethal_cost) {
          continue;
        }
        double d = std::hypot(dx, dy);
        if (d < best) {
          best = d;
          out = c;
          found = true;
        }
      }
    }
  }
  return found;
}

nav_msgs::msg::Path PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid& map, double sx, double sy, double gx, double gy) const {
  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;

  CellIndex start, goal;
  if (!worldToGrid(map, sx, sy, start)) {
    RCLCPP_WARN(logger_, "Planner: robot is outside the map");
    return path;
  }
  if (!worldToGrid(map, gx, gy, goal)) {
    RCLCPP_WARN(logger_, "Planner: goal is outside the map");
    return path;
  }

  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double res = map.info.resolution;

  auto cost_at = [&](const CellIndex& c) -> int {
    int v = map.data[static_cast<size_t>(c.y) * w + c.x];
    return v < 0 ? 0 : v;  // unknown is treated as free
  };
  // If the robot is already sitting inside an inflated zone, it must be allowed to leave it.
  // So nothing costing more than the start cell is passable, but anything up to that is.
  // That lets it climb back out without ever letting a path tunnel deeper into an obstacle.
  const int block_threshold = std::max(params_.lethal_cost, cost_at(start) + 1);
  auto blocked = [&](const CellIndex& c) -> bool {
    return cost_at(c) >= block_threshold;
  };

  if (blocked(goal)) {
    CellIndex alt;
    if (!findNearestFree(map, goal, alt)) {
      RCLCPP_WARN(logger_, "Planner: goal is blocked and no free cell nearby");
      return path;
    }
    RCLCPP_INFO(logger_, "Planner: goal cell is blocked, moving it to nearest free cell");
    goal = alt;
  }

  // ---------------- A* ----------------
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(static_cast<size_t>(w) * h, inf);
  std::vector<int> came_from(static_cast<size_t>(w) * h, -1);
  std::vector<bool> closed(static_cast<size_t>(w) * h, false);
  auto flat = [&](const CellIndex& c) { return c.y * w + c.x; };

  // Octile distance never overestimates a cost of at least 1 per meter, so it is admissible
  auto heuristic = [&](const CellIndex& c) {
    double dx = std::abs(c.x - goal.x);
    double dy = std::abs(c.y - goal.y);
    return res * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  g_score[flat(start)] = 0.0;
  open.emplace(start, heuristic(start));

  static const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool reached = false;
  while (!open.empty()) {
    CellIndex cur = open.top().index;
    open.pop();
    int cur_flat = flat(cur);
    if (closed[cur_flat]) {
      continue;  // stale duplicate entry
    }
    closed[cur_flat] = true;

    if (cur == goal) {
      reached = true;
      break;
    }

    for (int k = 0; k < 8; ++k) {
      CellIndex nb(cur.x + dxs[k], cur.y + dys[k]);
      if (!inBounds(map, nb) || closed[flat(nb)] || blocked(nb)) {
        continue;
      }
      bool diagonal = dxs[k] != 0 && dys[k] != 0;
      // Don't cut the corner of an obstacle on a diagonal step
      if (diagonal && (blocked(CellIndex(cur.x + dxs[k], cur.y)) || blocked(CellIndex(cur.x, cur.y + dys[k])))) {
        continue;
      }
      double step = (diagonal ? std::sqrt(2.0) : 1.0) * res;
      // Cheaper to go through low-cost cells, so paths keep away from obstacles when they can
      double tentative = g_score[cur_flat] + step * (1.0 + params_.cost_weight * cost_at(nb) / 100.0);
      if (tentative < g_score[flat(nb)]) {
        g_score[flat(nb)] = tentative;
        came_from[flat(nb)] = cur_flat;
        open.emplace(nb, tentative + heuristic(nb));
      }
    }
  }

  if (!reached) {
    RCLCPP_WARN(logger_, "Planner: no path found");
    return path;
  }

  // Walk back from the goal to the start
  std::vector<CellIndex> cells;
  for (int i = flat(goal); i != -1; i = came_from[i]) {
    cells.emplace_back(i % w, i / w);
  }
  std::reverse(cells.begin(), cells.end());

  for (const auto& c : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map.header.frame_id;
    gridToWorld(map, c, pose.pose.position.x, pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  // Land exactly on the requested goal when it was reachable as asked
  CellIndex asked;
  if (worldToGrid(map, gx, gy, asked) && asked == goal) {
    path.poses.back().pose.position.x = gx;
    path.poses.back().pose.position.y = gy;
  }
  return path;
}

}
