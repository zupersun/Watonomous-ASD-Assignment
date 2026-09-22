#include "planner_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& out) const {
  int gx = static_cast<int>(std::round((wx - map.info.origin.position.x) / map.info.resolution));
  int gy = static_cast<int>(std::round((wy - map.info.origin.position.y) / map.info.resolution));

  if (gx < 0 || gx >= static_cast<int>(map.info.width) || gy < 0 || gy >= static_cast<int>(map.info.height)) {
      return false;
  }

  out = CellIndex(gx, gy);
  return true;
}

void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c, double& wx, double& wy) const {
  wx = c.x * map.info.resolution + map.info.origin.position.x;
  wy = c.y * map.info.resolution + map.info.origin.position.y;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerCore::getNeighbors(
    const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const {
  std::vector<CellIndex> out;

  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      if (dx == 0 && dy == 0) continue;         

      int nx = c.x + dx;
      int ny = c.y + dy;

      if (nx < 0 || nx >= static_cast<int>(map.info.width) ||
          ny < 0 || ny >= static_cast<int>(map.info.height)) continue;

      if (map.data[ny * map.info.width + nx] >= 100) continue; 

      out.emplace_back(nx, ny);
    }
  }
  return out;
}

void PlannerCore::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
    const CellIndex& goal,
    std::vector<CellIndex>& out_path) const {
  out_path.clear();

  CellIndex c = goal;
  out_path.push_back(c);

  while (came_from.count(c)) {      
    c = came_from.at(c);
    out_path.push_back(c);
  }

  std::reverse(out_path.begin(), out_path.end());   
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map, double start_x, double start_y, double goal_x, double goal_y, std::vector<CellIndex>& out_path) {
  CellIndex start, goal;
  if (!worldToGrid(map, start_x, start_y, start)) {
    RCLCPP_WARN(logger_, "start is outside the map");
    return false;
  }
  if (!worldToGrid(map, goal_x, goal_y, goal)) {
    RCLCPP_WARN(logger_, "goal is outside the map");
    return false;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start, goal));

  while (!open.empty()) {
    AStarNode current = open.top();
    open.pop();

    if (current.index == goal) {
      reconstructPath(came_from, goal, out_path);
      return true;
    }

    if (closed.count(current.index)) continue;    
    closed.insert(current.index);

    for (const CellIndex& nb : getNeighbors(map, current.index)) {
      if (closed.count(nb)) continue;

      bool diagonal = (nb.x != current.index.x) && (nb.y != current.index.y);
      double step = diagonal ? 1.41421356 : 1.0;

      double penalty = static_cast<double>(map.data[nb.y * map.info.width + nb.x]) / 5.0;

      double tentative_g = g_score[current.index] + step + penalty;

      if (!g_score.count(nb) || tentative_g < g_score[nb]) {
        g_score[nb] = tentative_g;
        came_from[nb] = current.index;
        open.emplace(nb, tentative_g + heuristic(nb, goal));
      }
    }
  }

  RCLCPP_WARN(logger_, "no path found");
  return false;
}

} 
