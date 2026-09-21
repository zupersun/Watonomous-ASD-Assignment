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

} 
