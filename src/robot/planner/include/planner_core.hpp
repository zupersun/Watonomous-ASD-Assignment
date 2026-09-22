#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <functional>

namespace robot
{

struct CellIndex {
  int x;
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }
  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    bool planPath(const nav_msgs::msg::OccupancyGrid& map, double start_x, double start_y, double goal_x, double goal_y, std::vector<CellIndex>& out_path);
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& out) const;
    void gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c, double& wx, double& wy) const;

  private:
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    std::vector<CellIndex> getNeighbors(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c, bool from_lethal) const;

    void reconstructPath(
      const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, 
      const CellIndex& goal, 
      std::vector<CellIndex>& out_path) const;
    
    rclcpp::Logger logger_;
};

}  

#endif  
