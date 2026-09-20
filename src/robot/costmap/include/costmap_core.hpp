#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cstdint>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeGrid();
    void markObstacle(double range, double angle);
    void inflateObstacle();

    std::vector<int8_t>& getGrid() { return grid_; }
    int getWidth() const { return width_; }
    int getHeight() const {return height_; }
    double getResolution() const {return resolution_; }

  private:
    rclcpp::Logger logger_;
    std::vector<int8_t> grid_;
    int width_;
    int height_;
    double resolution_;
    double inflation_radius_;
    int max_cost_;
};

}  

#endif  