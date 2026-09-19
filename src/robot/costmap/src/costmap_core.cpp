#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger), width_(300), height_(300), resolution_(0.1) {}

void CostmapCore::initializeGrid() {
    grid_.assign(width_ * height_, 0);
}
}