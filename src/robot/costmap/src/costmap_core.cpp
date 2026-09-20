#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger), width_(300), height_(300), resolution_(0.1) {}

void CostmapCore::initializeGrid() {
  grid_.assign(width_ * height_, 0);
}

void CostmapCore::markObstacle(double range, double angle) {
  double x_m = range * std::cos(angle);
  double y_m = range * std::sin(angle);
  
  int gx = static_cast<int>(std::round(x_m / resolution_)) + width_ / 2;
  int gy = static_cast<int>(std::round(y_m / resolution_)) + height_ / 2;

  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return;

  grid_[gy * width_ + gx] = 100;
}

}