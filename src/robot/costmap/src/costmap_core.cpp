#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger), width_(300), height_(300), resolution_(0.1), inflation_radius_(1.0), max_cost_(100) {}

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

void CostmapCore::inflateObstacle() {
  int radius_cells = static_cast<int>(inflation_radius_ / resolution_);

  std::vector<int> obstacles;
  for (int i = 0; i < width_ * height_; i++) {
    if (grid_[i] == max_cost_) obstacles.push_back(i);
  }

  for (int idx : obstacles) {
    int ox = idx % width_;
    int oy = idx / width_;

    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
      for (int dx = -radius_cells; dx <= radius_cells; dx++) {
        int nx = ox + dx;
        int ny = oy + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

        double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
        if (dist > inflation_radius_) continue;

        int cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));
        int i = ny * width_ + nx;
        if (cost > grid_[i]) grid_[i] = static_cast<int8_t>(cost);
      }
    }
  }
}

}