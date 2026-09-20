#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), width_(300), height_(300), resolution_(0.1), origin_x_(-15.0), origin_y_(-15.0) {
  initializeMap();
}

void MapMemoryCore::initializeMap() {
  global_map_.assign(width_ * height_, 0);
}

void MapMemoryCore::integrateCostmap(
    const nav_msgs::msg::OccupancyGrid& local,
    double robot_x, double robot_y, double robot_yaw) {
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  for (unsigned int ly = 0; ly < local.info.height; ly++) {
    for (unsigned int lx = 0; lx < local.info.width; lx++) {
      int8_t value = local.data[ly * local.info.width + lx];
      if (value <= 0) continue;

      double x_l = lx * local.info.resolution + local.info.origin.position.x;
      double y_l = ly * local.info.resolution + local.info.origin.position.y;
      double x_w = x_l * cos_yaw - y_l * sin_yaw + robot_x;
      double y_w = x_l * sin_yaw + y_l * cos_yaw + robot_y;

      int gx = static_cast<int>(std::round((x_w - origin_x_) / resolution_));
      int gy = static_cast<int>(std::round((y_w - origin_y_) / resolution_));

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) continue;

      int i = gy * width_ + gx;
      if (value > global_map_[i]) global_map_[i] = value;
    }
  }
}  

} 
