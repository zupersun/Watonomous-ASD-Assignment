#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cstdint>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initializeMap();
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& local, double robot_x, double robot_y, double robot_yaw);

    std::vector<int8_t>& getGlobalMap() { return global_map_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }

  private:
    rclcpp::Logger logger_;
    std::vector<int8_t> global_map_;
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;
};

}  

#endif  
