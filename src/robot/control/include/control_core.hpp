#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
    
    double computeDistance(double x1, double y1, double x2, double y2) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& q) const;

  private:
    rclcpp::Logger logger_;
};

} 

#endif 
