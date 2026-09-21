#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) const {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) const {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool ControlCore::findLookaheadPoint(const nav_msgs::msg::Path& path, double robot_x, double robot_y, double lookahead, geometry_msgs::msg::PoseStamped& out) const {
  if (path.poses.empty()) return false;

  for (const auto& pose : path.poses) {
    double d = computeDistance(robot_x, robot_y,
    pose.pose.position.x, pose.pose.position.y);
    
    if (d >= lookahead) {
      out = pose;
      return true;
    }
  }

  out = path.poses.back();
  return true;
}

}  
