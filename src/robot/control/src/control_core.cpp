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

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::PoseStamped& lookahead,
  double robot_x, double robot_y, double robot_yaw,
  double linear_speed) const {

geometry_msgs::msg::Twist cmd;

double dx = lookahead.pose.position.x - robot_x;
double dy = lookahead.pose.position.y - robot_y;

double cos_yaw = std::cos(robot_yaw);
double sin_yaw = std::sin(robot_yaw);
double x_r =  dx * cos_yaw + dy * sin_yaw;   
double y_r = -dx * sin_yaw + dy * cos_yaw;  

double L_sq = x_r * x_r + y_r * y_r;

if (L_sq < 1e-6) return cmd;                

double curvature = 2.0 * y_r / L_sq;

cmd.linear.x  = linear_speed;
cmd.angular.z = linear_speed * curvature;

return cmd;
}

}  
