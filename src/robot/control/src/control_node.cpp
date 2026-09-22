#include "control_node.hpp"
#include <cmath>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
  path_received_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = control_.extractYaw(msg->pose.pose.orientation);
  odom_received_ = true;
}

void ControlNode::controlLoop() {
  geometry_msgs::msg::Twist stop;

  if (!path_received_ || !odom_received_ || current_path_.poses.empty()) {
    cmd_pub_->publish(stop);
    last_speed_ = 0.0;
    return;
  }

  const auto& last = current_path_.poses.back().pose.position;
  double remaining = control_.computeDistance(robot_x_, robot_y_, last.x, last.y);

  if (remaining < goal_tolerance_) {
    cmd_pub_->publish(stop);
    last_speed_ = 0.0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "at the goal, holding");
    return;
  }

  geometry_msgs::msg::PoseStamped carrot;
  if (!control_.findLookaheadPoint(current_path_, robot_x_, robot_y_, lookahead_distance_, carrot)) {
    cmd_pub_->publish(stop);
    last_speed_ = 0.0;
    return;
  }

  geometry_msgs::msg::Twist cmd = control_.computeVelocity(carrot, robot_x_, robot_y_, robot_yaw_, linear_speed_);

  // ramp up gradually; brake instantly
  if (cmd.linear.x > last_speed_ + max_accel_) {
    double ratio = (last_speed_ + max_accel_) / cmd.linear.x;
    cmd.linear.x  *= ratio;
    cmd.angular.z *= ratio;     // keep the turn radius the same
  }
  last_speed_ = cmd.linear.x;

  cmd_pub_->publish(cmd);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%.2f m to go | v %.2f  w %.2f", remaining, cmd.linear.x, cmd.angular.z);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
