#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  const auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  
  double dx = robot_x_ - last_x_;
  double dy = robot_y_ - last_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance >= distance_threshold_ || first_update_) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    first_update_ = false;
    should_update_ = true;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "robot at (%.2f, %.2f) yaw %.2f", robot_x_, robot_y_, robot_yaw_);
}

void MapMemoryNode::updateMap() {
  if (!should_update_ || !costmap_received_) return;

  map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
  should_update_ = false;

  RCLCPP_INFO(this->get_logger(), "integrated costmap at (%.2f, %.2f)", robot_x_, robot_y_);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}