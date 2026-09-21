#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_x_ = msg->point.x;
  goal_y_ = msg->point.y;
  goal_received_ = true;

  RCLCPP_INFO(this->get_logger(), "new goal: (%.2f, %.2f)", goal_x_, goal_y_);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
}

void PlannerNode::timerCallback() {
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "map:%d goal:%d robot(%.2f, %.2f)", map_received_, goal_received_, robot_x_, robot_y_);
}

void PlannerNode::planAndPublish() {
  if (!map_received_ || !goal_received_) return;

  std::vector<robot::CellIndex> cells;
  if (!planner_.planPath(current_map_, robot_x_, robot_y_, goal_x_, goal_y_, cells)) {
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = current_map_.header.frame_id;   // "sim_world"

  for (const robot::CellIndex& c : cells) {
    double wx, wy;
    planner_.gridToWorld(current_map_, c, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);

  RCLCPP_INFO(this->get_logger(), "published path with %zu waypoints", path_msg.poses.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
