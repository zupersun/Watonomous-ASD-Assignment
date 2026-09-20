#include <chrono>
#include <memory>
#include <cmath>
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  costmap_.initializeGrid();
  RCLCPP_INFO(this->get_logger(), "Costmap ready: %d x %d at %.2f m/cell",
  costmap_.getWidth(), costmap_.getHeight(), costmap_.getResolution());
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.initializeGrid();

  int marked = 0;
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    double range = msg->ranges[i];

    if (std::isnan(range) || range < msg->range_min || range > msg->range_max) {
      continue;
    }

    double angle = msg->angle_min + i * msg->angle_increment;
    costmap_.markObstacle(range, angle);
    marked++;
  }

  costmap_.inflateObstacle();

  nav_msgs::msg::OccupancyGrid grid_msg;

  grid_msg.header.stamp = msg->header.stamp;
  grid_msg.header.frame_id = msg->header.frame_id;
  grid_msg.info.resolution = costmap_.getResolution();
  grid_msg.info.width = costmap_.getWidth();
  grid_msg.info.height = costmap_.getHeight();
  grid_msg.info.origin.position.x = -costmap_.getWidth() * costmap_.getResolution() / 2.0;
  grid_msg.info.origin.position.y = -costmap_.getHeight() * costmap_.getResolution() / 2.0;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.data = costmap_.getGrid();

  costmap_pub_->publish(grid_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "marked %d of %zu beams", marked, msg->ranges.size());
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}