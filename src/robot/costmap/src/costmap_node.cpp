#pragma once
#include <chrono>
#include <memory>
#include <vector>

#include "costmap_node.hpp"
#include <cmath>
#include <algorithm> 

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  grid_.assign(width_ * height_, 0);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&CostmapNode::publishMessage, this));

  RCLCPP_INFO(this->get_logger(), "costmap node started, publishing /costmap");
}

void CostmapNode::publishMessage() {
  nav_msgs::msg::OccupancyGrid msg;

  // WHEN this data is valid, and WHICH coordinate frame it belongs to.
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "robot/chassis/lidar";

  // The grid's geometry.
  msg.info.resolution = resolution_;
  msg.info.width  = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;   // no rotation; all-zero is invalid

  msg.data = grid_;

  costmap_pub_->publish(msg);

  
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // clears the grid after every scan, so that old readings don't persist forever.
  std::fill(grid_.begin(), grid_.end(), 0);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];

    // Foxglove showed 8 infinite values per scan — beams that hit nothing.
    // inf * cos(angle) is inf, and the grid index becomes garbage.
    if (!std::isfinite(range)) continue;
    if (range < scan->range_min || range > scan->range_max) continue;

    // Which direction was beam i pointing?
    double angle = scan->angle_min + i * scan->angle_increment;

    // Polar -> Cartesian, in metres, robot at (0,0).
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Metres -> grid cell. static_cast<int> truncates: 15.7 -> 15,
    // which is right, since metre 15.7 falls inside cell 15.
    int gx = static_cast<int>((x - origin_x_) / resolution_);
    int gy = static_cast<int>((y - origin_y_) / resolution_);

    // range_max is 20m but the grid only reaches 15m from centre, so
    // readings CAN land outside. Writing past the end of a vector is
    // undefined behaviour — crash, or silent corruption.
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) continue;

    grid_[gy * width_ + gx] = 100;   // 100 = definitely occupied
  }
    // Inflate: fade cost outward from every occupied cell.
  int cells = static_cast<int>(inflation_radius_ / resolution_);   // 1.0/0.1 = 10

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y * width_ + x] != max_cost_) continue;   // only real obstacles

      // Walk the square around this obstacle.
      for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
          int nx = x + dx;
          int ny = y + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

          double dist = resolution_ * std::sqrt(dx * dx + dy * dy);
          if (dist > inflation_radius_) continue;   // square loop, circular radius

          int cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));

          // Never lower a cell someone else already set higher.
          if (cost > grid_[ny * width_ + nx]) {
            grid_[ny * width_ + nx] = static_cast<int8_t>(cost);
          }
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}