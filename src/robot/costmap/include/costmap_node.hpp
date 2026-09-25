#pragma once
#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    robot::CostmapCore costmap_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Grid geometry: 300 x 300 cells at 0.1 m = 30 m x 30 m of world.
    // Origin at (-15, -15) puts the robot at (0,0) in the middle.
    double resolution_ = 0.1;
    int width_  = 300;
    int height_ = 300;
    double origin_x_ = -15.0;
    double origin_y_ = -15.0;

    double inflation_radius_ = 1.0;   // metres
    int max_cost_ = 100;

    std::vector<int8_t> grid_;

};
 
#endif 