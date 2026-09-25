#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    void publishMap();

  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // The global map. COARSER than the costmap's 0.1 m, per the wiki:
    // a finer costmap fused into a coarser map avoids holes.
    double resolution_ = 0.2;
    int width_  = 300;          // 300 * 0.2 = 60 m across
    int height_ = 300;
    double origin_x_ = -30.0;
    double origin_y_ = -30.0;
    std::vector<int8_t> map_;

    // Latest costmap, held until the timer decides to fuse it.
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_received_ = false;

    // Robot pose, from odometry.
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    // Only fuse once the robot has moved this far.
    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;
    double update_distance_ = 1.5;
    bool first_update_done_ = false;
};

#endif