#pragma once

#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

struct CellIndex {
  int x, y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &o) const { return x == o.x && y == o.y; }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    bool goalReached();
    void planPath();
    bool worldToGrid(double wx, double wy, CellIndex &out);
    geometry_msgs::msg::Point gridToWorld(const CellIndex &c);
    bool isFree(const CellIndex &c);

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    geometry_msgs::msg::PointStamped goal_;
    bool goal_received_ = false;
    State state_ = State::WAITING_FOR_GOAL;

    double goal_tolerance_ = 0.5;
    int occupancy_threshold_ = 1.0;  // cells at/above this are treated as blocked
};

#endif