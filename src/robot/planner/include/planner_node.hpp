#pragma once

#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <vector>
#include <unordered_map>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

// ── Supporting structures (from the assignment wiki) ────────────────────
struct CellIndex
{
  int x;
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }
  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

  private:
    void planPath();
    bool goalReached();

    // A* helpers
    bool worldToGrid(double wx, double wy, CellIndex &out) const;
    void gridToWorld(const CellIndex &c, double &wx, double &wy) const;
    bool isTraversable(const CellIndex &c) const;
    std::vector<CellIndex> aStar(const CellIndex &start, const CellIndex &goal);

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    bool goal_active_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    double goal_tolerance_ = 0.5;              // metres
    int obstacle_threshold_ = 50;              // cells >= this are blocked
    rclcpp::Time goal_start_time_;
    double timeout_sec_ = 30.0;
};

#endif