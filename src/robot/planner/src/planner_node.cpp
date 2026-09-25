#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

#include "planner_node.hpp"

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

PlannerNode::PlannerNode()
: Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "planner node started");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) planPath();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "new goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;
  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "goal reached");
    state_ = State::WAITING_FOR_GOAL;
    return;
  }
  planPath();  // periodic replan in case the map changed
}

bool PlannerNode::goalReached() {
  if (!robot_odom_) return false;
  double dx = goal_.point.x - robot_odom_->pose.pose.position.x;
  double dy = goal_.point.y - robot_odom_->pose.pose.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

bool PlannerNode::worldToGrid(double wx, double wy, CellIndex &out) {
  if (!current_map_) return false;
  const auto &info = current_map_->info;
  int col = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
  int row = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
  if (col < 0 || row < 0 || col >= (int)info.width || row >= (int)info.height) return false;
  out = CellIndex(col, row);
  return true;
}

geometry_msgs::msg::Point PlannerNode::gridToWorld(const CellIndex &c) {
  const auto &info = current_map_->info;
  geometry_msgs::msg::Point p;
  p.x = info.origin.position.x + (c.x + 0.5) * info.resolution;
  p.y = info.origin.position.y + (c.y + 0.5) * info.resolution;
  return p;
}

bool PlannerNode::isFree(const CellIndex &c) {
  const auto &info = current_map_->info;
  int8_t val = current_map_->data[c.y * (int)info.width + c.x];
  return val < occupancy_threshold_;  // unknown (-1) counts as passable
}

void PlannerNode::planPath() {
  if (!current_map_ || !goal_received_ || !robot_odom_) return;

  CellIndex start, goal_cell;
  if (!worldToGrid(robot_odom_->pose.pose.position.x, robot_odom_->pose.pose.position.y, start)) {
    RCLCPP_WARN(this->get_logger(), "robot is outside the map"); return;
  }
  if (!worldToGrid(goal_.point.x, goal_.point.y, goal_cell)) {
    RCLCPP_WARN(this->get_logger(), "goal is outside the map"); return;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  auto heuristic = [](const CellIndex &a, const CellIndex &b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal_cell));
  static const int offsets[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  bool found = false;

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();
    if (closed[current]) continue;
    closed[current] = true;
    if (current == goal_cell) { found = true; break; }

    for (const auto &off : offsets) {
      CellIndex neighbor(current.x + off[0], current.y + off[1]);
      if (neighbor.x < 0 || neighbor.y < 0 ||
          neighbor.x >= (int)current_map_->info.width ||
          neighbor.y >= (int)current_map_->info.height) continue;
      if (!isFree(neighbor)) continue;

      double step = (off[0] && off[1]) ? std::sqrt(2.0) : 1.0;
      double tentative_g = g_score[current] + step;
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        open_set.emplace(neighbor, tentative_g + heuristic(neighbor, goal_cell));
      }
    }
  }

  if (!found) { RCLCPP_WARN(this->get_logger(), "no path found to goal"); return; }

  std::vector<CellIndex> cells;
  CellIndex c = goal_cell;
  while (!(c == start)) { cells.push_back(c); c = came_from[c]; }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = current_map_->header.frame_id;
  for (const auto &cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = gridToWorld(cell);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}