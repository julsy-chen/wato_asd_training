
#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>

#include "planner_node.hpp"

// ── SETUP ────────────────────────────────────────────────────────────────
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
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "planner node started, waiting for a goal");
}

// ── CALLBACKS ────────────────────────────────────────────────────────────

// Store the map. If we're mid-journey, replan: the map just changed and the
// old path may now run through a wall we've since discovered.
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;
  map_received_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

// A new goal arrived (you clicked in Foxglove). Switch states and plan.
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_x_ = msg->point.x;
  goal_y_ = msg->point.y;
  goal_active_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "new goal: (%.2f, %.2f)", goal_x_, goal_y_);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
}

// ── TIMER: watch for arrival or timeout ──────────────────────────────────
void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "goal reached");
    state_ = State::WAITING_FOR_GOAL;
    goal_active_ = false;
    return;
  }

  double elapsed = (this->now() - goal_start_time_).seconds();
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "timed out after %.0f s, replanning", elapsed);
    goal_start_time_ = this->now();
    planPath();
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_x_ - robot_x_;
  double dy = goal_y_ - robot_y_;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

// ── PLANNING: straight line for now, A* comes next ───────────────────────
// ── Convert world metres -> grid cell. Returns false if outside the map. ──
bool PlannerNode::worldToGrid(double wx, double wy, CellIndex &out) const {
  double res = map_.info.resolution;
  int gx = static_cast<int>((wx - map_.info.origin.position.x) / res);
  int gy = static_cast<int>((wy - map_.info.origin.position.y) / res);
  if (gx < 0 || gx >= static_cast<int>(map_.info.width)) return false;
  if (gy < 0 || gy >= static_cast<int>(map_.info.height)) return false;
  out = CellIndex(gx, gy);
  return true;
}

// ── Convert a grid cell back to world metres, at the cell's CENTRE. ──
void PlannerNode::gridToWorld(const CellIndex &c, double &wx, double &wy) const {
  double res = map_.info.resolution;
  wx = map_.info.origin.position.x + (c.x + 0.5) * res;
  wy = map_.info.origin.position.y + (c.y + 0.5) * res;
}

// ── Can the robot occupy this cell? ──
bool PlannerNode::isTraversable(const CellIndex &c) const {
  if (c.x < 0 || c.x >= static_cast<int>(map_.info.width)) return false;
  if (c.y < 0 || c.y >= static_cast<int>(map_.info.height)) return false;

  int8_t v = map_.data[c.y * map_.info.width + c.x];

  // -1 means UNKNOWN. The wiki says to treat it as passable: "Your planner
  // node might plan through objects it can't see. That's okay, as the robot
  // moves, the map will update, and so will the planned path." If unknown
  // were blocked, the robot could never plan anywhere it hadn't been.
  if (v < 0) return true;

  return v < obstacle_threshold_;
}

// ── A* ───────────────────────────────────────────────────────────────────
std::vector<CellIndex> PlannerNode::aStar(const CellIndex &start,
                                          const CellIndex &goal) {
  // OPEN SET: cells we've seen but not yet expanded, ordered so the smallest
  // f_score pops first. CompareF reverses the default max-heap.
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  // g_score[c] = cheapest known cost to reach c from start.
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  // came_from[c] = the cell we arrived from. This is how the path is rebuilt.
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  // CLOSED SET: cells already expanded, so we never process one twice.
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  auto heuristic = [&](const CellIndex &c) {
    double dx = goal.x - c.x;
    double dy = goal.y - c.y;
    return std::sqrt(dx * dx + dy * dy);   // straight-line estimate
  };

  g_score[start] = 0.0;
  open.push(AStarNode(start, heuristic(start)));

  while (!open.empty()) {
    AStarNode current = open.top();
    open.pop();
    CellIndex c = current.index;

    if (closed[c]) continue;    // stale duplicate, already expanded
    closed[c] = true;

    // Reached the goal: walk came_from backwards to rebuild the path.
    if (c == goal) {
      std::vector<CellIndex> path;
      CellIndex step = goal;
      while (!(step == start)) {
        path.push_back(step);
        step = came_from[step];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());   // built backwards, so flip it
      return path;
    }

    // Expand the 8 neighbours (4 orthogonal + 4 diagonal).
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;

        CellIndex n(c.x + dx, c.y + dy);
        if (!isTraversable(n)) continue;
        if (closed[n]) continue;

        // Diagonal steps are sqrt(2) long, not 1. Using 1 for both makes
        // diagonals look artificially cheap and the path comes out wrong.
        double step_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;

        // Prefer routes with clearance: add a penalty proportional to the
        // inflation cost your costmap baked in.
        int8_t v = map_.data[n.y * map_.info.width + n.x];
        if (v > 0) step_cost += v * 0.05;

        double tentative = g_score[c] + step_cost;

        // Only keep this route if it's better than any we've found before.
        auto it = g_score.find(n);
        if (it == g_score.end() || tentative < it->second) {
          g_score[n] = tentative;
          came_from[n] = c;
          open.push(AStarNode(n, tentative + heuristic(n)));
        }
      }
    }
  }

  return {};    // open set exhausted: no route exists
}

// ── PLANNING ─────────────────────────────────────────────────────────────
void PlannerNode::planPath() {
  if (!goal_active_ || !map_received_) {
    RCLCPP_WARN(this->get_logger(), "cannot plan: no goal or no map yet");
    return;
  }

  CellIndex start, goal;
  if (!worldToGrid(robot_x_, robot_y_, start)) {
    RCLCPP_WARN(this->get_logger(), "robot is outside the map");
    return;
  }
  if (!worldToGrid(goal_x_, goal_y_, goal)) {
    RCLCPP_WARN(this->get_logger(), "goal is outside the map");
    return;
  }

  std::vector<CellIndex> cells = aStar(start, goal);

  if (cells.empty()) {
    RCLCPP_WARN(this->get_logger(), "A* found no path to (%.2f, %.2f)",
                goal_x_, goal_y_);
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = map_.header.frame_id;

  for (const auto &c : cells) {
    double wx, wy;
    gridToWorld(c, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "A* path: %zu cells", path.poses.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}