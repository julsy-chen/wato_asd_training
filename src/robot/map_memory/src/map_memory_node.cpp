
#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

// run at startup
MapMemoryNode::MapMemoryNode()
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  map_.assign(width_ * height_, -1);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&MapMemoryNode::updateMap, this));

  RCLCPP_INFO(this->get_logger(), "map_memory node started, publishing /map");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;     // the * copies the message out of the pointer
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // (x, y, z, w))
  const auto & q = msg->pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

// ── TIMER: decide whether to fuse, then fuse ─────────────────────────────
void MapMemoryNode::updateMap() {
  if (!costmap_received_) return;      // nothing to work with yet

  // Rate limit: skip unless the robot has moved far enough.
  // The FIRST fusion is forced so the planner has a map immediately.
  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  double moved = std::sqrt(dx * dx + dy * dy);

  if (first_update_done_ && moved < update_distance_) {
    publishMap();                      // republish unchanged
    return;
  }

  const auto & cm = latest_costmap_;
  double c_res = cm.info.resolution;
  int c_w = cm.info.width;
  int c_h = cm.info.height;
  double c_ox = cm.info.origin.position.x;
  double c_oy = cm.info.origin.position.y;

  double cy = std::cos(robot_yaw_);
  double sy = std::sin(robot_yaw_);

  for (int y = 0; y < c_h; ++y) {
    for (int x = 0; x < c_w; ++x) {
      int8_t v = cm.data[y * c_w + x];
      if (v < 0)  continue;   // unknown in the costmap: nothing to contribute
      if (v == 0) continue;   // free space: must NOT erase known walls

      // 1. costmap cell -> metres in the ROBOT's frame, using the cell CENTRE
      double rx = c_ox + (x + 0.5) * c_res;
      double ry = c_oy + (y + 0.5) * c_res;

      // 2 & 3. rotate by the robot's heading, then translate to its position
      double wx = robot_x_ + rx * cy - ry * sy;
      double wy = robot_y_ + rx * sy + ry * cy;

      // 4. world metres -> global map cell
      int gx = static_cast<int>((wx - origin_x_) / resolution_);
      int gy = static_cast<int>((wy - origin_y_) / resolution_);
      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) continue;

      // Keep the highest cost ever seen: a wall found once stays a wall.
      int idx = gy * width_ + gx;
      if (v > map_[idx]) map_[idx] = v;
    }
  }

  last_update_x_ = robot_x_;
  last_update_y_ = robot_y_;
  first_update_done_ = true;
  publishMap();
}

// ── HELPER: wrap the grid in a message and send it ───────────────────────
void MapMemoryNode::publishMap() {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = this->get_clock()->now();

  // A FIXED frame, not the lidar's. This is the whole point of the node:
  // the costmap rotates with the robot, the map must not.
  msg.header.frame_id = "sim_world";

  msg.info.resolution = resolution_;
  msg.info.width  = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data = map_;
  map_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}