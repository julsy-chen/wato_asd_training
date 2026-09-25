#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

// ── SETUP ────────────────────────────────────────────────────────────────
ControlNode::ControlNode()
: Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 10 Hz — smooth, and fast enough to react as the robot moves.
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "control node started, publishing /cmd_vel");
}

// ── MAIN LOOP ────────────────────────────────────────────────────────────
void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) return;
  if (current_path_->poses.empty()) return;

  // Have we arrived? Measure against the LAST point on the path.
  const auto &final_pose = current_path_->poses.back();
  double dist_to_goal = computeDistance(
    robot_odom_->pose.pose.position, final_pose.pose.position);

  if (dist_to_goal < goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());   // all zeros = stop
    return;
  }

  auto lookahead = findLookaheadPoint();
  if (!lookahead) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());   // nothing to aim at
    return;
  }

  cmd_vel_pub_->publish(computeVelocity(*lookahead));
}

// ── Find the first path point at least lookahead_distance_ away ─────────
std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  const auto &pos = robot_odom_->pose.pose.position;
  for (const auto &pose : current_path_->poses) {
    if (computeDistance(pos, pose.pose.position) >= lookahead_distance_) {
      return pose;
    }
  }

  // Every point is closer than the lookahead distance, which means we're
  // near the end. Aim at the final point so the robot finishes the path.
  if (!current_path_->poses.empty()) {
    return current_path_->poses.back();
  }
  return std::nullopt;
}

// ── Turn a target point into forward + turning speed ────────────────────
geometry_msgs::msg::Twist ControlNode::computeVelocity(
  const geometry_msgs::msg::PoseStamped &target)
{
  geometry_msgs::msg::Twist cmd;

  const auto &pos = robot_odom_->pose.pose.position;
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  // Direction from the robot to the target, in WORLD coordinates.
  double dx = target.pose.position.x - pos.x;
  double dy = target.pose.position.y - pos.y;
  double target_angle = std::atan2(dy, dx);

  // How far the robot must turn. Normalise into [-pi, pi] so that turning
  // 350 degrees left becomes 10 degrees right — otherwise the robot takes
  // the long way around.
  double alpha = target_angle - robot_yaw;
  while (alpha >  M_PI) alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;

  // Pure pursuit curvature: k = 2*sin(alpha) / L
  // This is the curvature of the circular arc from the robot's current
  // position and heading through the lookahead point.
  double curvature = 2.0 * std::sin(alpha) / lookahead_distance_;

  cmd.linear.x = linear_speed_; // forward speed in m/s
  cmd.angular.z = curvature * linear_speed_; // turn rate in rad/s w/ positive direction being left

  // Cap the turn rate so sharp corners don't produce absurd spin.
  if (cmd.angular.z >  max_angular_speed_) cmd.angular.z =  max_angular_speed_;
  if (cmd.angular.z < -max_angular_speed_) cmd.angular.z = -max_angular_speed_;

  // If the target is far off to the side, slow down and turn more first.
  if (std::fabs(alpha) > 1.0) cmd.linear.x = linear_speed_ * 0.3;

  return cmd;
}
double ControlNode::computeDistance(const geometry_msgs::msg::Point &a,
                                    const geometry_msgs::msg::Point &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

// Same quaternion -> yaw extraction as map_memory.
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}