#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace a2_bt
{

/**
 * Async BT node that captures the current robot pose and writes it to the blackboard.
 *
 * Subscribes to pose_topic. Accepts either:
 *   - nav_msgs/Odometry  (RESPLE: /state_estimation)
 *   - geometry_msgs/PoseStamped (DLIO: dlio/odom_node/pose)
 *
 * Default topic is /state_estimation (RESPLE). Override via the pose_topic port.
 *
 * Input ports:
 *   pose_topic  (string) : odometry topic (default: /state_estimation)
 *   timeout_sec (double) : Max wait [s] for first message (default: 5.0)
 *
 * Output ports:
 *   pose (PoseStamped) : Captured pose written to the blackboard
 */
class SavePose : public BT::StatefulActionNode
{
public:
  SavePose(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  std::atomic<bool> received_{false};

  std::chrono::steady_clock::time_point start_time_;
  double timeout_sec_{5.0};
};

}  // namespace a2_bt
