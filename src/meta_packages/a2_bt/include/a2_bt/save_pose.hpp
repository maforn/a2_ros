#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace a2_bt
{

/**
 * Async BT node that captures the current robot pose from DLIO and writes it
 * to the blackboard.
 *
 * Subscribes to pose_topic (geometry_msgs/PoseStamped published by DLIO in
 * the map frame), stores the first message received, and returns SUCCESS.
 * Returns FAILURE if no message arrives within timeout_sec.
 *
 * Input ports:
 *   pose_topic  (string) : DLIO map-frame pose topic (default: dlio/odom_node/pose)
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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  std::atomic<bool> received_{false};

  std::chrono::steady_clock::time_point start_time_;
  double timeout_sec_{5.0};
};

}  // namespace a2_bt
