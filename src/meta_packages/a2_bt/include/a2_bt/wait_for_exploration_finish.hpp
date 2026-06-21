#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace a2_bt
{

/**
 * Async BT node that blocks until TARE exploration is complete.
 *
 * Subscribes to exploration_finish and waits for data=true.
 * Publishes a human-readable status string to status_topic at ~1 Hz.
 * Returns FAILURE if timeout_sec elapses before completion.
 * A timeout_sec <= 0 means no timeout.
 *
 * Input ports:
 *   finish_topic (string) : Topic to subscribe to (default: exploration_finish)
 *   status_topic (string) : Topic for periodic status updates (default: bt/exploration_status)
 *   timeout_sec  (double) : Max wait [s]; <= 0 disables timeout (default: -1.0)
 */
class WaitForExplorationFinish : public BT::StatefulActionNode
{
public:
  WaitForExplorationFinish(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publishStatus(const std::string & state, double elapsed_sec);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::atomic<bool> finished_{false};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_status_time_;
  double timeout_sec_{-1.0};

  static constexpr double kStatusIntervalSec = 5.0;
};

}  // namespace a2_bt
