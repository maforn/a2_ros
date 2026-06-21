#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace a2_bt
{

/**
 * Async BT node that sends a goal to FAR Planner via /goal_point and
 * monitors /far_reach_goal_status for completion.
 *
 * onStart()   — publishes goal; returns RUNNING
 * onRunning() — returns SUCCESS when /far_reach_goal_status is true,
 *               FAILURE on timeout, else RUNNING
 * onHalted()  — cleans up subscription
 *
 * Input ports:
 *   goal_pose    (PoseStamped) : Goal from blackboard (x/y extracted; takes priority)
 *   goal_x       (double)      : Goal X [m]
 *   goal_y       (double)      : Goal Y [m]
 *   frame_id     (string)      : Reference frame (default: map)
 *   goal_topic   (string)      : Goal topic (default: /goal_point)
 *   timeout_sec  (double)      : Max wait time before FAILURE (default: 60.0)
 */
class NavigateToPoseAction : public BT::StatefulActionNode
{
public:
  NavigateToPoseAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;

  std::atomic<bool> reached_{false};
  std::chrono::steady_clock::time_point start_time_;
  double timeout_sec_{60.0};
};

}  // namespace a2_bt
