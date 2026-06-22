#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace a2_bt
{

/**
 * Condition node: returns SUCCESS if TARE has published exploration_finish=true,
 * FAILURE otherwise. Re-evaluated every BT tick.
 *
 * Intended for use inside ReactiveSequence / ReactiveFallback so that other
 * actions can be preempted as soon as exploration completes.
 *
 * Input ports:
 *   finish_topic (string) : Topic to subscribe to (default: exploration_finish)
 */
class IsExplorationFinished : public BT::ConditionNode
{
public:
  IsExplorationFinished(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_sub_;
  std::atomic<bool> finished_{false};
};

}  // namespace a2_bt
