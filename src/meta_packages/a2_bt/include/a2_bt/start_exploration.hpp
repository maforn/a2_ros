#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace a2_bt
{

/**
 * One-shot BT node that triggers TARE autonomous exploration.
 *
 * Publishes std_msgs/Bool(true) to the start topic and returns SUCCESS
 * immediately. Requires kAutoStart: false in tare_a2.yaml.
 *
 * Input ports:
 *   start_topic (string) : Topic to publish to (default: /start_exploration)
 */
class StartExploration : public BT::StatefulActionNode
{
public:
  StartExploration(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace a2_bt
