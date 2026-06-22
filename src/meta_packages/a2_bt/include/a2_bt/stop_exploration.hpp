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
 * Publishes Bool(false) to the TARE start/stop topic, signalling TARE to stop
 * publishing waypoints immediately. TARE's PublishWaypoint() is guarded by the
 * stop_exploration_ flag set by this message.
 *
 * Input ports:
 *   start_topic  (string, default: /start_exploration)
 */
class StopExploration : public BT::SyncActionNode
{
public:
  StopExploration(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace a2_bt
