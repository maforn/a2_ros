#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

namespace a2_bt
{

/**
 * Publishes std_msgs/Int8(1) to /stop to immediately zero pathFollower velocity.
 * Returns SUCCESS after one publish. Use ResumeMovement (data=0) to re-enable.
 *
 * Input ports:
 *   stop_topic  (string, default: /stop)   pathFollower stop topic
 *   data        (int,    default: 1)        1 = stop, 0 = resume
 */
class StopMovement : public BT::SyncActionNode
{
public:
  StopMovement(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace a2_bt
