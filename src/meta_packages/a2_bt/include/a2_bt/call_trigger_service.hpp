#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace a2_bt
{

/**
 * Calls any std_srvs/Trigger service and returns SUCCESS when it reports
 * success=true, FAILURE otherwise.
 *
 * Input ports:
 *   service_name (string) : Fully-qualified service name (e.g. "dlio/reset")
 */
class CallTriggerService : public BT::StatefulActionNode
{
public:
  CallTriggerService(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using SrvT = std_srvs::srv::Trigger;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SrvT>::SharedPtr client_;
  rclcpp::Client<SrvT>::SharedFuture future_;
};

}  // namespace a2_bt
