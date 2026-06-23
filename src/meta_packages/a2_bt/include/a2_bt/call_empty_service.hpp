#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace a2_bt
{

/**
 * Calls any std_srvs/Empty service and returns SUCCESS once it responds.
 * Used e.g. to trigger resple's "save_map" / "save_map_node" services.
 *
 * Input ports:
 *   service_name (string) : Fully-qualified service name (e.g. "save_map")
 *   timeout_sec  (double) : Max wait for the response [s]; <= 0 disables
 *                           the timeout (default 10.0). Without this, a
 *                           server that never replies hangs the node (and
 *                           the whole tree branch) forever.
 */
class CallEmptyService : public BT::StatefulActionNode
{
public:
  CallEmptyService(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using SrvT = std_srvs::srv::Empty;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SrvT>::SharedPtr client_;
  rclcpp::Client<SrvT>::SharedFuture future_;
  double timeout_sec_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace a2_bt
