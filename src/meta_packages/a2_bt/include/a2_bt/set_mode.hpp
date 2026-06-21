#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "a2_interfaces/srv/set_operating_mode.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace a2_bt
{

/**
 * Calls /a2/set_mode and waits for the FSM to accept the transition.
 * After acceptance, optionally waits `wait_for_sec` additional seconds
 * so the robot has time to physically complete the motion.
 *
 * Input ports:
 *   mode          (uint8)  : Target FSM mode
 *                            1=STAND_DOWN  2=STAND_UP  3=BALANCE_STAND  4=VELOCITY_MOVE
 *   wait_for_sec  (double) : Seconds to wait after acceptance (default: 0)
 */
class SetMode : public BT::StatefulActionNode
{
public:
  SetMode(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using SrvT = a2_interfaces::srv::SetOperatingMode;

  enum class Phase { CALLING, WAITING_MOTION };

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SrvT>::SharedPtr client_;
  rclcpp::Client<SrvT>::SharedFuture future_;

  Phase phase_{Phase::CALLING};
  std::chrono::steady_clock::time_point motion_start_;
  double wait_for_sec_{0.0};
  uint8_t mode_req_{0};
};

}  // namespace a2_bt
