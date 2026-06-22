#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace a2_bt
{

/**
 * Publishes a Twist to a configurable topic for a fixed duration, sends a
 * zero-velocity stop, then waits wait_after_sec before returning SUCCESS.
 *
 * Input ports:
 *   topic          (string) : Publish topic (default: /cmd_vel)
 *   linear_x       (double) : Linear velocity x [m/s]
 *   angular_z      (double) : Angular velocity z [rad/s]
 *   duration_sec   (double) : How long to publish [s]
 *   wait_after_sec (double) : Settling time after stop before SUCCESS [s]
 */
class PublishTwist : public BT::StatefulActionNode
{
public:
  PublishTwist(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { PUBLISHING, WAITING };

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  std::chrono::steady_clock::time_point start_time_;
  double duration_sec_{1.0};
  double wait_after_sec_{0.0};
  Phase phase_{Phase::PUBLISHING};
};

}  // namespace a2_bt
