#include "a2_bt/offset_pose.hpp"

#include <cmath>

namespace a2_bt
{

OffsetPose::OffsetPose(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{}

BT::PortsList OffsetPose::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose",
      "Pose to offset from (PoseStamped from blackboard)"),
    BT::InputPort<double>("distance_x", 1.0,
      "Distance along robot forward axis [m]"),
    BT::InputPort<double>("distance_y", 0.0,
      "Distance along robot left axis [m]"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose",
      "Resulting offset pose written to blackboard"),
  };
}

BT::NodeStatus OffsetPose::tick()
{
  const auto input = getInput<geometry_msgs::msg::PoseStamped>("input_pose");
  if (!input) {
    return BT::NodeStatus::FAILURE;
  }
  const double dx = getInput<double>("distance_x").value_or(1.0);
  const double dy = getInput<double>("distance_y").value_or(0.0);

  const auto & q = input->pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  geometry_msgs::msg::PoseStamped out = *input;
  out.pose.position.x += dx * cos_yaw - dy * sin_yaw;
  out.pose.position.y += dx * sin_yaw + dy * cos_yaw;

  setOutput("output_pose", out);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace a2_bt
