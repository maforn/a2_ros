#include "a2_bt/stop_movement.hpp"

namespace a2_bt
{

StopMovement::StopMovement(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList StopMovement::providedPorts()
{
  return {
    BT::InputPort<std::string>("stop_topic", "/stop", "pathFollower stop topic"),
    BT::InputPort<int>("data", 1, "1 = stop, 0 = resume"),
  };
}

BT::NodeStatus StopMovement::tick()
{
  const auto topic = getInput<std::string>("stop_topic").value_or("/stop");
  const int  data  = getInput<int>("data").value_or(1);

  auto pub = node_->create_publisher<std_msgs::msg::Int8>(topic, rclcpp::QoS(1));
  std_msgs::msg::Int8 msg;
  msg.data = static_cast<int8_t>(data);
  pub->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
    "[StopMovement] /stop ← %d (%s)", data, data ? "STOP" : "RESUME");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace a2_bt
