#include "a2_bt/stop_exploration.hpp"

namespace a2_bt
{

StopExploration::StopExploration(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList StopExploration::providedPorts()
{
  return {
    BT::InputPort<std::string>("start_topic", "/start_exploration", "TARE start/stop topic"),
  };
}

BT::NodeStatus StopExploration::tick()
{
  const auto topic = getInput<std::string>("start_topic").value_or("/start_exploration");

  auto pub = node_->create_publisher<std_msgs::msg::Bool>(topic, 5);
  std_msgs::msg::Bool msg;
  msg.data = false;
  pub->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
    "[StopExploration] Published false → %s", topic.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace a2_bt
