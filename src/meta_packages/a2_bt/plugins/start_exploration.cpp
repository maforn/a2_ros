#include "a2_bt/start_exploration.hpp"

namespace a2_bt
{

StartExploration::StartExploration(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList StartExploration::providedPorts()
{
  return {
    BT::InputPort<std::string>("start_topic", "/start_exploration", "TARE start topic"),
  };
}

BT::NodeStatus StartExploration::onStart()
{
  const auto topic = getInput<std::string>("start_topic").value_or("/start_exploration");

  auto pub = node_->create_publisher<std_msgs::msg::Bool>(topic, 5);
  std_msgs::msg::Bool msg;
  msg.data = true;
  pub->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
    "[StartExploration] Published true → %s", topic.c_str());

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StartExploration::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void StartExploration::onHalted() {}

}  // namespace a2_bt
