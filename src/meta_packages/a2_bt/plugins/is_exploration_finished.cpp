#include "a2_bt/is_exploration_finished.hpp"

namespace a2_bt
{

IsExplorationFinished::IsExplorationFinished(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList IsExplorationFinished::providedPorts()
{
  return {
    BT::InputPort<std::string>("finish_topic", "exploration_finish", "TARE exploration finish topic"),
  };
}

BT::NodeStatus IsExplorationFinished::tick()
{
  // Lazy-init: create subscription on first tick so the input port is resolved
  if (!finish_sub_) {
    const auto topic = getInput<std::string>("finish_topic").value_or("exploration_finish");
    finish_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic, 5,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          finished_ = true;
        }
      });
  }

  return finished_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace a2_bt
