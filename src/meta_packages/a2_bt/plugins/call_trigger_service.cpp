#include "a2_bt/call_trigger_service.hpp"

namespace a2_bt
{

CallTriggerService::CallTriggerService(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList CallTriggerService::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name", "", "Fully-qualified service name to call"),
  };
}

BT::NodeStatus CallTriggerService::onStart()
{
  const auto srv_name = getInput<std::string>("service_name").value_or("");
  if (srv_name.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[CallTriggerService] service_name port is empty");
    return BT::NodeStatus::FAILURE;
  }

  client_ = node_->create_client<SrvT>(srv_name);

  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[CallTriggerService] Service not available: %s", srv_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[CallTriggerService] Calling %s", srv_name.c_str());
  future_ = client_->async_send_request(std::make_shared<SrvT::Request>());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallTriggerService::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  const auto resp = future_.get();
  client_.reset();

  if (!resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
      "[CallTriggerService] Failed: %s", resp->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[CallTriggerService] OK: %s", resp->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

void CallTriggerService::onHalted()
{
  client_.reset();
}

}  // namespace a2_bt
