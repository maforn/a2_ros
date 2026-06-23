#include "a2_bt/call_empty_service.hpp"

namespace a2_bt
{

CallEmptyService::CallEmptyService(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList CallEmptyService::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name", "", "Fully-qualified service name to call"),
    BT::InputPort<double>("timeout_sec", 10.0, "Max wait for the response [s]; <= 0 disables the timeout"),
  };
}

BT::NodeStatus CallEmptyService::onStart()
{
  const auto srv_name = getInput<std::string>("service_name").value_or("");
  if (srv_name.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[CallEmptyService] service_name port is empty");
    return BT::NodeStatus::FAILURE;
  }
  timeout_sec_ = getInput<double>("timeout_sec").value_or(10.0);

  client_ = node_->create_client<SrvT>(srv_name);

  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[CallEmptyService] Service not available: %s", srv_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[CallEmptyService] Calling %s", srv_name.c_str());
  future_ = client_->async_send_request(std::make_shared<SrvT::Request>());
  start_time_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallEmptyService::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    future_.get();
    client_.reset();

    RCLCPP_INFO(node_->get_logger(), "[CallEmptyService] Done");
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  if (timeout_sec_ > 0.0 && elapsed > timeout_sec_) {
    RCLCPP_ERROR(node_->get_logger(),
      "[CallEmptyService] Timed out after %.1f s waiting for response", elapsed);
    client_.reset();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void CallEmptyService::onHalted()
{
  client_.reset();
}

}  // namespace a2_bt
