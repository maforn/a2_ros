#include "a2_bt/save_map.hpp"

namespace a2_bt
{

SaveMap::SaveMap(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList SaveMap::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name", "/save_pcd", "Fully-qualified save_pcd service name"),
    BT::InputPort<std::string>("save_path", "", "Output directory; empty uses DLIO's own default"),
    BT::InputPort<double>("leaf_size", -1.0, "Voxel leaf size; <=0 uses DLIO's own default"),
  };
}

BT::NodeStatus SaveMap::onStart()
{
  const auto srv_name = getInput<std::string>("service_name").value_or("/save_pcd");
  const auto save_path = getInput<std::string>("save_path").value_or("");
  const auto leaf_size = getInput<double>("leaf_size").value_or(-1.0);

  client_ = node_->create_client<SrvT>(srv_name);

  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[SaveMap] Service not available: %s", srv_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<SrvT::Request>();
  request->save_path = save_path;
  request->leaf_size = static_cast<float>(leaf_size);

  RCLCPP_INFO(node_->get_logger(),
    "[SaveMap] Calling %s (save_path='%s')", srv_name.c_str(), save_path.c_str());
  future_ = client_->async_send_request(request);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SaveMap::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  const auto resp = future_.get();
  client_.reset();

  if (!resp->success) {
    RCLCPP_ERROR(node_->get_logger(), "[SaveMap] DLIO reported failure saving map");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[SaveMap] Map saved successfully");
  return BT::NodeStatus::SUCCESS;
}

void SaveMap::onHalted()
{
  client_.reset();
}

}  // namespace a2_bt
