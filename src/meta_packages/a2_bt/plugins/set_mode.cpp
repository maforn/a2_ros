#include "a2_bt/set_mode.hpp"

namespace a2_bt
{

SetMode::SetMode(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList SetMode::providedPorts()
{
  return {
    BT::InputPort<uint8_t>("mode",        0,   "Target FSM mode (1=STAND_DOWN 2=STAND_UP 3=BALANCE_STAND 4=VELOCITY_MOVE)"),
    BT::InputPort<double>("wait_for_sec", 0.0, "Seconds to wait after acceptance for physical motion"),
  };
}

BT::NodeStatus SetMode::onStart()
{
  mode_req_     = getInput<uint8_t>("mode").value_or(0);
  wait_for_sec_ = getInput<double>("wait_for_sec").value_or(0.0);
  phase_        = Phase::CALLING;

  client_ = node_->create_client<SrvT>("/a2/set_mode");

  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "[SetMode] /a2/set_mode service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req  = std::make_shared<SrvT::Request>();
  req->mode = mode_req_;

  RCLCPP_INFO(node_->get_logger(), "[SetMode] Requesting mode %u", static_cast<unsigned>(mode_req_));
  future_ = client_->async_send_request(req);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetMode::onRunning()
{
  if (phase_ == Phase::CALLING) {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    const auto resp = future_.get();
    client_.reset();

    if (!resp->success) {
      // Already at or past the requested mode — skip this step.
      if (resp->current_mode >= mode_req_) {
        RCLCPP_INFO(node_->get_logger(),
          "[SetMode] Already at mode %u (requested %u) — skipping",
          static_cast<unsigned>(resp->current_mode),
          static_cast<unsigned>(mode_req_));
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_ERROR(node_->get_logger(), "[SetMode] Rejected: %s", resp->message.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "[SetMode] Accepted: %s", resp->message.c_str());

    if (wait_for_sec_ <= 0.0) {
      return BT::NodeStatus::SUCCESS;
    }

    motion_start_ = std::chrono::steady_clock::now();
    phase_        = Phase::WAITING_MOTION;
  }

  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - motion_start_).count();

  if (elapsed >= wait_for_sec_) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void SetMode::onHalted()
{
  client_.reset();
}

}  // namespace a2_bt
