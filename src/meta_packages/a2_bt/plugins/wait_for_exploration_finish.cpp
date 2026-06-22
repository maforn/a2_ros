#include "a2_bt/wait_for_exploration_finish.hpp"

#include <sstream>
#include <iomanip>

namespace a2_bt
{

WaitForExplorationFinish::WaitForExplorationFinish(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList WaitForExplorationFinish::providedPorts()
{
  return {
    BT::InputPort<std::string>("finish_topic", "exploration_finish", "TARE exploration finish topic"),
    BT::InputPort<std::string>("status_topic", "bt/exploration_status", "Periodic status string topic"),
    BT::InputPort<double>("timeout_sec", -1.0, "Max wait [s]; <= 0 disables timeout"),
  };
}

BT::NodeStatus WaitForExplorationFinish::onStart()
{
  finished_ = false;
  timeout_sec_ = getInput<double>("timeout_sec").value_or(-1.0);
  start_time_ = std::chrono::steady_clock::now();
  last_status_time_ = start_time_;

  const auto finish_topic = getInput<std::string>("finish_topic").value_or("exploration_finish");
  const auto status_topic = getInput<std::string>("status_topic").value_or("bt/exploration_status");

  finish_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    finish_topic, 5,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) {
        finished_ = true;
      }
    });

  status_pub_ = node_->create_publisher<std_msgs::msg::String>(status_topic, 1);

  RCLCPP_INFO(node_->get_logger(),
    "[WaitForExplorationFinish] Waiting on %s, status → %s (timeout=%.1f s)",
    finish_topic.c_str(), status_topic.c_str(), timeout_sec_);

  publishStatus("EXPLORING", 0.0);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForExplorationFinish::onRunning()
{
  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - start_time_).count();

  if (finished_.load()) {
    publishStatus("FINISHED", elapsed);
    RCLCPP_INFO(node_->get_logger(),
      "[WaitForExplorationFinish] Exploration finished after %.1f s — SUCCESS", elapsed);
    finish_sub_.reset();
    status_pub_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_sec_ > 0.0 && elapsed > timeout_sec_) {
    publishStatus("TIMEOUT", elapsed);
    RCLCPP_WARN(node_->get_logger(),
      "[WaitForExplorationFinish] Timeout (%.1f s) — FAILURE", timeout_sec_);
    finish_sub_.reset();
    status_pub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  // Throttle status publishing to ~1 every kStatusIntervalSec
  const double since_last = std::chrono::duration<double>(now - last_status_time_).count();
  if (since_last >= kStatusIntervalSec) {
    publishStatus("EXPLORING", elapsed);
    last_status_time_ = now;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForExplorationFinish::onHalted()
{
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();
  publishStatus("HALTED", elapsed);
  RCLCPP_WARN(node_->get_logger(), "[WaitForExplorationFinish] Halted after %.1f s", elapsed);
  finish_sub_.reset();
  status_pub_.reset();
  finished_ = false;
}

void WaitForExplorationFinish::publishStatus(const std::string & state, double elapsed_sec)
{
  if (!status_pub_) return;

  std::ostringstream ss;
  ss << state << " | elapsed=" << std::fixed << std::setprecision(1) << elapsed_sec << "s";
  if (timeout_sec_ > 0.0 && state == "EXPLORING") {
    ss << " | timeout=" << timeout_sec_ << "s";
  }

  std_msgs::msg::String msg;
  msg.data = ss.str();
  status_pub_->publish(msg);
}

}  // namespace a2_bt
