#include "a2_bt/publish_twist.hpp"

namespace a2_bt
{

PublishTwist::PublishTwist(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList PublishTwist::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic",          "/cmd_vel", "TwistStamped publish topic"),
    BT::InputPort<double>("linear_x",     0.0,  "Linear velocity x [m/s]"),
    BT::InputPort<double>("angular_z",    0.0,  "Angular velocity z [rad/s]"),
    BT::InputPort<double>("duration_sec", 1.0,  "How long to publish [s]"),
    BT::InputPort<double>("wait_after_sec", 0.0, "Settling time after stop before SUCCESS [s]"),
  };
}

BT::NodeStatus PublishTwist::onStart()
{
  const auto topic  = getInput<std::string>("topic").value_or("/cmd_vel");
  duration_sec_     = getInput<double>("duration_sec").value_or(1.0);
  wait_after_sec_   = getInput<double>("wait_after_sec").value_or(0.0);
  phase_            = Phase::PUBLISHING;

  pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic, 1);
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(),
    "[PublishTwist] Publishing to %s for %.1f s then %.1f s settle (lin=%.2f ang=%.2f)",
    topic.c_str(), duration_sec_, wait_after_sec_,
    getInput<double>("linear_x").value_or(0.0),
    getInput<double>("angular_z").value_or(0.0));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishTwist::onRunning()
{
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (phase_ == Phase::PUBLISHING) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = node_->now();

    if (elapsed >= duration_sec_) {
      pub_->publish(msg);  // zero twist to stop
      pub_.reset();

      if (wait_after_sec_ <= 0.0) {
        return BT::NodeStatus::SUCCESS;
      }

      start_time_ = std::chrono::steady_clock::now();
      phase_ = Phase::WAITING;
      return BT::NodeStatus::RUNNING;
    }

    msg.twist.linear.x  = getInput<double>("linear_x").value_or(0.0);
    msg.twist.angular.z = getInput<double>("angular_z").value_or(0.0);
    pub_->publish(msg);
    return BT::NodeStatus::RUNNING;
  }

  // Phase::WAITING — settling time after stop
  const double wait_elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (wait_elapsed >= wait_after_sec_) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void PublishTwist::onHalted()
{
  if (pub_) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = node_->now();
    pub_->publish(stop);
    pub_.reset();
  }
}

}  // namespace a2_bt
