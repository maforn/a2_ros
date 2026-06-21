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
    BT::InputPort<std::string>("topic",        "/cmd_vel", "Twist publish topic"),
    BT::InputPort<double>("linear_x",   0.0,  "Linear velocity x [m/s]"),
    BT::InputPort<double>("angular_z",  0.0,  "Angular velocity z [rad/s]"),
    BT::InputPort<double>("duration_sec", 1.0, "How long to publish [s]"),
  };
}

BT::NodeStatus PublishTwist::onStart()
{
  const auto topic = getInput<std::string>("topic").value_or("/cmd_vel");
  duration_sec_ = getInput<double>("duration_sec").value_or(1.0);

  pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic, 1);
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(),
    "[PublishTwist] Publishing to %s for %.1f s (lin=%.2f ang=%.2f)",
    topic.c_str(), duration_sec_,
    getInput<double>("linear_x").value_or(0.0),
    getInput<double>("angular_z").value_or(0.0));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishTwist::onRunning()
{
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (elapsed >= duration_sec_) {
    pub_->publish(geometry_msgs::msg::Twist{});
    pub_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist msg;
  msg.linear.x  = getInput<double>("linear_x").value_or(0.0);
  msg.angular.z = getInput<double>("angular_z").value_or(0.0);
  pub_->publish(msg);

  return BT::NodeStatus::RUNNING;
}

void PublishTwist::onHalted()
{
  if (pub_) {
    pub_->publish(geometry_msgs::msg::Twist{});
    pub_.reset();
  }
}

}  // namespace a2_bt
