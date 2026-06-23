#include "a2_bt/save_pose.hpp"

namespace a2_bt
{

SavePose::SavePose(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList SavePose::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose_topic", "/state_estimation",
      "Odometry topic (nav_msgs/Odometry) — default: RESPLE /state_estimation"),
    BT::InputPort<double>("timeout_sec", 5.0,
      "Max wait [s] for first pose message"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose",
      "Captured pose in map frame, written to blackboard"),
  };
}

BT::NodeStatus SavePose::onStart()
{
  received_ = false;
  timeout_sec_ = getInput<double>("timeout_sec").value_or(5.0);
  start_time_ = std::chrono::steady_clock::now();

  const auto topic = getInput<std::string>("pose_topic").value_or("/state_estimation");

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic, 1,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (!received_.load()) {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        latest_pose_.header = msg->header;
        latest_pose_.pose   = msg->pose.pose;
        received_ = true;
      }
    });

  RCLCPP_INFO(node_->get_logger(),
    "[SavePose] Waiting for odometry on '%s' (timeout=%.1f s)",
    topic.c_str(), timeout_sec_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SavePose::onRunning()
{
  if (received_.load()) {
    geometry_msgs::msg::PoseStamped pose;
    {
      std::lock_guard<std::mutex> lk(pose_mutex_);
      pose = latest_pose_;
    }
    odom_sub_.reset();

    RCLCPP_INFO(node_->get_logger(),
      "[SavePose] Pose saved → x=%.3f y=%.3f frame='%s'",
      pose.pose.position.x, pose.pose.position.y,
      pose.header.frame_id.c_str());

    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (elapsed > timeout_sec_) {
    const auto topic = getInput<std::string>("pose_topic").value_or("/state_estimation");
    RCLCPP_ERROR(node_->get_logger(),
      "[SavePose] Timeout (%.1f s) — no odometry received on '%s'",
      timeout_sec_, topic.c_str());
    odom_sub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SavePose::onHalted()
{
  odom_sub_.reset();
  received_ = false;
  RCLCPP_WARN(node_->get_logger(), "[SavePose] Halted");
}

}  // namespace a2_bt
