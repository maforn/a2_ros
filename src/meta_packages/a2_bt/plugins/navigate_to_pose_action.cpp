#include "a2_bt/navigate_to_pose_action.hpp"
#include "std_msgs/msg/int8.hpp"

namespace a2_bt
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList NavigateToPoseAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose",
      "Goal pose from blackboard (x/y extracted; takes priority over goal_x/goal_y)"),
    BT::InputPort<double>("goal_x",   0.0,  "Goal X [m]"),
    BT::InputPort<double>("goal_y",   0.0,  "Goal Y [m]"),
    BT::InputPort<std::string>("frame_id",    "map",         "Reference frame"),
    BT::InputPort<std::string>("goal_topic",  "/goal_point", "FAR Planner goal topic"),
    BT::InputPort<double>("timeout_sec", 60.0, "Max seconds to wait before FAILURE"),
  };
}

BT::NodeStatus NavigateToPoseAction::onStart()
{
  const auto goal_topic = getInput<std::string>("goal_topic").value_or("/goal_point");
  const auto frame_id   = getInput<std::string>("frame_id").value_or("map");

  geometry_msgs::msg::PointStamped goal_msg;
  goal_msg.header.stamp    = node_->now();
  goal_msg.header.frame_id = frame_id;

  geometry_msgs::msg::PoseStamped pose;
  if (getInput("goal_pose", pose)) {
    goal_msg.point.x = pose.pose.position.x;
    goal_msg.point.y = pose.pose.position.y;
    goal_msg.point.z = pose.pose.position.z;
    if (!pose.header.frame_id.empty()) {
      goal_msg.header.frame_id = pose.header.frame_id;
    }
  } else {
    goal_msg.point.x = getInput<double>("goal_x").value_or(0.0);
    goal_msg.point.y = getInput<double>("goal_y").value_or(0.0);
    goal_msg.point.z = 0.0;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[NavigateToPose] Sending goal → x=%.2f y=%.2f frame=%s via %s",
    goal_msg.point.x, goal_msg.point.y,
    goal_msg.header.frame_id.c_str(), goal_topic.c_str());

  // Resume pathFollower (clears any safetyStop set by StopMovement) so it can
  // follow the new path once far_planner starts publishing waypoints.
  {
    auto stop_pub = node_->create_publisher<std_msgs::msg::Int8>("/stop", rclcpp::QoS(1));
    std_msgs::msg::Int8 stop_msg;
    stop_msg.data = 0;
    stop_pub->publish(stop_msg);
  }

  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(goal_topic, 1);
  goal_pub_->publish(goal_msg);

  reached_ = false;
  timeout_sec_ = getInput<double>("timeout_sec").value_or(60.0);
  start_time_ = std::chrono::steady_clock::now();

  status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/far_reach_goal_status", 5,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) {
        reached_ = true;
      }
    });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning()
{
  if (reached_.load()) {
    RCLCPP_INFO(node_->get_logger(), "[NavigateToPose] Goal reached — SUCCESS");
    status_sub_.reset();
    goal_pub_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(),
      "[NavigateToPose] Timeout (%.1f s) — FAILURE", timeout_sec_);
    status_sub_.reset();
    goal_pub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateToPoseAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[NavigateToPose] Halted — cancelling navigation");
  status_sub_.reset();
  goal_pub_.reset();
  reached_ = false;
}

}  // namespace a2_bt
