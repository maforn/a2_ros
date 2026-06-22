#include "a2_bt/create_pose.hpp"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"

namespace a2_bt
{

CreatePose::CreatePose(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{}

BT::PortsList CreatePose::providedPorts()
{
  return {
    BT::InputPort<double>("x",        0.0,   "X position [m]"),
    BT::InputPort<double>("y",        0.0,   "Y position [m]"),
    BT::InputPort<double>("yaw",      0.0,   "Yaw angle [rad]"),
    BT::InputPort<std::string>("frame_id", "map", "TF reference frame"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Resulting PoseStamped"),
  };
}

BT::NodeStatus CreatePose::tick()
{
  const double x      = getInput<double>("x").value();
  const double y      = getInput<double>("y").value();
  const double yaw    = getInput<double>("yaw").value();
  const auto frame_id = getInput<std::string>("frame_id").value();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id    = frame_id;
  pose.pose.position.x    = x;
  pose.pose.position.y    = y;
  pose.pose.position.z    = 0.0;
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  setOutput("pose", pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace a2_bt
