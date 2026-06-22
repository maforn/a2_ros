#pragma once

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace a2_bt
{

/**
 * Synchronous BT node that builds a geometry_msgs::PoseStamped from
 * individual x / y / yaw / frame_id inputs and writes it to the blackboard.
 *
 * Input ports : x, y, yaw (rad), frame_id
 * Output ports: pose (PoseStamped)
 */
class CreatePose : public BT::SyncActionNode
{
public:
  CreatePose(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace a2_bt
