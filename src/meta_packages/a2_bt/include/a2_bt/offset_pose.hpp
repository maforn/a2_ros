#pragma once

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace a2_bt
{

/**
 * Synchronous BT node that offsets a pose along the robot's forward (X) axis
 * and writes the result to the blackboard.
 *
 * Given an input PoseStamped and a distance, computes:
 *   out.x = in.x + distance_x * cos(yaw)
 *   out.y = in.y + distance_x * sin(yaw)
 * preserving the original orientation and frame_id.
 *
 * Input ports:
 *   input_pose  (PoseStamped) : pose to offset from (read from blackboard)
 *   distance_x  (double)      : distance along robot forward axis [m] (default: 1.0)
 *   distance_y  (double)      : distance along robot left axis [m] (default: 0.0)
 *
 * Output ports:
 *   output_pose (PoseStamped) : resulting pose written to blackboard
 */
class OffsetPose : public BT::SyncActionNode
{
public:
  OffsetPose(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace a2_bt
