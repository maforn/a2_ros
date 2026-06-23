#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "direct_lidar_inertial_odometry/srv/save_pcd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace a2_bt
{

/**
 * Calls DLIO's save_pcd service to persist the accumulated map to disk.
 *
 * Input ports:
 *   service_name (string) : Fully-qualified service name (default: /save_pcd)
 *   save_path    (string) : Output directory; empty uses DLIO's own default
 *   leaf_size    (double) : Voxel leaf size; <=0 uses DLIO's own default
 */
class SaveMap : public BT::StatefulActionNode
{
public:
  SaveMap(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using SrvT = direct_lidar_inertial_odometry::srv::SavePCD;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SrvT>::SharedPtr client_;
  rclcpp::Client<SrvT>::SharedFuture future_;
};

}  // namespace a2_bt
