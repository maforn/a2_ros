#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "object_detection_msgs/msg/object_detection_info_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace a2_bt
{

/**
 * Subscribes to detection_info, locks onto the first matching object, then
 * uses TF to compute a safe approach position (safe_distance metres in front of
 * the object along the robot→object vector) in the requested output frame.
 * The resulting PoseStamped is written to the blackboard for NavigateToPose.
 *
 * Input ports:
 *   class_id         (string) : YOLO class to find (empty = any)
 *   detection_topic  (string) : ObjectDetectionInfoArray topic (default: /detection_info)
 *   safe_distance    (double) : Goal distance from the object [m] (default: 3.0)
 *   output_frame     (string) : TF frame for the output pose (default: map)
 *   timeout_sec      (double) : Abort if no detection arrives (default: 10.0)
 *
 * Output ports:
 *   goal_pose (PoseStamped) : Safe approach pose on the blackboard
 */
class GetObjectPose : public BT::StatefulActionNode
{
public:
  GetObjectPose(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct RawDetection
  {
    std::string class_id;
    int32_t id;
    geometry_msgs::msg::Point position;
    std::string frame_id;
  };

  void detectionCb(
    const object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<
    object_detection_msgs::msg::ObjectDetectionInfoArray>::SharedPtr det_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string class_id_;
  double safe_distance_{3.0};
  double timeout_sec_{10.0};
  std::string output_frame_{"map"};

  std::mutex mtx_;
  std::optional<RawDetection> locked_;
  int32_t locked_id_{-1};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace a2_bt
