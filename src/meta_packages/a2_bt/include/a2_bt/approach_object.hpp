#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "object_detection_msgs/msg/object_detection_info_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace a2_bt
{

/**
 * Approaches a detected object of a given class until within stop_distance.
 * Uses proportional visual servoing on the 3-D position reported by
 * ObjectDetectionInfoArray (camera optical frame, z = depth):
 *   - angular_z proportional to horizontal bearing
 *   - linear_x proportional to remaining distance (only when roughly aligned)
 *
 * Input ports:
 *   class_id         (string) : YOLO class to approach, e.g. "bottle". Empty = any class.
 *   stop_distance    (double) : Target distance in metres (default: 3.0)
 *   detection_topic  (string) : ObjectDetectionInfoArray topic (default: /detection_info)
 *   cmd_vel_topic    (string) : Velocity command topic (default: /cmd_vel)
 *   timeout_sec      (double) : Total abort timeout (default: 30.0)
 *   lost_timeout_sec (double) : Abort if object not seen for this long (default: 2.0)
 */
class ApproachObject : public BT::StatefulActionNode
{
public:
  ApproachObject(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct Detection { double x, z; };

  void detectionCb(
    const object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg);
  void publishTwist(double linear_x, double angular_z);
  void publishStop();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<
    object_detection_msgs::msg::ObjectDetectionInfoArray>::SharedPtr det_sub_;

  std::string class_id_;
  double stop_distance_{3.0};
  double timeout_sec_{30.0};
  double lost_timeout_sec_{2.0};

  std::mutex mtx_;
  std::optional<Detection> latest_;
  std::chrono::steady_clock::time_point last_seen_;
  bool seen_ever_{false};
  int32_t locked_id_{-1};  // id of the object we locked onto (-1 = not yet locked)

  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace a2_bt
