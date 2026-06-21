#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace a2_bt
{

/**
 * Waits for the next frame on an image topic and saves it to disk.
 * Useful for capturing a detection snapshot after ApproachObject succeeds.
 *
 * Input ports:
 *   image_topic  (string) : Source image topic (default: /detections_in_image)
 *   output_path  (string) : Absolute file path to write (default: /tmp/detection.jpg)
 *   timeout_sec  (double) : Abort if no image arrives within this time (default: 5.0)
 */
class SaveImage : public BT::StatefulActionNode
{
public:
  SaveImage(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  std::string output_path_;
  double timeout_sec_{5.0};
  std::chrono::steady_clock::time_point start_time_;

  std::mutex mtx_;
  cv::Mat frame_;
  std::atomic<bool> frame_ready_{false};
  std::atomic<bool> cb_failed_{false};
};

}  // namespace a2_bt
