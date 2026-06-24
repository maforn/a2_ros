#pragma once

#include <atomic>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "object_detection_msgs/msg/object_detection_info_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace a2_bt
{

struct DetectedObject
{
  int id;
  std::string class_id;
  geometry_msgs::msg::Point position;  // map frame, running mean
  int count{1};
  float best_confidence{0.0f};
};

/**
 * Standalone node that accumulates object detections from a YOLO pipeline into
 * a deduplicated map-frame list and publishes RViz markers.
 *
 * Processing flow (at detection_hz):
 *   1. Take the latest ObjectDetectionInfoArray message.
 *   2. Transform each detection's position from the camera frame to output_frame.
 *   3. For each detection, search existing objects of the same class within
 *      cluster_radius metres. If found, update its running-mean position and
 *      increment detection count. Otherwise add a new entry.
 *   4. Republish the full list as a MarkerArray (sphere + label per object).
 *
 * Services:
 *   ~/start    (std_srvs/Trigger) — enable detection accumulation, clears list
 *   ~/stop     (std_srvs/Trigger) — disable accumulation (keeps stored objects)
 *   ~/save_csv (std_srvs/Trigger) — write current list to CSV (id,class,x,y,z,confidence)
 *
 * Parameters:
 *   detection_topic  (string, default: /detection_info)
 *   output_frame     (string, default: map)
 *   cluster_radius   (double, default: 1.0)   — merge radius [m]
 *   min_confidence   (double, default: 0.4)   — ignore detections below this
 *   detection_hz     (double, default: 2.0)   — processing rate [Hz]
 *   marker_ns        (string, default: detected_objects)
 *
 * Published topics:
 *   /detected_objects_markers  (visualization_msgs/MarkerArray)
 */
class DetectionMapperNode : public rclcpp::Node
{
public:
  explicit DetectionMapperNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Trigger = std_srvs::srv::Trigger;

  void detectionCb(
    object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg);
  void processTick();
  void publishMarkers();
  std_msgs::msg::ColorRGBA classColor(const std::string & class_id) const;

  rclcpp::Subscription<
    object_detection_msgs::msg::ObjectDetectionInfoArray>::SharedPtr det_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<Trigger>::SharedPtr save_csv_srv_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Controlled by start/stop services — processing is off until started
  std::atomic<bool> running_{false};

  // Latest raw detections (written by subscription callback, read by timer)
  std::mutex mtx_;
  object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr latest_;
  bool has_new_{false};

  // Accumulated, deduplicated object list — protected by objects_mtx_
  std::mutex objects_mtx_;
  std::vector<DetectedObject> objects_;
  int next_id_{0};

  // Parameters
  std::string output_frame_;
  double cluster_radius_;
  float min_confidence_;
  std::string marker_ns_;
  std::string csv_path_;
};

}  // namespace a2_bt
