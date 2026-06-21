#include "a2_bt/approach_object.hpp"

#include <algorithm>
#include <cmath>

namespace a2_bt
{

// Proportional gains and safety caps for the visual-servoing controller.
static constexpr double kAngGain  = 2.0;   // (rad/s) / rad bearing error
static constexpr double kLinGain  = 0.15;  // (m/s)   / m distance error
static constexpr double kMaxAng   = 0.6;   // rad/s  (square loop uses 0.5 successfully)
static constexpr double kMaxLin   = 0.3;   // m/s
// Only move forward once heading error is within this threshold.
static constexpr double kAlignThr = 0.2;   // rad (~11°)

ApproachObject::ApproachObject(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList ApproachObject::providedPorts()
{
  return {
    BT::InputPort<std::string>("class_id",         "",                  "YOLO class to approach (empty = any)"),
    BT::InputPort<double>("stop_distance",          3.0,                 "Stop distance [m]"),
    BT::InputPort<std::string>("detection_topic",   "/detection_info",   "ObjectDetectionInfoArray topic"),
    BT::InputPort<std::string>("cmd_vel_topic",     "/cmd_vel",          "Velocity command topic"),
    BT::InputPort<double>("timeout_sec",            30.0,                "Total timeout [s]"),
    BT::InputPort<double>("lost_timeout_sec",       2.0,                 "Abort if object lost for [s]"),
  };
}

BT::NodeStatus ApproachObject::onStart()
{
  class_id_         = getInput<std::string>("class_id").value_or("");
  stop_distance_    = getInput<double>("stop_distance").value_or(3.0);
  timeout_sec_      = getInput<double>("timeout_sec").value_or(30.0);
  lost_timeout_sec_ = getInput<double>("lost_timeout_sec").value_or(2.0);

  const auto det_topic = getInput<std::string>("detection_topic").value_or("/detection_info");
  const auto cmd_topic = getInput<std::string>("cmd_vel_topic").value_or("/cmd_vel");

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic, 1);
  det_sub_ = node_->create_subscription<
    object_detection_msgs::msg::ObjectDetectionInfoArray>(
    det_topic, 5,
    [this](const object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg) {
      detectionCb(msg);
    });

  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_.reset();
    seen_ever_ = false;
    locked_id_ = -1;
  }

  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(),
    "[ApproachObject] Approaching '%s' to %.1f m",
    class_id_.empty() ? "<any>" : class_id_.c_str(), stop_distance_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachObject::onRunning()
{
  const double total_elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (total_elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(), "[ApproachObject] Timeout — aborting");
    publishStop();
    return BT::NodeStatus::FAILURE;
  }

  std::optional<Detection> det;
  bool seen_ever;
  std::chrono::steady_clock::time_point last_seen;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    det       = latest_;
    seen_ever = seen_ever_;
    last_seen = last_seen_;
  }

  if (!det) {
    if (seen_ever) {
      const double lost = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_seen).count();
      if (lost > lost_timeout_sec_) {
        RCLCPP_WARN(node_->get_logger(), "[ApproachObject] Object lost — aborting");
        publishStop();
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "[ApproachObject] Object briefly lost — waiting (%.1f / %.1f s)", lost, lost_timeout_sec_);
    } else {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[ApproachObject] Waiting for first detection of '%s' on topic '%s'...",
        class_id_.empty() ? "<any>" : class_id_.c_str(),
        getInput<std::string>("detection_topic").value_or("/detection_info").c_str());
    }
    publishTwist(0.0, 0.0);
    return BT::NodeStatus::RUNNING;
  }

  // Horizontal distance and bearing in the camera optical frame.
  // position.z = depth (forward), position.x = right.
  const double horiz_dist = std::sqrt(det->x * det->x + det->z * det->z);
  const double bearing    = std::atan2(det->x, det->z);  // positive = object to the right

  if (horiz_dist <= stop_distance_) {
    RCLCPP_INFO(node_->get_logger(),
      "[ApproachObject] Reached %.2f m — SUCCESS", horiz_dist);
    publishStop();
    return BT::NodeStatus::SUCCESS;
  }

  // Proportional controller: turn toward object, advance when aligned.
  const double ang = std::clamp(-kAngGain * bearing, -kMaxAng, kMaxAng);
  const double lin = (std::abs(bearing) < kAlignThr)
    ? std::clamp(kLinGain * (horiz_dist - stop_distance_), 0.0, kMaxLin)
    : 0.0;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
    "[ApproachObject] dist=%.2f m bearing=%.2f rad → lin=%.2f ang=%.2f (publishing to cmd_vel)",
    horiz_dist, bearing, lin, ang);

  publishTwist(lin, ang);
  return BT::NodeStatus::RUNNING;
}

void ApproachObject::onHalted()
{
  publishStop();
  det_sub_.reset();
  cmd_pub_.reset();
}

void ApproachObject::detectionCb(
  const object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (locked_id_ == -1) {
    // Not locked yet — pick the highest-confidence match and lock onto it.
    const object_detection_msgs::msg::ObjectDetectionInfo * best = nullptr;
    for (const auto & info : msg->info) {
      if (!class_id_.empty() && info.class_id != class_id_) continue;
      if (!best || info.confidence > best->confidence) best = &info;
    }
    if (best) {
      locked_id_ = best->id;
      latest_    = Detection{best->position.x, best->position.z};
      last_seen_ = std::chrono::steady_clock::now();
      seen_ever_ = true;
      RCLCPP_INFO(node_->get_logger(),
        "[ApproachObject] Locked onto '%s' (id=%d, confidence=%.2f)",
        best->class_id.c_str(), best->id, best->confidence);
    }
  } else {
    // Already locked — only accept updates from the same object id.
    const object_detection_msgs::msg::ObjectDetectionInfo * target = nullptr;
    for (const auto & info : msg->info) {
      if (info.id == locked_id_) { target = &info; break; }
    }
    if (target) {
      latest_    = Detection{target->position.x, target->position.z};
      last_seen_ = std::chrono::steady_clock::now();
    } else {
      latest_.reset();  // target not in this frame — lost_timeout will handle it
    }
  }
}

void ApproachObject::publishTwist(double linear_x, double angular_z)
{
  if (!cmd_pub_) return;
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp    = node_->now();
  msg.twist.linear.x  = linear_x;
  msg.twist.angular.z = angular_z;
  cmd_pub_->publish(msg);
}

void ApproachObject::publishStop()
{
  publishTwist(0.0, 0.0);
}

}  // namespace a2_bt
