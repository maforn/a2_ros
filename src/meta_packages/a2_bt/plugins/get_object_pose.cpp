#include "a2_bt/get_object_pose.hpp"

#include <cmath>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace a2_bt
{

GetObjectPose::GetObjectPose(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList GetObjectPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("class_id",        "",                 "YOLO class to find (empty = any)"),
    BT::InputPort<std::string>("detection_topic",  "/detection_info",  "ObjectDetectionInfoArray topic"),
    BT::InputPort<double>("safe_distance",         3.0,                "Goal distance from object [m]"),
    BT::InputPort<std::string>("output_frame",     "map",              "TF frame for output pose"),
    BT::InputPort<double>("timeout_sec",           10.0,               "Max wait for detection [s]"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose",       "Safe approach pose"),
  };
}

BT::NodeStatus GetObjectPose::onStart()
{
  class_id_      = getInput<std::string>("class_id").value_or("");
  safe_distance_ = getInput<double>("safe_distance").value_or(3.0);
  output_frame_  = getInput<std::string>("output_frame").value_or("map");
  timeout_sec_   = getInput<double>("timeout_sec").value_or(10.0);

  const auto det_topic = getInput<std::string>("detection_topic").value_or("/detection_info");

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    locked_.reset();
    locked_id_ = -1;
  }

  det_sub_ = node_->create_subscription<
    object_detection_msgs::msg::ObjectDetectionInfoArray>(
    det_topic, 5,
    [this](const object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg) {
      detectionCb(msg);
    });

  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(),
    "[GetObjectPose] Waiting for '%s' on %s",
    class_id_.empty() ? "<any>" : class_id_.c_str(), det_topic.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetObjectPose::onRunning()
{
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(), "[GetObjectPose] Timeout — no detection");
    det_sub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  std::optional<RawDetection> det;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    det = locked_;
  }

  if (!det) {
    return BT::NodeStatus::RUNNING;
  }

  det_sub_.reset();

  // Transform object position from detection frame → output_frame.
  geometry_msgs::msg::PointStamped obj_in, obj_out;
  obj_in.header.frame_id = det->frame_id;
  obj_in.header.stamp    = rclcpp::Time(0);
  obj_in.point           = det->position;

  try {
    tf_buffer_->transform(obj_in, obj_out, output_frame_);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
      "[GetObjectPose] TF %s→%s failed: %s",
      det->frame_id.c_str(), output_frame_.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Get robot position in output_frame.
  geometry_msgs::msg::PointStamped robot_in, robot_out;
  robot_in.header.frame_id = "base_link";
  robot_in.header.stamp    = rclcpp::Time(0);

  try {
    tf_buffer_->transform(robot_in, robot_out, output_frame_);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "[GetObjectPose] Can't get robot pose: %s — placing goal at object position", ex.what());
    robot_out = obj_out;
  }

  // Place goal safe_distance metres from the object along the robot→object line.
  const double dx   = obj_out.point.x - robot_out.point.x;
  const double dy   = obj_out.point.y - robot_out.point.y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = output_frame_;
  goal.header.stamp    = node_->now();
  goal.pose.orientation.w = 1.0;

  if (dist > safe_distance_) {
    const double scale        = (dist - safe_distance_) / dist;
    goal.pose.position.x = robot_out.point.x + dx * scale;
    goal.pose.position.y = robot_out.point.y + dy * scale;
  } else {
    // Already within safe_distance — stay in place.
    goal.pose.position.x = robot_out.point.x;
    goal.pose.position.y = robot_out.point.y;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[GetObjectPose] '%s' at (%.2f, %.2f) → goal (%.2f, %.2f) in %s",
    det->class_id.c_str(),
    obj_out.point.x, obj_out.point.y,
    goal.pose.position.x, goal.pose.position.y,
    output_frame_.c_str());

  setOutput("goal_pose", goal);
  return BT::NodeStatus::SUCCESS;
}

void GetObjectPose::onHalted()
{
  det_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
}

void GetObjectPose::detectionCb(
  const object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (locked_id_ != -1) return;

  const object_detection_msgs::msg::ObjectDetectionInfo * best = nullptr;
  for (const auto & info : msg->info) {
    if (!class_id_.empty() && info.class_id != class_id_) continue;
    if (!best || info.confidence > best->confidence) best = &info;
  }

  if (best) {
    locked_id_ = best->id;
    locked_    = RawDetection{best->class_id, best->id, best->position, msg->header.frame_id};
    RCLCPP_INFO(node_->get_logger(),
      "[GetObjectPose] Locked '%s' (id=%d, conf=%.2f)",
      best->class_id.c_str(), best->id, best->confidence);
  }
}

}  // namespace a2_bt
