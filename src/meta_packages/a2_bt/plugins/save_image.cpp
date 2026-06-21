#include "a2_bt/save_image.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/imgcodecs.hpp>

namespace a2_bt
{

SaveImage::SaveImage(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh)
{}

BT::PortsList SaveImage::providedPorts()
{
  return {
    BT::InputPort<std::string>("image_topic",  "/detections_in_image", "Source image topic"),
    BT::InputPort<std::string>("output_path",  "/tmp/detection.jpg",   "File path to write"),
    BT::InputPort<double>("timeout_sec",       5.0,                    "Timeout [s]"),
  };
}

BT::NodeStatus SaveImage::onStart()
{
  const auto topic = getInput<std::string>("image_topic").value_or("/detections_in_image");
  output_path_     = getInput<std::string>("output_path").value_or("/tmp/detection.jpg");
  timeout_sec_     = getInput<double>("timeout_sec").value_or(5.0);

  frame_ready_ = false;
  cb_failed_   = false;
  start_time_  = std::chrono::steady_clock::now();

  img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topic, 1,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      if (frame_ready_ || cb_failed_) return;
      try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        {
          std::lock_guard<std::mutex> lk(mtx_);
          frame_ = cv_ptr->image.clone();
        }
        frame_ready_ = true;
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "[SaveImage] cv_bridge: %s", e.what());
        cb_failed_ = true;
      }
    });

  RCLCPP_INFO(node_->get_logger(),
    "[SaveImage] Waiting for image on %s → %s", topic.c_str(), output_path_.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SaveImage::onRunning()
{
  if (cb_failed_) {
    img_sub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  if (frame_ready_) {
    img_sub_.reset();
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      frame = frame_;
    }
    if (cv::imwrite(output_path_, frame)) {
      RCLCPP_INFO(node_->get_logger(), "[SaveImage] Saved to %s", output_path_.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(node_->get_logger(),
      "[SaveImage] cv::imwrite failed — check path: %s", output_path_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(), "[SaveImage] Timeout — no image received");
    img_sub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SaveImage::onHalted()
{
  img_sub_.reset();
}

}  // namespace a2_bt
