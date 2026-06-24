#include "a2_bt/detection_mapper.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace a2_bt
{

// ── Color palette (HSV-inspired, one stable colour per class) ────────────────

static std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 0.85f)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}

static const std::array<std_msgs::msg::ColorRGBA, 10> kPalette = {{
  makeColor(0.95f, 0.26f, 0.21f),  // red
  makeColor(0.13f, 0.59f, 0.95f),  // blue
  makeColor(0.30f, 0.69f, 0.31f),  // green
  makeColor(1.00f, 0.76f, 0.03f),  // yellow
  makeColor(0.61f, 0.15f, 0.69f),  // purple
  makeColor(1.00f, 0.34f, 0.13f),  // orange
  makeColor(0.00f, 0.74f, 0.83f),  // cyan
  makeColor(0.91f, 0.12f, 0.39f),  // pink
  makeColor(0.47f, 0.33f, 0.28f),  // brown
  makeColor(0.38f, 0.49f, 0.55f),  // blue-grey
}};

// ── Node implementation ───────────────────────────────────────────────────────

DetectionMapperNode::DetectionMapperNode(const rclcpp::NodeOptions & options)
: Node("detection_mapper", options)
{
  const auto det_topic = declare_parameter<std::string>("detection_topic", "/detection_info");
  output_frame_        = declare_parameter<std::string>("output_frame",    "map");
  cluster_radius_      = declare_parameter<double>("cluster_radius",       1.0);
  min_confidence_      = static_cast<float>(declare_parameter<double>("min_confidence", 0.4));
  marker_ns_           = declare_parameter<std::string>("marker_ns",       "detected_objects");
  csv_path_            = declare_parameter<std::string>("csv_path",        "/tmp/detections.csv");
  const double hz      = declare_parameter<double>("detection_hz",         2.0);

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  det_sub_ = create_subscription<
    object_detection_msgs::msg::ObjectDetectionInfoArray>(
    det_topic, 10,
    [this](object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg) {
      detectionCb(std::move(msg));
    });

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/detected_objects_markers", 10);

  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
  process_timer_ = create_wall_timer(period, [this] { processTick(); });

  start_srv_ = create_service<Trigger>(
    "detection_mapper/start",
    [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
      {
        std::lock_guard<std::mutex> lk(objects_mtx_);
        objects_.clear();
        next_id_ = 0;
      }
      running_ = true;
      RCLCPP_INFO(get_logger(), "[DetectionMapper] Started — object list cleared");
      resp->success = true;
      resp->message = "Detection mapper started";
    });

  stop_srv_ = create_service<Trigger>(
    "detection_mapper/stop",
    [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
      running_ = false;
      std::size_t n;
      {
        std::lock_guard<std::mutex> lk(objects_mtx_);
        n = objects_.size();
      }
      RCLCPP_INFO(get_logger(),
        "[DetectionMapper] Stopped — %zu objects accumulated", n);
      resp->success = true;
      resp->message = "Detection mapper stopped, " + std::to_string(n) + " objects stored";
    });

  save_csv_srv_ = create_service<Trigger>(
    "detection_mapper/save_csv",
    [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
      std::vector<DetectedObject> snapshot;
      {
        std::lock_guard<std::mutex> lk(objects_mtx_);
        snapshot = objects_;
      }

      std::ofstream f(csv_path_);
      if (!f.is_open()) {
        const std::string msg = "Failed to open: " + csv_path_;
        RCLCPP_ERROR(get_logger(), "[DetectionMapper] %s", msg.c_str());
        resp->success = false;
        resp->message = msg;
        return;
      }

      f << "id,class,x,y,z,confidence\n";
      for (const auto & obj : snapshot) {
        f << obj.id << ","
          << obj.class_id << ","
          << obj.position.x << ","
          << obj.position.y << ","
          << obj.position.z << ","
          << obj.best_confidence << "\n";
      }

      RCLCPP_INFO(get_logger(),
        "[DetectionMapper] Saved %zu detections → %s",
        snapshot.size(), csv_path_.c_str());
      resp->success = true;
      resp->message = "Saved " + std::to_string(snapshot.size()) +
                      " detections to " + csv_path_;
    });

  RCLCPP_INFO(get_logger(),
    "[DetectionMapper] Ready (idle). topic=%s frame=%s cluster_r=%.1f m "
    "conf>=%.2f rate=%.1f Hz csv=%s — call ~/start to begin",
    det_topic.c_str(), output_frame_.c_str(),
    cluster_radius_, min_confidence_, hz, csv_path_.c_str());
}

void DetectionMapperNode::detectionCb(
  object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  latest_  = std::move(msg);
  has_new_ = true;
}

void DetectionMapperNode::processTick()
{
  if (!running_.load()) return;

  // Grab latest detections under the lock, then release before processing
  object_detection_msgs::msg::ObjectDetectionInfoArray::SharedPtr detections;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_new_ || !latest_) return;
    detections = latest_;
    has_new_   = false;
  }

  std::lock_guard<std::mutex> obj_lk(objects_mtx_);

  for (const auto & info : detections->info) {
    if (info.confidence < min_confidence_) continue;

    // Transform detection position from camera frame → output_frame
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.header          = detections->header;
    pt_in.header.stamp    = rclcpp::Time(0);  // use latest available TF
    pt_in.point           = info.position;

    try {
      tf_buffer_->transform(pt_in, pt_out, output_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[DetectionMapper] TF '%s'→'%s' failed: %s",
        detections->header.frame_id.c_str(), output_frame_.c_str(), ex.what());
      continue;
    }

    // Search for an existing object of the same class within cluster_radius
    DetectedObject * nearest = nullptr;
    double nearest_dist = std::numeric_limits<double>::max();

    for (auto & obj : objects_) {
      if (obj.class_id != info.class_id) continue;
      const double dx   = obj.position.x - pt_out.point.x;
      const double dy   = obj.position.y - pt_out.point.y;
      const double dz   = obj.position.z - pt_out.point.z;
      const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist < cluster_radius_ && dist < nearest_dist) {
        nearest_dist = dist;
        nearest      = &obj;
      }
    }

    if (nearest) {
      // Merge into existing: running mean position
      const double n        = static_cast<double>(nearest->count);
      nearest->position.x   = (nearest->position.x * n + pt_out.point.x) / (n + 1.0);
      nearest->position.y   = (nearest->position.y * n + pt_out.point.y) / (n + 1.0);
      nearest->position.z   = (nearest->position.z * n + pt_out.point.z) / (n + 1.0);
      nearest->best_confidence = std::max(nearest->best_confidence, static_cast<float>(info.confidence));
      nearest->count++;
    } else {
      // New unique object
      DetectedObject obj;
      obj.id              = next_id_++;
      obj.class_id        = info.class_id;
      obj.position        = pt_out.point;
      obj.count           = 1;
      obj.best_confidence = static_cast<float>(info.confidence);
      objects_.push_back(obj);

      RCLCPP_INFO(get_logger(),
        "[DetectionMapper] New object: '%s' id=%d at (%.2f, %.2f, %.2f) conf=%.2f",
        obj.class_id.c_str(), obj.id,
        obj.position.x, obj.position.y, obj.position.z,
        obj.best_confidence);
    }
  }

  publishMarkers();
}

void DetectionMapperNode::publishMarkers()
{
  visualization_msgs::msg::MarkerArray array;

  // Clear all previous markers in our namespace
  visualization_msgs::msg::Marker del;
  del.header.frame_id = output_frame_;
  del.header.stamp    = rclcpp::Time(0);  // latest available TF
  del.ns              = marker_ns_;
  del.action          = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(del);

  for (const auto & obj : objects_) {
    const auto color = classColor(obj.class_id);

    // ── Sphere at object position ─────────────────────────────────────────
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id    = output_frame_;
    sphere.header.stamp       = rclcpp::Time(0);  // use latest available TF
    sphere.ns                 = marker_ns_;
    sphere.id                 = obj.id * 2;
    sphere.type               = visualization_msgs::msg::Marker::SPHERE;
    sphere.action             = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position      = obj.position;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3;
    sphere.color              = color;
    array.markers.push_back(sphere);

    // ── Text label above sphere ───────────────────────────────────────────
    visualization_msgs::msg::Marker text;
    text.header.frame_id    = output_frame_;
    text.header.stamp       = rclcpp::Time(0);
    text.ns                 = marker_ns_;
    text.id                 = obj.id * 2 + 1;
    text.type               = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action             = visualization_msgs::msg::Marker::ADD;
    text.pose.position      = obj.position;
    text.pose.position.z   += 0.4;
    text.pose.orientation.w = 1.0;
    text.scale.z            = 0.22;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.text = obj.class_id;
    array.markers.push_back(text);
  }

  marker_pub_->publish(array);
}

std_msgs::msg::ColorRGBA DetectionMapperNode::classColor(const std::string & class_id) const
{
  const std::size_t idx = std::hash<std::string>{}(class_id) % kPalette.size();
  return kPalette[idx];
}

}  // namespace a2_bt

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<a2_bt::DetectionMapperNode>());
  rclcpp::shutdown();
  return 0;
}
