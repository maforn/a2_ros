#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Converts MuJoCo sim point cloud (x,y,z,dist,nx,ny,nz) to Hesai-compatible
// format by injecting intensity=0, ring=0, timestamp=header_stamp.
// This lets RESPLE's HesaiBuff process sim clouds without modification.

class SimLidarAdapter : public rclcpp::Node
{
public:
  SimLidarAdapter() : Node("sim_lidar_adapter")
  {
    const auto in_topic  = declare_parameter<std::string>("input_topic",  "/front_lidar/points_raw");
    const auto out_topic = declare_parameter<std::string>("output_topic", "/front_lidar/points");

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { adapt(msg); });

    RCLCPP_INFO(get_logger(),
      "[SimLidarAdapter] %s → %s", in_topic.c_str(), out_topic.c_str());
  }

private:
  void adapt(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in)
  {
    const uint32_t n = in->width * in->height;
    if (n == 0) return;

    // Build output layout: x,y,z (float32) + intensity (float32) + ring (uint16) + timestamp (float64)
    sensor_msgs::msg::PointCloud2 out;
    out.header     = in->header;
    out.height     = 1;
    out.width      = n;
    out.is_dense   = in->is_dense;
    out.is_bigendian = false;

    // Field offsets
    constexpr uint32_t OFF_X   = 0;
    constexpr uint32_t OFF_Y   = 4;
    constexpr uint32_t OFF_Z   = 8;
    constexpr uint32_t OFF_I   = 12;
    constexpr uint32_t OFF_R   = 16;
    constexpr uint32_t OFF_T   = 20;  // double: 8 bytes after 2-byte ring + 2-byte padding
    constexpr uint32_t PSTEP   = 28;

    auto add_field = [&](const std::string & name, uint32_t off, uint8_t dt) {
      sensor_msgs::msg::PointField f;
      f.name     = name;
      f.offset   = off;
      f.datatype = dt;
      f.count    = 1;
      out.fields.push_back(f);
    };
    add_field("x",         OFF_X, sensor_msgs::msg::PointField::FLOAT32);
    add_field("y",         OFF_Y, sensor_msgs::msg::PointField::FLOAT32);
    add_field("z",         OFF_Z, sensor_msgs::msg::PointField::FLOAT32);
    add_field("intensity", OFF_I, sensor_msgs::msg::PointField::FLOAT32);
    add_field("ring",      OFF_R, sensor_msgs::msg::PointField::UINT16);
    add_field("timestamp", OFF_T, sensor_msgs::msg::PointField::FLOAT64);

    out.point_step = PSTEP;
    out.row_step   = PSTEP * n;
    out.data.assign(out.row_step, 0);

    // Timestamp common to all points — instantaneous capture
    const double stamp_sec = in->header.stamp.sec +
                             in->header.stamp.nanosec * 1e-9;

    // Iterators over input x,y,z
    sensor_msgs::PointCloud2ConstIterator<float> ix(*in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*in, "z");

    uint8_t * base = out.data.data();
    for (uint32_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
      uint8_t * p = base + i * PSTEP;
      *reinterpret_cast<float  *>(p + OFF_X) = *ix;
      *reinterpret_cast<float  *>(p + OFF_Y) = *iy;
      *reinterpret_cast<float  *>(p + OFF_Z) = *iz;
      *reinterpret_cast<float  *>(p + OFF_I) = 0.0f;
      *reinterpret_cast<uint16_t*>(p + OFF_R) = 0;
      *reinterpret_cast<double  *>(p + OFF_T) = stamp_sec;
    }

    pub_->publish(out);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimLidarAdapter>());
  rclcpp::shutdown();
  return 0;
}
