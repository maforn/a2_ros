#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"

class WaypointMux : public rclcpp::Node {
public:
  WaypointMux() : Node("waypoint_mux")
  {
    pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 10);

    sub_tare_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point_tare", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (mode_ == "explore") {
          pub_->publish(*msg);
        }
      });

    sub_far_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point_far", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (mode_ == "navigate") {
          pub_->publish(*msg);
        }
      });

    sub_mode_ = create_subscription<std_msgs::msg::String>(
      "/autonomy_mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "explore" || msg->data == "navigate") {
          mode_ = msg->data;
          RCLCPP_INFO(get_logger(), "Switching waypoint source to mode: %s", mode_.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Unknown autonomy mode: %s", msg->data.c_str());
        }
      });

    RCLCPP_INFO(get_logger(), "Waypoint Multiplexer initialized in mode: %s", mode_.c_str());
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_tare_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_far_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
  std::string mode_{"explore"};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointMux>());
  rclcpp::shutdown();
  return 0;
}
