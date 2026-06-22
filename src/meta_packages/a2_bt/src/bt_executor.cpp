#include <chrono>
#include <filesystem>
#include <memory>

#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "a2_bt/create_pose.hpp"
#include "a2_bt/navigate_to_pose_action.hpp"
#include "a2_bt/publish_twist.hpp"
#include "a2_bt/set_mode.hpp"
#include "a2_bt/get_object_pose.hpp"
#include "a2_bt/save_image.hpp"
#include "a2_bt/call_trigger_service.hpp"
#include "a2_bt/start_exploration.hpp"
#include "a2_bt/wait_for_exploration_finish.hpp"
#include "a2_bt/is_exploration_finished.hpp"
#include "a2_bt/save_pose.hpp"

class A2BtExecutor : public BT::TreeExecutionServer
{
public:
  explicit A2BtExecutor(const rclcpp::NodeOptions & options)
  : BT::TreeExecutionServer(options)
  {}

protected:
  void registerNodesIntoFactory(BT::BehaviorTreeFactory & factory) override
  {
    BT::RosNodeParams params;
    params.nh                    = node();
    params.server_timeout        = std::chrono::milliseconds(30000);
    params.wait_for_server_timeout = std::chrono::milliseconds(5000);

    factory.registerNodeType<a2_bt::CallTriggerService>("CallTriggerService", params);
    factory.registerNodeType<a2_bt::CreatePose>("CreatePose");
    factory.registerNodeType<a2_bt::NavigateToPoseAction>("NavigateToPose", params);
    factory.registerNodeType<a2_bt::PublishTwist>("PublishTwist", params);
    factory.registerNodeType<a2_bt::SetMode>("SetMode", params);
    factory.registerNodeType<a2_bt::GetObjectPose>("GetObjectPose", params);
    factory.registerNodeType<a2_bt::SaveImage>("SaveImage", params);
    factory.registerNodeType<a2_bt::StartExploration>("StartExploration", params);
    factory.registerNodeType<a2_bt::WaitForExplorationFinish>("WaitForExplorationFinish", params);
    factory.registerNodeType<a2_bt::IsExplorationFinished>("IsExplorationFinished", params);
    factory.registerNodeType<a2_bt::SavePose>("SavePose", params);
  }

  // If payload is a filename, resolve it against the hardcoded trees directory
  // and load it into the factory so that createTree(target_tree) can find it.
  bool onGoalReceived(const std::string & tree_name, const std::string & payload) override
  {
    if (payload.empty()) {
      return true;
    }

    static constexpr const char * kTreesDir =
      "/a2_ros/src/meta_packages/a2_bt/behavior_trees";

    const auto path = std::filesystem::path(kTreesDir) / payload;

    if (!std::filesystem::exists(path)) {
      RCLCPP_ERROR(node()->get_logger(),
        "[BtExecutor] Tree file not found: %s", path.c_str());
      return false;
    }

    try {
      factory().registerBehaviorTreeFromFile(path.string());
      RCLCPP_INFO(node()->get_logger(),
        "[BtExecutor] Loaded '%s' from %s", tree_name.c_str(), path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node()->get_logger(),
        "[BtExecutor] Failed to load tree file: %s", e.what());
      return false;
    }

    return true;
  }

  void onTreeCreated(BT::Tree & tree) override
  {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }

  std::optional<std::string> onTreeExecutionCompleted(
    BT::NodeStatus status, bool was_cancelled) override
  {
    logger_cout_.reset();
    (void)status;
    (void)was_cancelled;
    return std::nullopt;
  }

private:
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto server = std::make_shared<A2BtExecutor>(options);

  // MultiThreadedExecutor with 250 ms timeout to avoid deadlock on dynamic
  // publisher/subscriber removal while spinning (known upstream issue).
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(server->node());
  exec.spin();
  exec.remove_node(server->node());

  rclcpp::shutdown();
  return 0;
}
