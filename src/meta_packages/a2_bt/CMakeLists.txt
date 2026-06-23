cmake_minimum_required(VERSION 3.8)
project(a2_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(a2_interfaces REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(direct_lidar_inertial_odometry REQUIRED)

set(DEPENDENCIES
  behaviortree_cpp
  behaviortree_ros2
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  a2_interfaces
  object_detection_msgs
  sensor_msgs
  cv_bridge
  visualization_msgs
  direct_lidar_inertial_odometry
)

# Creates a shared library BT plugin
macro(bt_plugin target source)
  add_library(${target} SHARED ${source})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${target} ${DEPENDENCIES})
endmacro()

# ── Plugins ──────────────────────────────────────────────────────────────────
bt_plugin(a2_bt_create_pose_plugin                 plugins/create_pose.cpp)
bt_plugin(a2_bt_navigate_to_pose_plugin            plugins/navigate_to_pose_action.cpp)
bt_plugin(a2_bt_publish_twist_plugin               plugins/publish_twist.cpp)
bt_plugin(a2_bt_set_mode_plugin         plugins/set_mode.cpp)
bt_plugin(a2_bt_get_object_pose_plugin  plugins/get_object_pose.cpp)

# save_image links OpenCV directly for cv::imwrite
add_library(a2_bt_save_image_plugin SHARED plugins/save_image.cpp)
target_include_directories(a2_bt_save_image_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(a2_bt_save_image_plugin ${DEPENDENCIES})
target_link_libraries(a2_bt_save_image_plugin ${OpenCV_LIBS})
bt_plugin(a2_bt_call_trigger_service_plugin        plugins/call_trigger_service.cpp)
bt_plugin(a2_bt_start_exploration_plugin           plugins/start_exploration.cpp)
bt_plugin(a2_bt_wait_for_exploration_finish_plugin plugins/wait_for_exploration_finish.cpp)
bt_plugin(a2_bt_is_exploration_finished_plugin     plugins/is_exploration_finished.cpp)
bt_plugin(a2_bt_save_pose_plugin                   plugins/save_pose.cpp)
bt_plugin(a2_bt_stop_movement_plugin               plugins/stop_movement.cpp)
bt_plugin(a2_bt_stop_exploration_plugin            plugins/stop_exploration.cpp)
bt_plugin(a2_bt_save_map_plugin                    plugins/save_map.cpp)

# ── Standalone nodes ──────────────────────────────────────────────────────────
add_executable(detection_mapper src/detection_mapper.cpp)
target_include_directories(detection_mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(detection_mapper ${DEPENDENCIES})

# ── Executor node ─────────────────────────────────────────────────────────────
add_executable(bt_executor src/bt_executor.cpp)
target_include_directories(bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bt_executor ${DEPENDENCIES})
target_link_libraries(bt_executor
  a2_bt_create_pose_plugin
  a2_bt_navigate_to_pose_plugin
  a2_bt_publish_twist_plugin
  a2_bt_set_mode_plugin
  a2_bt_get_object_pose_plugin
  a2_bt_save_image_plugin
  a2_bt_call_trigger_service_plugin
  a2_bt_start_exploration_plugin
  a2_bt_wait_for_exploration_finish_plugin
  a2_bt_is_exploration_finished_plugin
  a2_bt_save_pose_plugin
  a2_bt_stop_movement_plugin
  a2_bt_stop_exploration_plugin
  a2_bt_save_map_plugin
)

install(TARGETS
  detection_mapper
  a2_bt_create_pose_plugin
  a2_bt_navigate_to_pose_plugin
  a2_bt_publish_twist_plugin
  a2_bt_set_mode_plugin
  a2_bt_get_object_pose_plugin
  a2_bt_save_image_plugin
  a2_bt_call_trigger_service_plugin
  a2_bt_start_exploration_plugin
  a2_bt_wait_for_exploration_finish_plugin
  a2_bt_is_exploration_finished_plugin
  a2_bt_save_pose_plugin
  a2_bt_save_map_plugin
  bt_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
