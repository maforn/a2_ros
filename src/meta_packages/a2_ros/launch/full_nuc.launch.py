"""
Full real-robot mission stack — NUC.

Starts:
  - nuc.launch.py                   : LiDAR driver + robot_state_publisher + TF
  - resple.launch.py                : RESPLE LiDAR-inertial odometry + MapSaving
  - navigate_and_explore.launch.py  : ALO + terrain_analysis + far_planner +
                                      local_planner + pathFollower + detection_mapper
  - object_detection_real.launch.py : YOLO + camera (same as `a2 detect` on robot)
  - bt_executor.launch.py           : BT action server

Topic wiring (detection):
  object_detection_node → /detection_info  →  detection_mapper → /detected_objects

Usage:
  a2 nuc-full
  a2 nuc-full --rviz
  ros2 launch a2_ros full_nuc.launch.py rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    PopLaunchConfigurations,
    PushLaunchConfigurations,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    a2_ros_launch_dir = os.path.join(get_package_share_directory('a2_ros'), 'launch')
    a2_bt_launch_dir  = os.path.join(get_package_share_directory('a2_bt'), 'launch')
    detect_launch_dir = os.path.join(get_package_share_directory('object_detection'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('rviz',         default_value='false'),
        DeclareLaunchArgument('lidar_config', default_value='config_front.yaml'),

        # ── NUC hardware (LiDAR driver + robot_state_publisher + TF) ────────
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(a2_ros_launch_dir, 'nuc.launch.py')),
            launch_arguments={
                'lidar_config': LaunchConfiguration('lidar_config'),
                'rviz':         'false',
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ── RESPLE LiDAR-inertial odometry + MapSaving ───────────────────────
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(a2_ros_launch_dir, 'resple.launch.py')),
            launch_arguments={
                'use_sim_time':    'false',
                'rviz':            'false',
                'map_saving_node': 'true',
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ── ALO + terrain_analysis + far_planner + local_planner + detection_mapper ──
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(a2_ros_launch_dir, 'navigate_and_explore.launch.py')
            ),
            launch_arguments={
                'use_sim_time':  'false',
                'rviz':          LaunchConfiguration('rviz'),
                'planner':       'alo',
                'resple_scan':   'true',
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ── Object detection — identical to `a2 detect` on robot ─────────────
        # object_detection_real.launch.py defaults:
        #   input_camera_name: /camera   → subscribes /camera/image_raw + /camera/camera_info
        #   debayer_image:     false      (camera already outputs RGB, no Bayer decode needed)
        #   lidar_topic:       /front_lidar/points
        #   gpu:               off        (NUC is CPU-only)
        # Output: /detection_info  →  consumed by detection_mapper (in navigate_and_explore)
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(detect_launch_dir, 'object_detection_real.launch.py')
            ),
        ),
        PopLaunchConfigurations(),

        # ── BT action server ─────────────────────────────────────────────────
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(a2_bt_launch_dir, 'bt_executor.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        ),
        PopLaunchConfigurations(),
    ])
