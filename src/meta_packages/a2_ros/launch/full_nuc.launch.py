"""
Full real-robot mission stack — nuc + RESPLE + navigate_and_explore + object detection + BT executor.

Starts:
  - nuc.launch.py                   : LiDAR driver + robot_state_publisher + TF
  - resple.launch.py                : LiDAR-inertial odometry + MapSaving node
  - navigate_and_explore.launch.py  : TARE + far_planner + detection_mapper
  - object_detection_real.launch.py : YOLO object detection (real robot variant)
  - bt_executor.launch.py           : BT action server

Usage:
  ros2 launch a2_ros full_nuc.launch.py
  ros2 launch a2_ros full_nuc.launch.py rviz:=true

  a2 nuc-full
  a2 nuc-full --rviz
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
    a2_ros_launch_dir  = os.path.join(get_package_share_directory('a2_ros'), 'launch')
    a2_bt_launch_dir   = os.path.join(get_package_share_directory('a2_bt'), 'launch')
    detect_launch_dir  = os.path.join(get_package_share_directory('object_detection'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('rviz',         default_value='false'),
        DeclareLaunchArgument('lidar_config', default_value='config_front.yaml'),

        # ---- NUC hardware (LiDAR driver + robot_state_publisher + TF) ----
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(a2_ros_launch_dir, 'nuc.launch.py')),
            launch_arguments={
                'lidar_config': LaunchConfiguration('lidar_config'),
                'rviz':         'false',
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ---- RESPLE odometry + MapSaving node ----
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

        # ---- TARE + far_planner + detection_mapper ----
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(a2_ros_launch_dir, 'navigate_and_explore.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'rviz':         LaunchConfiguration('rviz'),
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ---- Object detection (real robot variant) ----
        # PushLaunchConfigurations(),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(detect_launch_dir, 'object_detection_real.launch.py')
        #     ),
        # ),
        # PopLaunchConfigurations(),

        # ---- BT executor ----
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(a2_bt_launch_dir, 'bt_executor.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        ),
        PopLaunchConfigurations(),
    ])
