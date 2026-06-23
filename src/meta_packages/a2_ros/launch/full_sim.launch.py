"""
Full simulation mission stack — sim + DLIO + navigate_and_explore + BT executor.

Starts:
  - sim.launch.py            : MuJoCo + locomotion controller (dlio:=true mode —
                               a2_bridge publishes IMU/joints only, DLIO provides odometry)
  - dlio.launch.py           : LiDAR-inertial odometry (use_sim_time=true)
  - navigate_and_explore.launch.py : TARE + far_planner + detection_mapper
  - bt_executor.launch.py    : BT action server

Usage:
  ros2 launch a2_ros full_sim.launch.py
  ros2 launch a2_ros full_sim.launch.py scene:=scene_obstacles.xml rviz:=true

  a2 sim-full
  a2 sim-full --rviz --scene scene_obstacles.xml --headless
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

    return LaunchDescription([
        DeclareLaunchArgument('rviz',      default_value='true'),
        DeclareLaunchArgument('scene',     default_value='scene_maze.xml'),
        DeclareLaunchArgument('headless',  default_value='false'),

        # ---- MuJoCo sim in DLIO mode (a2_bridge publishes IMU/joints; DLIO provides /state_estimation) ----
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(a2_ros_launch_dir, 'sim.launch.py')),
            launch_arguments={
                'scene':    LaunchConfiguration('scene'),
                'headless': LaunchConfiguration('headless'),
                'rviz':     'false',
                'dlio':     'true',   # suppress ground-truth TF; DLIO provides odometry
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ---- DLIO odometry ----
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(a2_ros_launch_dir, 'dlio.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz':         'false',
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
                'use_sim_time': 'true',
                'rviz':         LaunchConfiguration('rviz'),
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ---- BT executor ----
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(a2_bt_launch_dir, 'bt_executor.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
            }.items(),
        ),
        PopLaunchConfigurations(),
    ])
