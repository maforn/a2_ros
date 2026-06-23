"""
Navigation stack launch for A2.

Starts the CMU Autonomous Exploration stack:
  - terrain_analysis     : builds /terrain_map from /registered_scan + /state_estimation
  - terrain_analysis_ext : builds /terrain_map_ext (global terrain for far_planner)
  - local_planner        : obstacle-aware path selection + path follower
  - far_planner          : global visibility-graph planner

Parameters loaded from config/autonomy/navigation_a2.yaml and far_a2.yaml.

Prerequisites:
  /state_estimation  - odometry
  /registered_scan   - world-frame lidar cloud

Usage:
  ros2 launch a2_ros navigation.launch.py
  ros2 launch a2_ros navigation.launch.py rviz:=true use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    a2_ros_dir   = get_package_share_directory('a2_ros')
    rviz_path    = os.path.join(a2_ros_dir, 'rviz', 'navigation.rviz')
    nav_config   = os.path.join(a2_ros_dir, 'config', 'autonomy', 'navigation_a2.yaml')
    far_config   = os.path.join(a2_ros_dir, 'config', 'autonomy', 'far_a2.yaml')
    path_folder  = get_package_share_directory('local_planner') + '/paths'

    nodes = [
        DeclareLaunchArgument('rviz',         default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[nav_config],
        ),

        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrainAnalysisExt',
            output='screen',
            parameters=[nav_config],
        ),

        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            parameters=[nav_config, {'pathFolder': path_folder}],
        ),

        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[nav_config],
        ),

        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='log',
            additional_env={'QT_QPA_PLATFORM': 'offscreen'},
            parameters=[far_config],
            remappings=[
                ('/odom_world',          '/state_estimation'),
                ('/terrain_cloud',       '/terrain_map_ext'),
                ('/scan_cloud',          '/registered_scan'),
                ('/terrain_local_cloud', '/terrain_map'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]

    return LaunchDescription(nodes)
