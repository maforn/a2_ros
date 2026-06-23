"""
Exploration + navigation launch for A2.

Combines the full TARE exploration stack with far_planner so that BT trees that
explore and then navigate home work in a single launch.

Starts:
  - terrain_analysis     : /terrain_map
  - terrain_analysis_ext : /terrain_map_ext (shared by TARE and far_planner)
  - local_planner        : obstacle-aware path selection
  - pathFollower         : waypoint → /nav_vel
  - tare_planner         : autonomous coverage exploration
  - far_planner          : global visibility-graph planner (for NavigateToPose home return)
  - detection_mapper     : accumulates YOLO detections into deduplicated map-frame list

Parameters loaded from config/autonomy/navigation_a2.yaml, tare_a2.yaml, far_a2.yaml.

Prerequisites:
  /state_estimation  - odometry
  /registered_scan   - world-frame lidar cloud

Usage:
  ros2 launch a2_ros navigate_and_explore.launch.py rviz:=true
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
    rviz_path    = os.path.join(a2_ros_dir, 'rviz', 'exploration.rviz')
    nav_config   = os.path.join(a2_ros_dir, 'config', 'autonomy', 'navigation_a2.yaml')
    tare_config  = os.path.join(a2_ros_dir, 'config', 'autonomy', 'tare_a2.yaml')
    far_config   = os.path.join(a2_ros_dir, 'config', 'autonomy', 'far_a2.yaml')
    path_folder  = get_package_share_directory('local_planner') + '/paths'

    nodes = [
        DeclareLaunchArgument('rviz',         default_value='true'),
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
            package='tare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='screen',
            parameters=[tare_config],
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
            package='a2_bt',
            executable='detection_mapper',
            name='detection_mapper',
            output='screen',
            parameters=[{
                'detection_topic': '/detection_info',
                'output_frame':    'map',
                'cluster_radius':  1.0,
                'min_confidence':  0.4,
                'detection_hz':    2.0,
                'marker_ns':       'detected_objects',
                'csv_path':        '/tmp/detections.csv',
            }],
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
