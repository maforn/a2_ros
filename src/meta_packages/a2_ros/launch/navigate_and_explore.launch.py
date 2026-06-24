"""
Exploration + navigation launch for A2.

Combines the exploration stack with far_planner so that BT trees that
explore and then navigate home work in a single launch.

Starts:
  - terrain_analysis     : /terrain_map
  - terrain_analysis_ext : /terrain_map_ext (shared by planner and far_planner)
  - local_planner        : obstacle-aware path selection
  - pathFollower         : waypoint → /nav_vel
  - planner              : autonomous exploration (tare | frontier, see planner arg)
  - far_planner          : global visibility-graph planner (for NavigateToPose home return)
  - detection_mapper     : accumulates YOLO detections into deduplicated map-frame list

  planner:=tare      TARE coverage planner (default)
  planner:=frontier  Simple 2-D frontier explorer — more reliable in bounded enclosures

Parameters loaded from config/autonomy/navigation_a2.yaml, tare_a2.yaml, far_a2.yaml.

Topic split:
  /registered_scan  — RESPLE's world-frame cloud (downsampled); used by terrain_analysis
                      and far_planner.
  /alo/scan         — registered_scan_pub full-resolution transform of /front_lidar/points;
                      used only by ALO so it controls its own voxel downsampling.

Prerequisites:
  /state_estimation  - odometry (RESPLE → frame_id=map)
  /registered_scan   - world-frame lidar cloud (RESPLE)

Usage:
  ros2 launch a2_ros navigate_and_explore.launch.py rviz:=true planner:=frontier
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    a2_ros_dir   = get_package_share_directory('a2_ros')
    rviz_path    = os.path.join(a2_ros_dir, 'rviz', 'exploration.rviz')
    nav_config   = os.path.join(a2_ros_dir, 'config', 'autonomy', 'navigation_a2.yaml')
    tare_config  = os.path.join(a2_ros_dir, 'config', 'autonomy', 'tare_a2.yaml')
    far_config   = os.path.join(a2_ros_dir, 'config', 'autonomy', 'far_a2.yaml')
    alo_config   = os.path.join(a2_ros_dir, 'config', 'autonomy', 'alo_a2.yaml')
    path_folder  = get_package_share_directory('local_planner') + '/paths'

    is_tare = IfCondition(PythonExpression(["'", LaunchConfiguration('planner'), "' == 'tare'"]))
    is_alo  = IfCondition(PythonExpression(["'", LaunchConfiguration('planner'), "' == 'alo'"]))

    nodes = [
        DeclareLaunchArgument('rviz',         default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('planner',      default_value='alo',
                              description='Exploration planner: "tare" or "alo"'),
        # In sim the TF is always current (bridge publishes it synchronously), so
        # tf_lag_sec should be ~0. On the real robot RESPLE has ~200ms TF lag so
        # 0.25s keeps the lookup safely behind the available TF history.
        # Pass tf_lag_sec:=0.0 from full_sim.launch.py, 0.25 from full_nuc.launch.py.
        DeclareLaunchArgument('tf_lag_sec',   default_value='0.25',
                              description='TF lookup lag for registered_scan_pub (s). '
                                          '0.0 for sim, ~0.25 for real robot with RESPLE.'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # Transforms raw /front_lidar/points → /alo/scan (map frame, full resolution).
        # Separate from /registered_scan so RESPLE's downsampled cloud stays intact
        # for terrain_analysis and far_planner. ALO uses /alo/scan and applies its
        # own voxel filter.
        Node(
            package='a2_utils',
            executable='registered_scan_pub',
            name='registered_scan_pub',
            output='screen',
            parameters=[{
                'input_topic':  '/front_lidar/points',
                'target_frame': 'map',
                'tf_lag_sec':   LaunchConfiguration('tf_lag_sec'),
            }],
            remappings=[('/registered_scan', '/alo/scan')],
        ),

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

        # ---- TARE planner (planner:=tare) ----
        Node(
            package='tare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='screen',
            parameters=[tare_config],
            condition=is_tare,
        ),

        # ---- ALO exploration planner (planner:=alo) ----
        Node(
            package='a2_ros',
            executable='alo',
            name='alo',
            output='screen',
            parameters=[alo_config],
            condition=is_alo,
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
