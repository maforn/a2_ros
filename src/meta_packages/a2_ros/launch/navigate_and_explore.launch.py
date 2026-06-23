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

Prerequisites:
  /state_estimation  - odometry
  /registered_scan   - world-frame lidar cloud

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
    path_folder  = get_package_share_directory('local_planner') + '/paths'

    is_tare = IfCondition(PythonExpression(["'", LaunchConfiguration('planner'), "' == 'tare'"]))
    is_alo  = IfCondition(PythonExpression(["'", LaunchConfiguration('planner'), "' == 'alo'"]))

    nodes = [
        DeclareLaunchArgument('rviz',         default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('planner',      default_value='alo',
                              description='Exploration planner: "tare" or "alo"'),
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
            executable='alo.py',
            name='alo',
            output='screen',
            parameters=[{
                'resolution':          0.25,   # m/cell — 0.6 m door = 2.4 cells, don't go finer
                'grid_half_width':     80.0,   # m each side → 640×640 grid total
                'z_min_rel':          -0.3,
                'z_max_rel':           1.5,
                'max_ray_range':       10.0,
                'reach_dist':          1.5,
                'robot_clear_radius':  0.3,
                'nav_clearance':       0.25,   # m — min wall clearance for NBV candidates
                'wp_timeout':         30.0,    # s — blacklist wp if not reached in time
                'planning_hz':         1.0,
                'wp_publish_hz':       2.0,
                'done_timeout':        8.0,
            }],
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
