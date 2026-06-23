"""
Exploration + navigation launch for A2 — extends exploration.launch.py with far_planner.

Combines the full TARE exploration stack with far_planner so that BT trees that
explore and then navigate home (e.g. ObjectExploration) work in a single launch.

Starts:
  - terrain_analysis     : /terrain_map
  - terrain_analysis_ext : /terrain_map_ext (shared by TARE and far_planner)
  - local_planner        : obstacle-aware path selection
  - pathFollower         : waypoint → /nav_vel
  - tare_planner         : autonomous coverage exploration
  - far_planner          : global visibility-graph planner (for NavigateToPose home return)
  - detection_mapper     : accumulates YOLO detections into deduplicated map-frame list

Prerequisites (provided by sim.launch.py + a2_bridge):
  /state_estimation  - ground-truth odometry
  /registered_scan   - world-frame lidar cloud
  /clock             - sim time clock

Usage:
  # Terminal 1
  ros2 launch a2_ros sim.launch.py scene:=scene_obstacles.xml

  # Terminal 2
  ros2 launch a2_ros navigate_and_explore.launch.py rviz:=true

  # Terminal 3 — run a BT tree that uses exploration + home navigation
  a2 tree object_exploration
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    description_dir = get_package_share_directory('a2_description')
    a2_ros_dir      = get_package_share_directory('a2_ros')
    rviz_path        = os.path.join(a2_ros_dir, 'rviz', 'exploration.rviz')
    tare_config      = os.path.join(a2_ros_dir, 'config', 'autonomy', 'tare_a2.yaml')
    far_config       = os.path.join(a2_ros_dir, 'config', 'autonomy', 'far_a2.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (sim time)'
    )

    nodes = [
        rviz_arg,
        use_sim_time_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # ---- terrain analysis (local map) ----
        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[{
                'scanVoxelSize':       0.1,    # 0.05→0.1: coarser voxels reduce noise hits
                'decayTime':           10.0,
                'noDecayDis':          0.0,
                'clearingDis':         8.0,
                'useSorting':          True,
                'quantileZ':           0.25,
                'considerDrop':        True,
                'limitGroundLift':     True,
                'maxGroundLift':       0.25,
                'clearDyObs':          False,
                'minDyObsDis':         0.3,
                'minDyObsAngle':       0.0,
                'minDyObsRelZ':        -0.5,
                'absDyObsRelZThre':    0.2,
                'minDyObsVFOV':        -16.0,
                'maxDyObsVFOV':        16.0,
                'minDyObsPointNum':    1,
                'noDataObstacle':      False,
                'noDataBlockSkipNum':  0,
                'minBlockPointNum':    20,   # 10→20: need more points to declare obstacle cell
                'vehicleHeight':       0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 2.0,
                'minRelZ':             -1.0,
                'maxRelZ':             1.5,  # 1.0→1.5: pass gate beams up to 1.5 m to local_planner
                'disRatioZ':           0.2,
            }],
        ),

        # ---- terrain analysis ext (global map — shared by TARE and far_planner) ----
        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrainAnalysisExt',
            output='screen',
            parameters=[{
                'scanVoxelSize':        0.1,
                'decayTime':            10.0,
                'noDecayDis':           0.0,
                'clearingDis':          30.0,
                'useSorting':           True,
                'quantileZ':            0.25,
                'vehicleHeight':        0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre':  2.0,
                'lowerBoundZ':          -1.0,
                'upperBoundZ':          1.5,  # 1.0→1.5: consistent with terrain_analysis maxRelZ
                'disRatioZ':            0.1,
                'checkTerrainConn':     True,
                'terrainUnderVehicle':  -0.75,
                'terrainConnThre':      0.5,
                'ceilingFilteringThre': 2.0,
                'localTerrainMapRadius': 4.0,
            }],
        ),

        # ---- local planner ----
        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            parameters=[{
                'pathFolder':          get_package_share_directory('local_planner') + '/paths',
                'vehicleLength':       0.65,
                'vehicleWidth':        0.40,
                'sensorOffsetX':       0.0,
                'sensorOffsetY':       0.0,
                'twoWayDrive':         False,
                'laserVoxelSize':      0.05,
                'terrainVoxelSize':    0.2,
                'useTerrainAnalysis':  True,
                'checkObstacle':       True,
                'checkRotObstacle':    True,
                'adjacentRange':       3.5,
                'obstacleHeightThre':  0.30,  # 0.25→0.30: ignore bumps under 30 cm
                'groundHeightThre':    0.1,
                'costHeightThre':      0.1,
                'costScore':           0.02,
                'useCost':             False,
                'pointPerPathThre':    3,    # 2→3: need 3 blocked points per path (less noise)
                'minRelZ':             -0.5,
                'maxRelZ':             1.2,  # 0.8→1.2: detect obstacles/gate beams up to 1.2 m
                'maxSpeed':            0.5,
                'dirWeight':           0.1,
                'dirThre':             90.0,
                'dirToVehicle':        False,
                'pathScale':           1.0,
                'minPathScale':        0.75,
                'pathScaleStep':       0.25,
                'pathScaleBySpeed':    True,
                'minPathRange':        1.0,
                'pathRangeStep':       0.5,
                'pathRangeBySpeed':    True,
                'pathCropByGoal':      True,
                'autonomyMode':        True,
                'autonomySpeed':       2.0,
                'joyToSpeedDelay':     2.0,
                'joyToCheckObstacleDelay': 5.0,
                'goalClearRange':      0.4,
                'goalX':               0.0,
                'goalY':               0.0,
            }],
        ),

        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[{
                'sensorOffsetX':    0.0,
                'sensorOffsetY':    0.0,
                'pubSkipNum':       1,
                'twoWayDrive':      False,
                'lookAheadDis':     0.4,
                'yawRateGain':      5.0,
                'stopYawRateGain':  4.0,
                'maxYawRate':       30.0,
                'maxSpeed':         0.5,
                'maxAccel':         2.0,
                'switchTimeThre':   1.0,
                'dirDiffThre':      0.1,
                'stopDisThre':      0.3,
                'slowDwnDisThre':   0.6,
                'useInclRateToSlow': False,
                'inclRateThre':     120.0,
                'slowRate1':        0.25,
                'slowRate2':        0.5,
                'slowTime1':        2.0,
                'slowTime2':        2.0,
                'useInclToStop':    False,
                'inclThre':         45.0,
                'stopTime':         5.0,
                'noRotAtStop':      False,
                'noRotAtGoal':      True,
                'autonomyMode':     True,
                'autonomySpeed':    2.0,
                'joyToSpeedDelay':  2.0,
            }],
        ),

        # ---- TARE planner (autonomous exploration) ----
        Node(
            package='tare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='screen',
            parameters=[tare_config],
        ),

        # ---- far_planner (global path planner — used by NavigateToPose to return home) ----
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

        # ---- Detection mapper ----
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

        # ---- RViz ----
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
