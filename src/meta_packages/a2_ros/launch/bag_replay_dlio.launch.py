"""
Replay a real-robot rosbag through DLIO + ALO planning stack.

Plays back raw lidar + IMU from the bag, runs DLIO to recompute odometry,
then feeds the result into the full navigation/exploration stack
(terrain_analysis, far_planner, local_planner, ALO).

Dynamic /tf from the bag is excluded so it doesn't conflict with DLIO's
freshly-computed transforms. /tf_static is kept (static robot-model links).

Usage:
  ros2 launch a2_ros bag_replay_dlio.launch.py bag:=/path/to/bag
  ros2 launch a2_ros bag_replay_dlio.launch.py bag:=my_run  # resolves under $ROS_BAGS_DIR
  ros2 launch a2_ros bag_replay_dlio.launch.py bag:=/path/to/bag rviz:=true rate:=0.5
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    PopLaunchConfigurations,
    PushLaunchConfigurations,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def play_bag_setup(context, *args, **kwargs):
    bag_path = context.perform_substitution(LaunchConfiguration('bag'))
    if not bag_path:
        return []

    workspace_dir = '/a2_ros'
    bag_dir = os.environ.get('ROS_BAGS_DIR', os.path.join(workspace_dir, 'bags'))

    resolved_path = bag_path
    if not os.path.exists(bag_path):
        candidate = os.path.join(bag_dir, bag_path)
        if os.path.exists(candidate):
            resolved_path = candidate

    cmd = ['ros2', 'bag', 'play', resolved_path, '--clock']

    # Exclude /tf: DLIO will recompute it fresh; the bag's recorded /tf
    # (map→body from the original run) would conflict with the new instance.
    # /tf_static is kept so static robot-model links are available.
    cmd += ['--exclude-topics', '/tf']

    rate = context.perform_substitution(LaunchConfiguration('rate'))
    if rate and rate != '1.0':
        cmd.extend(['--rate', rate])

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    description_dir   = get_package_share_directory('a2_description')
    a2_ros_dir        = get_package_share_directory('a2_ros')
    a2_ros_launch_dir = os.path.join(a2_ros_dir, 'launch')
    urdf_path         = os.path.join(description_dir, 'urdf', 'a2.urdf')
    rviz_path         = os.path.join(a2_ros_dir, 'rviz', 'exploration.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('bag',  default_value='',
                              description='Bag path or name under $ROS_BAGS_DIR'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Playback rate (0.5 = half speed)'),

        # All nodes use bag time published by ros2 bag play --clock.
        SetParameter(name='use_sim_time', value=True),

        # Robot model: publishes static joint TFs (base_link → links).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_path]), value_type=str
                ),
                'use_sim_time': True,
            }],
        ),

        # ── DLIO: recompute odometry from raw lidar + IMU ───────────────────
        # IMU calibration is auto-disabled when use_sim_time=true (dlio.launch.py).
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(a2_ros_launch_dir, 'dlio.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz':         'false',
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ── ALO + terrain_analysis + far_planner + local_planner ────────────
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(a2_ros_launch_dir, 'navigate_and_explore.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz':         'false',
                'planner':      'alo',
            }.items(),
        ),
        PopLaunchConfigurations(),

        # ── RViz ────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # ── Bag playback ─────────────────────────────────────────────────────
        OpaqueFunction(function=play_bag_setup),
    ])
