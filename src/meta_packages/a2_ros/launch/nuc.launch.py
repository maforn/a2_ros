"""
Full A2 real-robot launch.

Starts:
  - a2_unitree_bridge  : bridge node (publishes /joint_states and /imu/data from hardware)
  - hesai_ros_driver   : Hesai LiDAR driver (front + rear lidars)
  - joy_node           : reads gamepad from /dev/input/js0
  - teleop_joy         : maps gamepad axes/buttons to /joy_vel (via twist_mux) and /a2/mode
  - gscam2             : H.264 multicast camera stream

Always on:
  - robot_state_publisher : broadcasts fixed TF links from URDF

Optional (pass rviz:=true):
  - rviz2 : 3-D visualisation

Usage:
  ros2 launch a2_ros nuc.launch.py
  ros2 launch a2_ros nuc.launch.py rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('a2_description')
    bridge_launch_dir = get_package_share_directory('a2_unitree_bridge')
    a2_ros_launch_dir = os.path.join(get_package_share_directory('a2_ros'), 'launch')
    hesai_dir = get_package_share_directory('hesai_ros_driver')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 visualisation'
    )

    a2_ros_dir = get_package_share_directory('a2_ros')
    urdf_path = os.path.join(description_dir, 'urdf', 'a2.urdf')
    rviz_path = os.path.join(a2_ros_dir, 'rviz', 'robot.rviz')

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_launch_dir, 'launch', 'robot.launch.py')
        )
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(a2_ros_launch_dir, 'teleop_joy.launch.py')
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(a2_ros_launch_dir, 'camera.launch.py')
        )
    )

    front_lidar_node = Node(
        namespace='hesai_ros_driver_front',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': os.path.join(hesai_dir, 'config', 'config_front.yaml')}]
    )

    rear_lidar_node = Node(
        namespace='hesai_ros_driver_rear',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': os.path.join(hesai_dir, 'config', 'config_back.yaml')}]
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', urdf_path]), value_type=str
            ),
            'use_sim_time': False,
        }],
    )

    #IMU sits at [8.62, -9.14, -39.16] mm relative to the lidar frame.
    front_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_lidar_imu_tf',
        arguments=[
            '--x', '0.00862', '--y', '-0.00914', '--z', '-0.03916',
            '--frame-id', 'front_lidar_link', '--child-frame-id', 'front_imu_link',
        ],
    )

    rear_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rear_lidar_imu_tf',
        arguments=[
            '--x', '0.00862', '--y', '-0.00914', '--z', '-0.03916',
            '--frame-id', 'rear_lidar_link', '--child-frame-id', 'rear_imu_link',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        # bridge_launch,
        # teleop_launch,
        # camera_launch,
        front_lidar_node,
        rear_lidar_node,
        robot_state_pub_node,
        front_imu_tf_node,
        rear_imu_tf_node,
        rviz_node,
    ])
