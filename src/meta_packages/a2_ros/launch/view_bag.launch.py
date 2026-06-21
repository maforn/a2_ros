import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
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

    cmd = ['ros2', 'bag', 'play', resolved_path]

    clock = context.perform_substitution(LaunchConfiguration('clock'))
    pause = context.perform_substitution(LaunchConfiguration('pause'))

    if clock.lower() in ['true', '1']:
        cmd.append('--clock')
    if pause.lower() in ['true', '1']:
        cmd.append('--pause')

    return [
        ExecuteProcess(
            cmd=cmd,
            output='screen'
        )
    ]


def generate_launch_description():
    description_dir = get_package_share_directory('a2_description')
    a2_ros_dir = get_package_share_directory('a2_ros')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 visualisation'
    )

    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value='',
        description='Path to the rosbag to play'
    )

    clock_arg = DeclareLaunchArgument(
        'clock',
        default_value='true',
        description='Publish clock during bag playback'
    )

    pause_arg = DeclareLaunchArgument(
        'pause',
        default_value='false',
        description='Start bag playback paused'
    )

    urdf_path = os.path.join(description_dir, 'urdf', 'a2.urdf')
    rviz_path = os.path.join(a2_ros_dir, 'rviz', 'default.rviz')

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', urdf_path]), value_type=str
            ),
            'use_sim_time': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        bag_arg,
        clock_arg,
        pause_arg,
        robot_state_pub_node,
        rviz_node,
        OpaqueFunction(function=play_bag_setup),
    ])
