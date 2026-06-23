from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('a2_bt'),
        'config',
        'bt_executor.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='a2_bt',
            executable='bt_executor',
            name='bt_action_server',
            output='screen',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
