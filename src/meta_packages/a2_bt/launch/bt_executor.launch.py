from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('a2_bt'),
        'config',
        'bt_executor.yaml',
    ])

    return LaunchDescription([
        Node(
            package='a2_bt',
            executable='bt_executor',
            name='bt_action_server',
            output='screen',
            parameters=[config],
        ),
    ])
