"""
Consumer-side camera decompress + rectify.

The camera node (gscam2) and an image_transport republish node run on PC2 and
publish:
  - <camera>/image_raw             raw rgb8 (H.264 already decoded by gscam2)
  - <camera>/image_raw/compressed  JPEG (quality 60), the small stream shipped
                                   over Zenoh to the NUC
  - <camera>/camera_info           intrinsics + plumb_bob distortion

This launch runs image_proc::RectifyNode on the consumer side (the NUC). Because
RectifyNode subscribes through image_transport, setting 'image_transport' to
'compressed' makes the single node JPEG-decode <camera>/image_raw/compressed and
undistort it (using <camera>/camera_info) in one step, publishing:
  - <camera>/image_rect            decompressed + rectified rgb8

object_detection_real.launch.py defaults its input to <camera>/image_rect, so
bringing this up alongside `a2 detect` feeds the detector the rectified stream.

Usage:
  ros2 launch a2_ros rectify.launch.py
  ros2 launch a2_ros rectify.launch.py input_camera_name:=/camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    input_camera_name = DeclareLaunchArgument(
        "input_camera_name",
        default_value="/camera",
        description="Camera topic prefix (image stream + camera_info live under it)",
    )

    rectify_container = ComposableNodeContainer(
        name="image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
                # Pull the compressed transport, not raw — decode happens in-node.
                parameters=[{"image_transport": "compressed"}],
                remappings=[
                    ("image", PathJoinSubstitution(
                        [LaunchConfiguration("input_camera_name"), "image_raw"]
                    )),
                    ("camera_info", PathJoinSubstitution(
                        [LaunchConfiguration("input_camera_name"), "camera_info"]
                    )),
                    ("image_rect", PathJoinSubstitution(
                        [LaunchConfiguration("input_camera_name"), "image_rect"]
                    )),
                ],
            ),
        ],
    )

    return LaunchDescription([
        input_camera_name,
        rectify_container,
    ])
