import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("irb120pe_cognitive"), "config", "cognitive.yaml")
    return LaunchDescription(
        [
            Node(
                package="irb120pe_cognitive",
                executable="planning_scene_sync_node",
                name="irb120pe_planning_scene_sync",
                output="screen",
                parameters=[config],
            )
        ]
    )
