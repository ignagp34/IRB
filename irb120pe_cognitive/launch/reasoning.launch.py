import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("irb120pe_cognitive"), "config", "cognitive.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("llm_provider", default_value="mock"),
            DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            Node(
                package="irb120pe_cognitive",
                executable="action_adapter_node",
                name="irb120pe_action_adapter",
                output="screen",
                parameters=[config, {"dry_run": LaunchConfiguration("dry_run")}],
            ),
            Node(
                package="irb120pe_cognitive",
                executable="langchain_reasoning_node",
                name="irb120pe_langchain_reasoning",
                output="screen",
                parameters=[
                    config,
                    {
                        "llm_provider": LaunchConfiguration("llm_provider"),
                        "llm_model": LaunchConfiguration("llm_model"),
                    },
                ],
            ),
        ]
    )
