import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("irb120pe_cognitive"), "config", "cognitive.yaml")
    moveit_launch = os.path.join(get_package_share_directory("irb120pe_moveit2"), "launch", "moveit2.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("llm_provider", default_value="mock"),
            DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("execution_backend", default_value="moveit_sim"),
            DeclareLaunchArgument("spawn_timeout", default_value="120.0"),
            DeclareLaunchArgument("start_legacy_interfaces", default_value="false"),
            DeclareLaunchArgument("rviz_file", default_value="True"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={
                    "spawn_timeout": LaunchConfiguration("spawn_timeout"),
                    "start_legacy_interfaces": LaunchConfiguration("start_legacy_interfaces"),
                    "rviz_file": LaunchConfiguration("rviz_file"),
                }.items(),
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="irb120pe_cognitive",
                        executable="perception_node",
                        name="irb120pe_cognitive_perception",
                        output="screen",
                        parameters=[config],
                    ),
                    Node(
                        package="irb120pe_cognitive",
                        executable="planning_scene_sync_node",
                        name="irb120pe_planning_scene_sync",
                        output="screen",
                        parameters=[config],
                    ),
                    Node(
                        package="irb120pe_cognitive",
                        executable="action_adapter_node",
                        name="irb120pe_action_adapter",
                        output="screen",
                        parameters=[
                            config,
                            {
                                "dry_run": LaunchConfiguration("dry_run"),
                                "execution_backend": LaunchConfiguration("execution_backend"),
                            },
                        ],
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
                ],
            ),
        ]
    )
