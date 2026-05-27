"""Single-command launch for the goal-oriented arrangement demo.

Wraps `cognitive_demo.launch.py` with the defaults required by the RI_26
arrangement scenario: real MoveIt execution in Gazebo, mock LLM provider for
deterministic videos/CI, and explicit Gazebo cube attachment. Override
`llm_provider:=openrouter llm_model:=anthropic/claude-3.5-sonnet` to drive the
exact same demo with a real LLM (an `OPENROUTER_API_KEY` must be exported).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cognitive_launch = os.path.join(
        get_package_share_directory("irb120pe_cognitive"),
        "launch",
        "cognitive_demo.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("llm_provider", default_value="mock"),
            DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("execution_backend", default_value="moveit_sim"),
            DeclareLaunchArgument("spawn_timeout", default_value="120.0"),
            DeclareLaunchArgument("start_legacy_interfaces", default_value="false"),
            DeclareLaunchArgument("rviz_file", default_value="True"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cognitive_launch),
                launch_arguments={
                    "llm_provider": LaunchConfiguration("llm_provider"),
                    "llm_model": LaunchConfiguration("llm_model"),
                    "dry_run": LaunchConfiguration("dry_run"),
                    "execution_backend": LaunchConfiguration("execution_backend"),
                    "spawn_timeout": LaunchConfiguration("spawn_timeout"),
                    "start_legacy_interfaces": LaunchConfiguration("start_legacy_interfaces"),
                    "rviz_file": LaunchConfiguration("rviz_file"),
                }.items(),
            ),
        ]
    )
