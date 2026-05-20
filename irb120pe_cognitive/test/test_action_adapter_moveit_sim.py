import pytest
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

from irb120pe_cognitive.action_adapter_node import (
    build_move_group_pose_goal,
    normalize_execution_backend,
)
from irb120pe_cognitive.validation import ValidationError


def _pose() -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = 0.2
    pose.pose.position.y = 0.2
    pose.pose.position.z = 1.1
    pose.pose.orientation.x = 0.707
    pose.pose.orientation.y = 0.707
    pose.pose.orientation.w = 0.0
    return pose


def test_normalize_execution_backend_accepts_supported_names():
    assert normalize_execution_backend("legacy") == "legacy"
    assert normalize_execution_backend("MOVEIT_SIM") == "moveit_sim"


def test_normalize_execution_backend_rejects_unknown_name():
    with pytest.raises(ValidationError, match="execution_backend"):
        normalize_execution_backend("hardware")


def test_build_move_group_pose_goal_uses_pose_constraints_for_tool0():
    goal = build_move_group_pose_goal(
        _pose(),
        group_name="irb120_arm",
        end_effector_link="tool0",
        planner_id="PTP",
        planning_time=10.0,
        velocity_scaling=0.05,
        acceleration_scaling=0.05,
        position_tolerance=0.01,
        orientation_tolerance=0.25,
        collision_objects_to_remove=["blue_1"],
    )

    assert goal.request.group_name == "irb120_arm"
    assert goal.request.planner_id == "PTP"
    assert goal.request.max_velocity_scaling_factor == pytest.approx(0.05)
    assert goal.planning_options.plan_only is False
    constraints = goal.request.goal_constraints[0]
    position = constraints.position_constraints[0]
    orientation = constraints.orientation_constraints[0]
    assert position.header.frame_id == "world"
    assert position.link_name == "tool0"
    assert position.constraint_region.primitives[0].type == SolidPrimitive.BOX
    assert list(position.constraint_region.primitives[0].dimensions) == pytest.approx([0.01, 0.01, 0.01])
    assert orientation.link_name == "tool0"
    assert orientation.absolute_x_axis_tolerance == pytest.approx(0.25)
    assert goal.planning_options.planning_scene_diff.world.collision_objects[0].id == "blue_1"
