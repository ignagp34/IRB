import pytest
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

from irb120pe_cognitive.action_adapter_node import (
    build_move_group_pose_goal,
    compute_pick_heights,
    compute_place_heights,
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


def test_compute_pick_heights_preserves_calibrated_tool_offset():
    pick_z, approach_z = compute_pick_heights(
        0.90,
        configured_pick_z=1.07,
        configured_pick_approach_z=1.10,
        pick_z_offset_from_object=0.17,
        pick_approach_offset_from_object=0.20,
    )
    assert pick_z == pytest.approx(1.07)
    assert approach_z == pytest.approx(1.10)


def test_compute_pick_heights_tracks_an_elevated_detected_object():
    pick_z, approach_z = compute_pick_heights(
        0.96,
        configured_pick_z=1.07,
        configured_pick_approach_z=1.10,
        pick_z_offset_from_object=0.17,
        pick_approach_offset_from_object=0.20,
    )
    assert pick_z == pytest.approx(1.13)
    assert approach_z == pytest.approx(1.16)


def test_compute_pick_heights_falls_back_without_valid_detection_height():
    pick_z, approach_z = compute_pick_heights(
        float("nan"),
        configured_pick_z=1.07,
        configured_pick_approach_z=1.10,
        pick_z_offset_from_object=0.17,
        pick_approach_offset_from_object=0.20,
    )
    assert pick_z == pytest.approx(1.07)
    assert approach_z == pytest.approx(1.10)


def test_compute_place_heights_converts_object_target_to_tool0_goals():
    place_z, approach_z = compute_place_heights(
        0.90,
        place_z_offset_from_object=0.18,
        place_approach_offset_z=0.10,
    )
    assert place_z == pytest.approx(1.08)
    assert approach_z == pytest.approx(1.18)


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
