from __future__ import annotations

from typing import Any

from geometry_msgs.msg import PoseStamped

from .defaults import DEFAULT_SLOTS, DEFAULT_WORKSPACE_LIMITS
from .validation import Slot, load_slots, load_workspace_limits


def declare_common_parameters(node: Any) -> None:
    node.declare_parameter("planning_frame", "world")
    node.declare_parameter("workspace_limits.x", list(DEFAULT_WORKSPACE_LIMITS["x"]))
    node.declare_parameter("workspace_limits.y", list(DEFAULT_WORKSPACE_LIMITS["y"]))
    node.declare_parameter("workspace_limits.z", list(DEFAULT_WORKSPACE_LIMITS["z"]))
    for slot_key, slot in DEFAULT_SLOTS.items():
        node.declare_parameter(f"slots.{slot_key}.label", slot["label"])
        node.declare_parameter(f"slots.{slot_key}.aliases", slot["aliases"])
        node.declare_parameter(f"slots.{slot_key}.pose", slot["pose"])


def get_workspace_limits(node: Any) -> dict[str, tuple[float, float]]:
    raw = {
        axis: node.get_parameter(f"workspace_limits.{axis}").value
        for axis in ("x", "y", "z")
    }
    return load_workspace_limits(raw)


def get_slots(node: Any) -> dict[str, Slot]:
    raw = {}
    for slot_key in DEFAULT_SLOTS:
        raw[slot_key] = {
            "label": node.get_parameter(f"slots.{slot_key}.label").value,
            "aliases": node.get_parameter(f"slots.{slot_key}.aliases").value,
            "pose": node.get_parameter(f"slots.{slot_key}.pose").value,
        }
    return load_slots(raw)


def make_pose_stamped(frame_id: str, pose_values: tuple[float, float, float, float, float, float, float]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = pose_values[0]
    pose.pose.position.y = pose_values[1]
    pose.pose.position.z = pose_values[2]
    pose.pose.orientation.x = pose_values[3]
    pose.pose.orientation.y = pose_values[4]
    pose.pose.orientation.z = pose_values[5]
    pose.pose.orientation.w = pose_values[6]
    return pose


def pose_to_dict(pose: PoseStamped) -> dict[str, Any]:
    return {
        "frame_id": pose.header.frame_id,
        "x": pose.pose.position.x,
        "y": pose.pose.position.y,
        "z": pose.pose.position.z,
        "qx": pose.pose.orientation.x,
        "qy": pose.pose.orientation.y,
        "qz": pose.pose.orientation.z,
        "qw": pose.pose.orientation.w,
    }
