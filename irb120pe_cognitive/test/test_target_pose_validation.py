"""Tests for the free target_pose path through the arrangement planner.

These tests intentionally cover the *planner* contract (the boundary the
LangChain tool relies on) without booting a real ROS node. That keeps the
suite runnable in any environment (CI, WSL without Gazebo, native Windows).
"""

from __future__ import annotations

import pytest

from irb120pe_cognitive.arrangement_planner import build_arrangement_plan
from irb120pe_cognitive.validation import ValidationError, validate_coordinates


def _objects():
    return [
        {"object_id": "white_1", "label": "WhiteCube", "confidence": 0.9},
        {"object_id": "black_1", "label": "BlackCube", "confidence": 0.85},
        {"object_id": "blue_1", "label": "BlueCube", "confidence": 0.95},
    ]


def test_planner_targets_pass_workspace_validation():
    steps = build_arrangement_plan(
        "Line by color from white to black to blue along Y at x=0.55, z=1.00, spacing=0.06",
        _objects(),
    )
    for step in steps:
        x, y, z = step.target_pose[:3]
        validated = validate_coordinates(x, y, z)
        assert validated == pytest.approx((x, y, z))


def test_planner_rejects_out_of_workspace_target():
    with pytest.raises(ValidationError):
        build_arrangement_plan(
            "Line by color along Y at x=1.50, z=1.00, spacing=0.06",  # x out of bounds
            _objects(),
        )


def test_empty_instruction_rejected():
    with pytest.raises(ValidationError):
        build_arrangement_plan("", _objects())


def test_planner_prefers_highest_confidence_per_color():
    objects = [
        {"object_id": "blue_1", "label": "BlueCube", "confidence": 0.40},
        {"object_id": "blue_2", "label": "BlueCube", "confidence": 0.95},
        {"object_id": "black_1", "label": "BlackCube", "confidence": 0.80},
        {"object_id": "white_1", "label": "WhiteCube", "confidence": 0.70},
    ]
    steps = build_arrangement_plan(
        "Line by color from white to black to blue along Y at x=0.55, z=1.00",
        objects,
    )
    blue_step = next(step for step in steps if step.color == "blue")
    assert blue_step.object_id == "blue_2"
