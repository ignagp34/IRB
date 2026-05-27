"""Pure-Python tests for the arrangement planner. No ROS runtime required."""

from __future__ import annotations

import pytest

from irb120pe_cognitive.arrangement_planner import (
    DEFAULT_GRASP_ORIENTATION,
    build_arrangement_plan,
    parse_layout,
    plan_as_json_payload,
)
from irb120pe_cognitive.validation import ValidationError


def _objects():
    return [
        {"object_id": "white_1", "label": "WhiteCube", "confidence": 0.9},
        {"object_id": "black_1", "label": "BlackCube", "confidence": 0.85},
        {"object_id": "blue_1", "label": "BlueCube", "confidence": 0.95},
    ]


def test_parse_layout_recognises_line_along_y():
    spec = parse_layout("Arrange cubes in a line by color from white to black to blue along Y at x=0.55, z=0.90")
    assert spec.kind == "line"
    assert spec.axis == "y"
    assert spec.color_order == ("white", "black", "blue")
    assert spec.fixed["x"] == pytest.approx(0.55)
    assert spec.fixed["z"] == pytest.approx(0.90)


def test_parse_layout_recognises_tower():
    spec = parse_layout("Build a tower at x=0.55, y=0.52, z=1.00")
    assert spec.kind == "tower"
    assert spec.color_order  # default order applied
    assert spec.anchor == (0.55, 0.52, 1.00)


def test_build_plan_produces_ordered_targets_inside_workspace():
    steps = build_arrangement_plan(
        "Line by color from white to black to blue along Y at x=0.55, z=0.90, spacing=0.06",
        _objects(),
    )
    assert [s.color for s in steps] == ["white", "black", "blue"]
    # x and z constant, y monotonically increasing
    assert all(s.target_pose[0] == pytest.approx(0.55) for s in steps)
    assert all(s.target_pose[2] == pytest.approx(0.90) for s in steps)
    ys = [s.target_pose[1] for s in steps]
    assert ys == sorted(ys)
    # grasp orientation defaults applied
    assert steps[0].target_pose[3:] == DEFAULT_GRASP_ORIENTATION


def test_build_plan_rejects_targets_outside_workspace():
    with pytest.raises(ValidationError, match="outside the safe workspace"):
        build_arrangement_plan(
            "Line by color along Y at x=0.55, z=0.50, spacing=0.06",  # z too low
            _objects(),
        )


def test_build_plan_rejects_when_no_color_matches():
    with pytest.raises(ValidationError, match="at least one detected cube"):
        build_arrangement_plan(
            "Line by color from white to blue along Y at x=0.55, z=1.00",
            [{"object_id": "obj-1", "label": "GreenSphere", "confidence": 0.5}],
        )


def test_build_plan_emits_tower_with_stacked_z():
    steps = build_arrangement_plan(
        "Build a tower at x=0.55, y=0.52, z=1.00",
        _objects(),
    )
    zs = [s.target_pose[2] for s in steps]
    assert zs == sorted(zs)
    assert all(s.target_pose[0] == pytest.approx(0.55) for s in steps)
    assert all(s.target_pose[1] == pytest.approx(0.52) for s in steps)


def test_plan_as_json_payload_is_serialisable():
    import json

    steps = build_arrangement_plan(
        "Line by color along Y at x=0.55, z=0.90, spacing=0.06",
        _objects(),
    )
    payload = plan_as_json_payload(steps)
    encoded = json.dumps(payload)
    decoded = json.loads(encoded)
    assert len(decoded) == 3
    assert {entry["color"] for entry in decoded} == {"white", "black", "blue"}
