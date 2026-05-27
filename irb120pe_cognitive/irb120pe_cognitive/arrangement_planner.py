"""Deterministic layout planner for the goal-oriented arrangement mission.

This module is intentionally pure Python (no rclpy imports) so it can be
unit-tested without a running ROS stack and reused by both the mock provider
and the LangChain tool implementations.

The planner converts a natural-language instruction such as
``"arrange the cubes in a line by color along Y at x=0.55, z=0.88"`` into an
ordered list of target poses for the detected cubes. Every generated pose is
validated against the cognitive workspace limits before being returned.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Iterable, Mapping

from .defaults import COLOR_ALIASES, DEFAULT_WORKSPACE_LIMITS
from .validation import ValidationError, label_matches, validate_coordinates


# Default colour order when the instruction asks for a "line by color" without
# specifying the order explicitly. White is the brightest, blue is the
# canonical "blue cube" target so we put it last for visual clarity.
DEFAULT_COLOR_ORDER = ("white", "black", "blue")

# Default spacing (meters) between adjacent placements in a line.
DEFAULT_SPACING_M = 0.06

# Default tower step height (meters) — slightly larger than cube_size_m so the
# next cube does not collide with the previous one before the gripper opens.
DEFAULT_TOWER_STEP_M = 0.04

# Default grasp orientation used when the planner emits target poses.
DEFAULT_GRASP_ORIENTATION = (0.707, 0.707, 0.0, 0.0)


@dataclass(frozen=True)
class ArrangementStep:
    """A single (object, target pose) pair produced by the planner."""

    object_id: str
    label: str
    color: str
    target_pose: tuple[float, float, float, float, float, float, float]

    def as_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "label": self.label,
            "color": self.color,
            "target_pose": {
                "x": self.target_pose[0],
                "y": self.target_pose[1],
                "z": self.target_pose[2],
                "qx": self.target_pose[3],
                "qy": self.target_pose[4],
                "qz": self.target_pose[5],
                "qw": self.target_pose[6],
            },
        }


@dataclass(frozen=True)
class LayoutSpec:
    """Parsed representation of an arrangement instruction."""

    kind: str  # "line" | "tower" | "between"
    axis: str  # "x" | "y" (line/between)
    fixed: dict[str, float]  # axes that are held constant
    spacing: float
    color_order: tuple[str, ...]
    anchor: tuple[float, float, float] | None = None  # tower anchor (x, y, z)


def _canonical_color(label: str) -> str | None:
    """Return the canonical colour name for an arbitrary detection label."""

    if not label:
        return None
    for canonical, aliases in COLOR_ALIASES.items():
        if canonical == "cube":
            continue
        if label_matches(canonical, label):
            return canonical
    return None


def _extract_axis_value(text: str, axis: str) -> float | None:
    """Parse 'axis=value' or 'axis = value' style anchors."""

    pattern = rf"{axis}\s*=\s*(-?\d+(?:\.\d+)?)"
    match = re.search(pattern, text)
    if match is None:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def _extract_spacing(text: str) -> float:
    match = re.search(r"spac(?:e|ing)\s*[:=]?\s*(-?\d+(?:\.\d+)?)", text)
    if match is None:
        return DEFAULT_SPACING_M
    return float(match.group(1))


def _extract_color_order(text: str) -> tuple[str, ...]:
    """Extract a colour order written as 'from X to Y to Z' or fallback default."""

    colors_found: list[str] = []
    for canonical in COLOR_ALIASES:
        if canonical == "cube":
            continue
        for alias in COLOR_ALIASES[canonical]:
            for match in re.finditer(rf"\b{re.escape(alias)}\b", text):
                colors_found.append((match.start(), canonical))
                break  # one match per alias is enough; we keep the order
    colors_found.sort(key=lambda item: item[0])
    ordered: list[str] = []
    seen: set[str] = set()
    for _, color in colors_found:
        if color not in seen:
            seen.add(color)
            ordered.append(color)
    if len(ordered) >= 2:
        return tuple(ordered)
    return DEFAULT_COLOR_ORDER


def parse_layout(instruction: str, *, defaults: Mapping[str, Any] | None = None) -> LayoutSpec:
    """Parse a natural-language arrangement instruction into a LayoutSpec.

    The parser is deliberately conservative — it only recognises the layouts
    documented in the demo material. Any unrecognised instruction raises
    ``ValidationError`` so the caller can surface a clear error to the LLM.
    """

    if not instruction or not instruction.strip():
        raise ValidationError("Arrangement instruction is empty.")

    defaults = defaults or {}
    text = instruction.lower()
    color_order = _extract_color_order(text)

    if "tower" in text or "stack" in text:
        anchor_x = _extract_axis_value(text, "x") or float(defaults.get("tower_x", 0.55))
        anchor_y = _extract_axis_value(text, "y") or float(defaults.get("tower_y", 0.52))
        anchor_z = _extract_axis_value(text, "z") or float(defaults.get("table_top_z", 1.00))
        return LayoutSpec(
            kind="tower",
            axis="z",
            fixed={"x": anchor_x, "y": anchor_y},
            spacing=_extract_spacing(text) if "spac" in text else DEFAULT_TOWER_STEP_M,
            color_order=color_order,
            anchor=(anchor_x, anchor_y, anchor_z),
        )

    # Default to a line layout.
    if "along x" in text:
        axis = "x"
    elif "along y" in text or "line" in text or "row" in text:
        axis = "y"
    else:
        axis = "y"

    fixed_axes = {a: _extract_axis_value(text, a) for a in ("x", "y", "z") if a != axis}
    if fixed_axes.get("x") is None:
        fixed_axes["x"] = float(defaults.get("line_x", 0.55))
    if fixed_axes.get("y") is None and axis != "y":
        fixed_axes["y"] = float(defaults.get("line_y", 0.40))
    if fixed_axes.get("z") is None:
        fixed_axes["z"] = float(defaults.get("table_top_z", 1.00))

    fixed = {axis_name: float(value) for axis_name, value in fixed_axes.items() if value is not None}
    spacing = _extract_spacing(text)

    # For a line we also need a starting offset on the moving axis. Use a
    # symmetric layout centred on the workspace mid-point unless one was
    # supplied via "start=" hint.
    start_value = _extract_axis_value(text, "start") if "start" in text else None
    if start_value is not None:
        fixed[f"{axis}_start"] = float(start_value)
    else:
        fixed[f"{axis}_start"] = float(defaults.get(f"line_{axis}_start", 0.30))

    return LayoutSpec(kind="line", axis=axis, fixed=fixed, spacing=spacing, color_order=color_order)


def _line_target(spec: LayoutSpec, index: int) -> tuple[float, float, float]:
    axis = spec.axis
    start = spec.fixed[f"{axis}_start"]
    moving_value = start + index * spec.spacing
    if axis == "y":
        x = spec.fixed["x"]
        y = moving_value
        z = spec.fixed["z"]
    else:
        x = moving_value
        y = spec.fixed["y"]
        z = spec.fixed["z"]
    return x, y, z


def _tower_target(spec: LayoutSpec, index: int) -> tuple[float, float, float]:
    anchor = spec.anchor or (spec.fixed["x"], spec.fixed["y"], 1.00)
    return anchor[0], anchor[1], anchor[2] + index * spec.spacing


def _match_objects_to_colors(
    spec: LayoutSpec, objects: Iterable[Mapping[str, Any]]
) -> list[Mapping[str, Any]]:
    """Order detections according to spec.color_order. Missing colours are skipped."""

    by_color: dict[str, list[Mapping[str, Any]]] = {}
    for obj in objects:
        color = _canonical_color(str(obj.get("label", "")))
        if color is None:
            continue
        by_color.setdefault(color, []).append(obj)
    # Within each colour pick the highest-confidence detection so duplicates do
    # not break the arrangement.
    ordered: list[Mapping[str, Any]] = []
    for color in spec.color_order:
        candidates = by_color.get(color, [])
        if not candidates:
            continue
        candidates.sort(key=lambda item: float(item.get("confidence", 0.0)), reverse=True)
        ordered.append(candidates[0])
    return ordered


def build_arrangement_plan(
    instruction: str,
    objects: Iterable[Mapping[str, Any]],
    *,
    workspace_limits: dict[str, tuple[float, float]] | None = None,
    grasp_orientation: tuple[float, float, float, float] = DEFAULT_GRASP_ORIENTATION,
    defaults: Mapping[str, Any] | None = None,
) -> list[ArrangementStep]:
    """Produce the ordered list of (object, target_pose) tuples for an instruction."""

    limits = workspace_limits or {axis: tuple(values) for axis, values in DEFAULT_WORKSPACE_LIMITS.items()}
    spec = parse_layout(instruction, defaults=defaults)
    ordered_objects = _match_objects_to_colors(spec, objects)
    if not ordered_objects:
        raise ValidationError("Arrangement requires at least one detected cube whose colour matches the order.")

    steps: list[ArrangementStep] = []
    for index, obj in enumerate(ordered_objects):
        if spec.kind == "tower":
            x, y, z = _tower_target(spec, index)
        else:
            x, y, z = _line_target(spec, index)
        validate_coordinates(x, y, z, limits)  # raises ValidationError if out of bounds
        qx, qy, qz, qw = grasp_orientation
        color = _canonical_color(str(obj.get("label", ""))) or "cube"
        steps.append(
            ArrangementStep(
                object_id=str(obj.get("object_id", "")),
                label=str(obj.get("label", "")),
                color=color,
                target_pose=(float(x), float(y), float(z), float(qx), float(qy), float(qz), float(qw)),
            )
        )
    return steps


def plan_as_json_payload(steps: Iterable[ArrangementStep]) -> list[dict[str, Any]]:
    """Convert plan steps into a serialisable payload (used in /trace and srv reply)."""

    return [step.as_dict() for step in steps]


def euclidean_distance(a: Iterable[float], b: Iterable[float]) -> float:
    """Small helper used by the validator to compare planned vs final positions."""

    a_list = list(a)
    b_list = list(b)
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a_list, b_list)))
