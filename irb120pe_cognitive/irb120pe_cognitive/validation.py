from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any, Iterable

from .defaults import COLOR_ALIASES, DEFAULT_SLOTS, DEFAULT_WORKSPACE_LIMITS


class ValidationError(ValueError):
    """Raised when an LLM tool request is incomplete, ambiguous, or unsafe."""


@dataclass(frozen=True)
class Slot:
    key: str
    label: str
    aliases: tuple[str, ...]
    pose: tuple[float, float, float, float, float, float, float]


def normalize_token(value: str) -> str:
    return value.strip().lower().replace("-", "_").replace(" ", "_")


def load_workspace_limits(raw: dict[str, Any] | None = None) -> dict[str, tuple[float, float]]:
    raw = raw or DEFAULT_WORKSPACE_LIMITS
    limits: dict[str, tuple[float, float]] = {}
    for axis in ("x", "y", "z"):
        pair = raw.get(axis, DEFAULT_WORKSPACE_LIMITS[axis])
        if len(pair) != 2:
            raise ValidationError(f"Workspace limit for {axis} must contain exactly two numeric values.")
        low, high = float(pair[0]), float(pair[1])
        if low >= high:
            raise ValidationError(f"Workspace limit for {axis} must be ordered low < high.")
        limits[axis] = (low, high)
    return limits


def validate_coordinates(x: Any, y: Any, z: Any, limits: dict[str, tuple[float, float]] | None = None) -> tuple[float, float, float]:
    limits = limits or DEFAULT_WORKSPACE_LIMITS
    values = {"x": x, "y": y, "z": z}
    result: dict[str, float] = {}
    for axis, value in values.items():
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exc:
            raise ValidationError(f"{axis} must be numeric.") from exc
        if not isfinite(numeric):
            raise ValidationError(f"{axis} must be finite.")
        low, high = limits[axis]
        if numeric < low or numeric > high:
            raise ValidationError(f"{axis}={numeric:.3f} is outside the safe workspace [{low:.3f}, {high:.3f}].")
        result[axis] = numeric
    return result["x"], result["y"], result["z"]


def load_slots(raw: dict[str, Any] | None = None) -> dict[str, Slot]:
    raw = raw or DEFAULT_SLOTS
    slots: dict[str, Slot] = {}
    for key, value in raw.items():
        pose = tuple(float(v) for v in value["pose"])
        if len(pose) != 7:
            raise ValidationError(f"Slot {key} pose must contain x, y, z, qx, qy, qz, qw.")
        aliases = tuple(normalize_token(a) for a in value.get("aliases", [key]))
        slots[normalize_token(key)] = Slot(
            key=normalize_token(key),
            label=str(value.get("label", key)),
            aliases=aliases,
            pose=pose,
        )
    return slots


def resolve_slot(name: str, slots: dict[str, Slot] | None = None) -> Slot:
    if not name:
        raise ValidationError("Destination slot/container is required.")
    slots = slots or load_slots()
    token = normalize_token(name)
    matches = [slot for slot in slots.values() if token == slot.key or token in slot.aliases]
    if not matches:
        valid = sorted({alias for slot in slots.values() for alias in slot.aliases})
        raise ValidationError(f"Unknown destination '{name}'. Valid destinations: {', '.join(valid)}.")
    if len(matches) > 1:
        raise ValidationError(f"Destination '{name}' is ambiguous.")
    return matches[0]


def label_matches(query: str, label: str) -> bool:
    query_token = normalize_token(query)
    label_token = normalize_token(label)
    if query_token == label_token:
        return True
    for canonical, aliases in COLOR_ALIASES.items():
        if query_token in aliases:
            return label_token in aliases or label_token == canonical
    return query_token in label_token


def select_object(objects: Iterable[dict[str, Any]], *, object_id: str | None = None, label: str | None = None) -> dict[str, Any]:
    object_list = list(objects)
    if object_id:
        matches = [obj for obj in object_list if str(obj.get("object_id", "")) == object_id]
    elif label:
        matches = [obj for obj in object_list if label_matches(label, str(obj.get("label", "")))]
    else:
        raise ValidationError("A source object id or label is required.")

    if not matches:
        raise ValidationError("No detected object matches the requested source.")
    if len(matches) > 1:
        matches = sorted(matches, key=lambda obj: float(obj.get("confidence", 0.0)), reverse=True)
        if float(matches[0].get("confidence", 0.0)) == float(matches[1].get("confidence", 0.0)):
            raise ValidationError("Requested source is ambiguous; provide an object id.")
    return matches[0]


def slots_as_tool_payload(slots: dict[str, Slot] | None = None) -> list[dict[str, Any]]:
    slots = slots or load_slots()
    return [
        {
            "slot_id": slot.key,
            "label": slot.label,
            "aliases": list(slot.aliases),
            "pose": {
                "x": slot.pose[0],
                "y": slot.pose[1],
                "z": slot.pose[2],
                "qx": slot.pose[3],
                "qy": slot.pose[4],
                "qz": slot.pose[5],
                "qw": slot.pose[6],
            },
        }
        for slot in slots.values()
    ]
