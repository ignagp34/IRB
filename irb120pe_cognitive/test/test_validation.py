import pytest

from irb120pe_cognitive.validation import (
    ValidationError,
    load_slots,
    resolve_slot,
    select_object,
    slots_as_tool_payload,
    validate_coordinates,
)


def test_validate_coordinates_accepts_safe_workspace_point():
    assert validate_coordinates(0.2, 0.2, 1.1) == (0.2, 0.2, 1.1)


def test_validate_coordinates_accepts_tabletop_object_pose():
    assert validate_coordinates(0.2, 0.2, 0.90) == (0.2, 0.2, 0.90)


def test_validate_coordinates_rejects_out_of_bounds_z():
    with pytest.raises(ValidationError, match="outside the safe workspace"):
        validate_coordinates(0.2, 0.2, 0.2)


def test_validate_coordinates_rejects_non_numeric_input():
    with pytest.raises(ValidationError, match="x must be numeric"):
        validate_coordinates("left", 0.2, 1.1)


def test_resolve_slot_accepts_aliases():
    slot = resolve_slot("left container")
    assert slot.key == "slot_a"


def test_resolve_slot_rejects_invalid_destination():
    with pytest.raises(ValidationError, match="Unknown destination"):
        resolve_slot("imaginary bin")


def test_select_object_by_color_label():
    objects = [
        {"object_id": "obj-1", "label": "BlueCube", "confidence": 0.8},
        {"object_id": "obj-2", "label": "BlackCube", "confidence": 0.7},
    ]
    selected = select_object(objects, label="blue")
    assert selected["object_id"] == "obj-1"


def test_select_object_requires_id_when_equal_confidence_ambiguous():
    objects = [
        {"object_id": "obj-1", "label": "BlueCube", "confidence": 0.8},
        {"object_id": "obj-2", "label": "BlueCube", "confidence": 0.8},
    ]
    with pytest.raises(ValidationError, match="ambiguous"):
        select_object(objects, label="blue")


def test_available_slots_payload_is_structured_for_llm_tools():
    payload = slots_as_tool_payload(load_slots())
    assert {slot["slot_id"] for slot in payload} >= {"slot_a", "slot_b", "slot_c", "unknown_slot"}
    assert payload[0]["pose"]["x"] is not None
