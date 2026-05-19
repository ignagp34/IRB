from argparse import Namespace
from types import SimpleNamespace

from irb120pe_cognitive.live_cube_validator import (
    DEFAULT_EXPECTED_REASONING,
    build_parser,
    format_summary,
    matching_detections,
    planning_scene_has_ids,
    planning_scene_is_clear,
    pose_from_args,
)


def _object(object_id: str, label: str):
    return SimpleNamespace(object_id=object_id, label=label)


def test_matching_detections_accepts_expected_label():
    objects = [_object("blue_1", "blue"), _object("sticker_1", "sticker")]

    matches = matching_detections(objects, expected_label="blue", expected_ids=[])

    assert [obj.object_id for obj in matches] == ["blue_1"]


def test_matching_detections_accepts_expected_object_id():
    objects = [_object("blue_1", "blue"), _object("sticker_1", "sticker")]

    matches = matching_detections(objects, expected_label="", expected_ids=["sticker_1"])

    assert [obj.object_id for obj in matches] == ["sticker_1"]


def test_planning_scene_id_checks():
    ids = ["blue_1", "sticker_1"]

    assert planning_scene_has_ids(ids, ["blue_1", "sticker_1"])
    assert not planning_scene_has_ids(ids, ["blue_1", "missing_1"])
    assert planning_scene_is_clear(["robot_table"], ["blue_1", "sticker_1"])
    assert not planning_scene_is_clear(ids, ["blue_1"])


def test_format_summary_reports_overall_failure():
    summary = format_summary(
        [
            ("spawn cube", True, "ok"),
            ("mock reasoning dry-run", False, "unexpected status"),
        ]
    )

    assert summary.splitlines()[0] == "Live cube validation: FAIL"
    assert "[PASS] spawn cube - ok" in summary
    assert "[FAIL] mock reasoning dry-run - unexpected status" in summary


def test_parser_defaults_match_live_blue_cube_validation():
    args = build_parser().parse_args([])

    assert args.cube == "BlueCube"
    assert args.name is None
    assert args.x == 0.55
    assert args.y == 0.52
    assert args.z == 0.88
    assert args.qw == 1.0
    assert args.expected_label == "blue"
    assert args.expected_object_ids == ["blue_1"]
    assert args.expected_planning_scene_ids == ["blue_1", "sticker_1"]
    assert args.expected_reasoning_status == DEFAULT_EXPECTED_REASONING


def test_pose_from_args_maps_position_and_orientation():
    pose = pose_from_args(
        Namespace(x=0.55, y=0.52, z=0.88, qx=0.1, qy=0.2, qz=0.3, qw=0.4)
    )

    assert pose.position.x == 0.55
    assert pose.position.y == 0.52
    assert pose.position.z == 0.88
    assert pose.orientation.x == 0.1
    assert pose.orientation.y == 0.2
    assert pose.orientation.z == 0.3
    assert pose.orientation.w == 0.4
