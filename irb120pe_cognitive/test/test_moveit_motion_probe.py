from types import SimpleNamespace

import pytest

from irb120pe_cognitive.moveit_motion_probe import (
    CheckResult,
    JointLimit,
    MotionProbeError,
    arm_positions_from_joint_state,
    choose_reversible_target,
    delta_report,
    format_summary,
    joint_state_text,
)


def test_choose_reversible_target_uses_positive_delta_when_inside_limits():
    current = {"joint_6": 0.0}

    target = choose_reversible_target(
        current,
        joint="joint_6",
        delta_rad=0.02,
        limit_margin_rad=0.05,
    )

    assert target["joint_6"] == pytest.approx(0.02)


def test_choose_reversible_target_reverses_near_upper_limit():
    current = {"joint_6": 0.99}
    limits = {"joint_6": JointLimit(-1.0, 1.0)}

    target = choose_reversible_target(
        current,
        joint="joint_6",
        delta_rad=0.02,
        limit_margin_rad=0.0,
        joint_limits=limits,
    )

    assert target["joint_6"] == pytest.approx(0.97)


def test_choose_reversible_target_aborts_when_delta_cannot_fit():
    current = {"joint_6": 0.0}
    limits = {"joint_6": JointLimit(-0.01, 0.01)}

    with pytest.raises(MotionProbeError, match="Cannot move joint_6"):
        choose_reversible_target(
            current,
            joint="joint_6",
            delta_rad=0.02,
            limit_margin_rad=0.0,
            joint_limits=limits,
        )


def test_arm_positions_from_joint_state_requires_all_arm_joints():
    message = SimpleNamespace(
        name=["joint_1", "joint_2"],
        position=[0.1, -0.2],
    )

    positions = arm_positions_from_joint_state(message, arm_joints=["joint_1", "joint_2"])

    assert positions == {"joint_1": 0.1, "joint_2": -0.2}

    with pytest.raises(MotionProbeError, match="required arm joints"):
        arm_positions_from_joint_state(message, arm_joints=["joint_1", "joint_3"])


def test_joint_state_text_formats_arm_positions():
    text = joint_state_text("sample", {"joint_1": 0.1, "joint_2": -0.2}, arm_joints=["joint_1", "joint_2"])

    assert text.splitlines()[0] == "sample=["
    assert "joint_1: position=0.100000000" in text
    assert "joint_2: position=-0.200000000" in text


def test_format_summary_prefers_fail_then_blocked_then_pass():
    failed = format_summary(
        [
            CheckResult("controllers active", "PASS", "ok"),
            CheckResult("MoveGroup outbound goal", "FAIL", "error"),
        ]
    )
    blocked = format_summary([CheckResult("MoveIt ready", "BLOCKED", "not ready")])
    passed = format_summary([CheckResult("plan only", "WARN", "no motion")])

    assert failed.splitlines()[0] == "MoveIt motion probe: FAIL"
    assert blocked.splitlines()[0] == "MoveIt motion probe: BLOCKED"
    assert passed.splitlines()[0] == "MoveIt motion probe: PASS"


def test_delta_report_records_target_after_and_final_deltas():
    before = {"joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0, "joint_4": 0.0, "joint_5": 0.0, "joint_6": 0.0}
    target = dict(before, joint_6=0.02)
    after = dict(before, joint_6=0.019)
    final = dict(before, joint_6=0.001)

    report = delta_report(before, after, final, target=target, moved_joint="joint_6")

    assert report["target_delta"] == pytest.approx(0.02)
    assert report["moved_joint_after_delta"] == pytest.approx(0.019)
    assert report["max_abs_final_delta"] == pytest.approx(0.001)
