from types import SimpleNamespace

from irb120pe_cognitive.motion_readiness_validator import (
    CheckResult,
    action_graph_has,
    action_graph_text,
    active_controller_names,
    format_summary,
    joint_state_text,
    missing_expected_controllers,
)


def _controller(name: str, state: str = "active", controller_type: str = "test/Controller"):
    return SimpleNamespace(name=name, state=state, type=controller_type)


def test_controller_helpers_report_active_and_missing_names():
    response = SimpleNamespace(
        controller=[
            _controller("joint_state_broadcaster"),
            _controller("irb120_controller"),
            _controller("inactive_controller", "inactive"),
        ]
    )

    assert active_controller_names(response) == ["joint_state_broadcaster", "irb120_controller"]
    assert missing_expected_controllers(response, ["joint_state_broadcaster", "missing"]) == ["missing"]


def test_action_graph_helpers_check_expected_type():
    graph = {
        "/move_action": ["moveit_msgs/action/MoveGroup"],
        "/Robmove": ["ros2srrc_data/action/Robmove"],
    }

    assert action_graph_has(graph, "/move_action", "moveit_msgs/action/MoveGroup")
    assert not action_graph_has(graph, "/Move", "ros2srrc_data/action/Move")
    assert "/Robmove: ros2srrc_data/action/Robmove" in action_graph_text(graph)


def test_joint_state_text_formats_positions_and_missing_velocity():
    message = SimpleNamespace(
        name=["joint_1", "joint_2"],
        position=[0.1, -0.2],
        velocity=[0.0],
    )

    text = joint_state_text(message)

    assert "joint_1: position=0.100000000, velocity=0.000000000" in text
    assert "joint_2: position=-0.200000000, velocity=nan" in text


def test_format_summary_prefers_fail_then_blocked_then_pass():
    failed = format_summary(
        [
            CheckResult("controllers active", "PASS", "ok"),
            CheckResult("MoveIt ready", "FAIL", "missing"),
            CheckResult("legacy actions", "BLOCKED", "not installed"),
        ]
    )
    blocked = format_summary(
        [
            CheckResult("controllers active", "PASS", "ok"),
            CheckResult("legacy actions", "BLOCKED", "not installed"),
        ]
    )
    passed = format_summary([CheckResult("controllers active", "PASS", "ok")])

    assert failed.splitlines()[0] == "Motion readiness validation: FAIL"
    assert blocked.splitlines()[0] == "Motion readiness validation: BLOCKED"
    assert passed.splitlines()[0] == "Motion readiness validation: PASS"
