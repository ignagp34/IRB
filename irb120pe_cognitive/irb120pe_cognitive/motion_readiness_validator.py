from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time
from typing import Sequence

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.node import Node
from ros2srrc_data.action import Move, Robmove
from sensor_msgs.msg import JointState


EXPECTED_CONTROLLERS = [
    "joint_state_broadcaster",
    "irb120_controller",
    "egp64_finger_left_controller",
    "egp64_finger_right_controller",
]

LEGACY_ACTIONS = {
    "/Robmove": "ros2srrc_data/action/Robmove",
    "/Move": "ros2srrc_data/action/Move",
}

MOVEIT_ACTIONS = {
    "/move_action": "moveit_msgs/action/MoveGroup",
    "/execute_trajectory": "moveit_msgs/action/ExecuteTrajectory",
    "/irb120_controller/follow_joint_trajectory": "control_msgs/action/FollowJointTrajectory",
}


@dataclass(frozen=True)
class CheckResult:
    label: str
    status: str
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.status in {"PASS", "WARN", "BLOCKED"}


def active_controller_names(controller_response: ListControllers.Response) -> list[str]:
    return [
        controller.name
        for controller in controller_response.controller
        if controller.state.lower() == "active"
    ]


def controller_lines(controller_response: ListControllers.Response) -> list[str]:
    return [
        f"{controller.name} {controller.state} {controller.type}"
        for controller in controller_response.controller
    ]


def missing_expected_controllers(controller_response: ListControllers.Response, expected: Sequence[str]) -> list[str]:
    active = set(active_controller_names(controller_response))
    return [name for name in expected if name not in active]


def action_graph_has(action_graph: dict[str, list[str]], action_name: str, expected_type: str) -> bool:
    return expected_type in action_graph.get(action_name, [])


def action_graph_text(action_graph: dict[str, list[str]]) -> str:
    if not action_graph:
        return "action_graph=[]"
    lines = ["action_graph_names_and_types= (server readiness is recorded in action_server_waits.json)"]
    for action_name in sorted(action_graph):
        types = ", ".join(action_graph[action_name])
        lines.append(f"{action_name}: {types}")
    return "\n".join(lines)


def joint_state_text(message: JointState | None) -> str:
    if message is None:
        return "joint_states=<unavailable>"
    lines = ["joint_states=["]
    for index, name in enumerate(message.name):
        position = message.position[index] if index < len(message.position) else float("nan")
        velocity = message.velocity[index] if index < len(message.velocity) else float("nan")
        lines.append(f"  {name}: position={position:.9f}, velocity={velocity:.9f}")
    lines.append("]")
    return "\n".join(lines)


def format_summary(results: Sequence[CheckResult]) -> str:
    if any(result.status == "FAIL" for result in results):
        overall = "FAIL"
    elif any(result.status == "BLOCKED" for result in results):
        overall = "BLOCKED"
    else:
        overall = "PASS"
    lines = [f"Motion readiness validation: {overall}"]
    for result in results:
        suffix = f" - {result.detail}" if result.detail else ""
        lines.append(f"[{result.status}] {result.label}{suffix}")
    return "\n".join(lines)


def write_evidence(output_dir: Path | None, filename: str, content: str) -> None:
    if output_dir is None:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / filename).write_text(content.rstrip() + "\n", encoding="utf-8")


class MotionReadinessValidator(Node):
    def __init__(self) -> None:
        super().__init__("irb120pe_motion_readiness_validator")
        self.controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.planning_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.joint_state_message: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.robmove_client = ActionClient(self, Robmove, "/Robmove")
        self.move_client = ActionClient(self, Move, "/Move")
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.follow_joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/irb120_controller/follow_joint_trajectory",
        )

    def list_controllers(self, timeout_sec: float) -> ListControllers.Response:
        if not self.controller_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/controller_manager/list_controllers was not available before timeout.")
        return self._call(self.controller_client, ListControllers.Request(), timeout_sec)

    def wait_for_joint_states(self, timeout_sec: float) -> JointState | None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() <= deadline:
            if self.joint_state_message is not None:
                return self.joint_state_message
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.joint_state_message

    def get_planning_scene_object_count(self, timeout_sec: float) -> int:
        if not self.planning_scene_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/get_planning_scene was not available before timeout.")
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        response = self._call(self.planning_scene_client, request, timeout_sec)
        return len(response.scene.world.collision_objects)

    def action_graph(self) -> dict[str, list[str]]:
        return {
            action_name: list(action_types)
            for action_name, action_types in get_action_names_and_types(self)
        }

    def wait_for_action_servers(self, timeout_sec: float) -> dict[str, bool]:
        return {
            "/Robmove": self.robmove_client.wait_for_server(timeout_sec=timeout_sec),
            "/Move": self.move_client.wait_for_server(timeout_sec=timeout_sec),
            "/move_action": self.move_group_client.wait_for_server(timeout_sec=timeout_sec),
            "/execute_trajectory": self.execute_trajectory_client.wait_for_server(timeout_sec=timeout_sec),
            "/irb120_controller/follow_joint_trajectory": self.follow_joint_trajectory_client.wait_for_server(
                timeout_sec=timeout_sec
            ),
        }

    def _joint_state_cb(self, message: JointState) -> None:
        self.joint_state_message = message

    def _call(self, client, request, timeout_sec: float):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() > deadline:
                raise TimeoutError("ROS service call did not finish before timeout.")
        result = future.result()
        if result is None:
            raise RuntimeError("ROS service call returned no result.")
        return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Capture non-moving simulation readiness evidence for IRB-120 Gazebo/MoveIt. "
            "This tool sends no trajectory, /Robmove, /Move, or LinkAttacher commands."
        )
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--service-timeout", type=float, default=30.0)
    parser.add_argument("--joint-state-timeout", type=float, default=10.0)
    parser.add_argument("--action-timeout", type=float, default=2.0)
    parser.add_argument(
        "--expected-controllers",
        default=",".join(EXPECTED_CONTROLLERS),
        help="Comma-separated controller names that must be active.",
    )
    parser.add_argument(
        "--require-legacy-actions",
        action="store_true",
        help="Return failure if /Robmove or /Move is unavailable instead of reporting the adapter path as blocked.",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    results: list[CheckResult] = []
    expected_controllers = [name.strip() for name in args.expected_controllers.split(",") if name.strip()]

    rclpy.init()
    node = MotionReadinessValidator()
    try:
        try:
            controller_response = node.list_controllers(args.service_timeout)
            controller_detail = "\n".join(controller_lines(controller_response))
            write_evidence(args.output_dir, "controllers.txt", controller_detail)
            missing = missing_expected_controllers(controller_response, expected_controllers)
            if missing:
                results.append(CheckResult("controllers active", "FAIL", f"missing active controllers={missing}"))
            else:
                results.append(CheckResult("controllers active", "PASS", f"active={expected_controllers}"))
        except Exception as exc:
            results.append(CheckResult("controllers active", "FAIL", str(exc)))

        joint_state = node.wait_for_joint_states(args.joint_state_timeout)
        write_evidence(args.output_dir, "joint_states_once.txt", joint_state_text(joint_state))
        if joint_state is None:
            results.append(CheckResult("/joint_states publishes", "FAIL", "no sample before timeout"))
        else:
            results.append(CheckResult("/joint_states publishes", "PASS", f"{len(joint_state.name)} joints sampled"))

        action_servers = node.wait_for_action_servers(args.action_timeout)
        action_graph = node.action_graph()
        write_evidence(args.output_dir, "action_graph.txt", action_graph_text(action_graph))
        write_evidence(args.output_dir, "action_server_waits.json", json.dumps(action_servers, indent=2, sort_keys=True))

        missing_moveit_actions = [
            action_name
            for action_name in MOVEIT_ACTIONS
            if not action_servers.get(action_name, False)
        ]
        if missing_moveit_actions:
            results.append(CheckResult("MoveIt/controller action servers ready", "FAIL", f"missing={missing_moveit_actions}"))
        else:
            results.append(CheckResult("MoveIt/controller action servers ready", "PASS", "move_action, execute_trajectory, follow_joint_trajectory available"))

        missing_legacy_actions = [
            action_name
            for action_name in LEGACY_ACTIONS
            if not action_servers.get(action_name, False)
        ]
        if missing_legacy_actions:
            status = "FAIL" if args.require_legacy_actions else "BLOCKED"
            results.append(
                CheckResult(
                    "existing action adapter execution path",
                    status,
                    f"missing={missing_legacy_actions}; ros2srrc_execution is required for /Robmove and /Move",
                )
            )
        else:
            results.append(CheckResult("existing action adapter execution path", "PASS", "/Robmove and /Move available; no goals sent"))

        try:
            object_count = node.get_planning_scene_object_count(args.service_timeout)
            write_evidence(args.output_dir, "moveit_planning_scene.txt", f"collision_object_count={object_count}")
            results.append(CheckResult("MoveIt planning scene service ready", "PASS", f"collision_object_count={object_count}"))
        except Exception as exc:
            results.append(CheckResult("MoveIt planning scene service ready", "FAIL", str(exc)))

        summary = format_summary(results)
        write_evidence(args.output_dir, "summary.txt", summary)
        print(summary)
        return 0 if all(result.ok for result in results) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        sys.exit(run(args))
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
