from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time
from typing import Mapping, Sequence

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ros2srrc_data.action import Move, Robmove
from sensor_msgs.msg import JointState


ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
EXPECTED_CONTROLLERS = [
    "joint_state_broadcaster",
    "irb120_controller",
    "egp64_finger_left_controller",
    "egp64_finger_right_controller",
]
SUCCESS = 1


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float


@dataclass(frozen=True)
class CheckResult:
    label: str
    status: str
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.status in {"PASS", "WARN"}


JOINT_LIMITS = {
    "joint_1": JointLimit(-2.87979, 2.87979),
    "joint_2": JointLimit(-1.91986, 1.91986),
    "joint_3": JointLimit(-1.91986, 1.22173),
    "joint_4": JointLimit(-2.79253, 2.79253),
    "joint_5": JointLimit(-2.0, 2.0),
    "joint_6": JointLimit(-6.98132, 6.98132),
}


class MotionProbeError(RuntimeError):
    pass


def active_controller_names(controller_response: ListControllers.Response) -> list[str]:
    return [
        controller.name
        for controller in controller_response.controller
        if controller.state.lower() == "active"
    ]


def missing_expected_controllers(controller_response: ListControllers.Response, expected: Sequence[str]) -> list[str]:
    active = set(active_controller_names(controller_response))
    return [name for name in expected if name not in active]


def arm_positions_from_joint_state(message: JointState, arm_joints: Sequence[str] = ARM_JOINTS) -> dict[str, float]:
    positions = {
        name: message.position[index]
        for index, name in enumerate(message.name)
        if index < len(message.position)
    }
    missing = [joint for joint in arm_joints if joint not in positions]
    if missing:
        raise MotionProbeError(f"/joint_states did not include required arm joints: {missing}")
    return {joint: float(positions[joint]) for joint in arm_joints}


def choose_reversible_target(
    current_positions: Mapping[str, float],
    *,
    joint: str,
    delta_rad: float,
    limit_margin_rad: float,
    joint_limits: Mapping[str, JointLimit] = JOINT_LIMITS,
) -> dict[str, float]:
    if joint not in current_positions:
        raise MotionProbeError(f"Target joint {joint} was not present in the sampled arm state.")
    if joint not in joint_limits:
        raise MotionProbeError(f"No configured joint limit is available for {joint}.")
    if delta_rad <= 0.0:
        raise MotionProbeError("--delta-rad must be greater than zero.")
    if limit_margin_rad < 0.0:
        raise MotionProbeError("--limit-margin-rad must be non-negative.")

    current = float(current_positions[joint])
    limit = joint_limits[joint]
    lower = limit.lower + limit_margin_rad
    upper = limit.upper - limit_margin_rad
    if lower > upper:
        raise MotionProbeError(f"Limit margin leaves no valid range for {joint}.")

    for signed_delta in (abs(delta_rad), -abs(delta_rad)):
        candidate = current + signed_delta
        if lower <= candidate <= upper:
            target = dict(current_positions)
            target[joint] = candidate
            return target

    raise MotionProbeError(
        f"Cannot move {joint} by +/-{delta_rad:.6f} rad from {current:.6f} "
        f"while staying inside [{lower:.6f}, {upper:.6f}]."
    )


def joint_state_text(label: str, positions: Mapping[str, float], arm_joints: Sequence[str] = ARM_JOINTS) -> str:
    lines = [f"{label}=["]
    for joint in arm_joints:
        value = positions.get(joint, float("nan"))
        lines.append(f"  {joint}: position={value:.9f}")
    lines.append("]")
    return "\n".join(lines)


def delta_report(
    before: Mapping[str, float],
    after: Mapping[str, float] | None,
    final: Mapping[str, float] | None,
    *,
    target: Mapping[str, float],
    moved_joint: str,
) -> dict[str, object]:
    report: dict[str, object] = {
        "moved_joint": moved_joint,
        "target_delta": float(target[moved_joint] - before[moved_joint]),
    }
    if after is not None:
        report["after_delta"] = {
            joint: float(after[joint] - before[joint])
            for joint in ARM_JOINTS
            if joint in before and joint in after
        }
        report["moved_joint_after_delta"] = float(after[moved_joint] - before[moved_joint])
    if final is not None:
        report["final_delta"] = {
            joint: float(final[joint] - before[joint])
            for joint in ARM_JOINTS
            if joint in before and joint in final
        }
        report["max_abs_final_delta"] = max(abs(final[joint] - before[joint]) for joint in ARM_JOINTS)
    return report


def move_group_result_summary(result) -> dict[str, object]:
    trajectory = result.planned_trajectory.joint_trajectory
    executed = result.executed_trajectory.joint_trajectory
    return {
        "error_code": int(result.error_code.val),
        "success": int(result.error_code.val) == SUCCESS,
        "planning_time": float(result.planning_time),
        "planned_joint_names": list(trajectory.joint_names),
        "planned_point_count": len(trajectory.points),
        "executed_joint_names": list(executed.joint_names),
        "executed_point_count": len(executed.points),
    }


def format_summary(results: Sequence[CheckResult]) -> str:
    if any(result.status == "FAIL" for result in results):
        overall = "FAIL"
    elif any(result.status == "BLOCKED" for result in results):
        overall = "BLOCKED"
    else:
        overall = "PASS"
    lines = [f"MoveIt motion probe: {overall}"]
    for result in results:
        suffix = f" - {result.detail}" if result.detail else ""
        lines.append(f"[{result.status}] {result.label}{suffix}")
    return "\n".join(lines)


def write_evidence(output_dir: Path | None, filename: str, content: str) -> None:
    if output_dir is None:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / filename).write_text(content.rstrip() + "\n", encoding="utf-8")


def write_json(output_dir: Path | None, filename: str, content: object) -> None:
    write_evidence(output_dir, filename, json.dumps(content, indent=2, sort_keys=True))


class MoveItMotionProbe(Node):
    def __init__(self) -> None:
        super().__init__("irb120pe_moveit_motion_probe")
        self.controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.follow_joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/irb120_controller/follow_joint_trajectory",
        )
        self.robmove_client = ActionClient(self, Robmove, "/Robmove")
        self.move_client = ActionClient(self, Move, "/Move")
        self.joint_state_message: JointState | None = None
        self.joint_state_received_at = 0.0
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

    def list_controllers(self, timeout_sec: float) -> ListControllers.Response:
        if not self.controller_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/controller_manager/list_controllers was not available before timeout.")
        return self._call(self.controller_client, ListControllers.Request(), timeout_sec)

    def wait_for_joint_state(self, timeout_sec: float, *, after_monotonic: float = 0.0) -> JointState:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() <= deadline:
            if self.joint_state_message is not None and self.joint_state_received_at >= after_monotonic:
                return self.joint_state_message
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError("/joint_states did not publish a fresh sample before timeout.")

    def wait_for_action_servers(self, timeout_sec: float) -> dict[str, bool]:
        return {
            "/move_action": self.move_group_client.wait_for_server(timeout_sec=timeout_sec),
            "/execute_trajectory": self.execute_trajectory_client.wait_for_server(timeout_sec=timeout_sec),
            "/irb120_controller/follow_joint_trajectory": self.follow_joint_trajectory_client.wait_for_server(
                timeout_sec=timeout_sec
            ),
            "/Robmove": self.robmove_client.wait_for_server(timeout_sec=timeout_sec),
            "/Move": self.move_client.wait_for_server(timeout_sec=timeout_sec),
        }

    def send_move_group_goal(
        self,
        *,
        start_positions: Mapping[str, float],
        target_positions: Mapping[str, float],
        group_name: str,
        planner_id: str,
        planning_time: float,
        velocity_scaling: float,
        acceleration_scaling: float,
        execute: bool,
        timeout_sec: float,
    ):
        if not self.move_group_client.wait_for_server(timeout_sec=timeout_sec):
            raise TimeoutError("/move_action was not available before timeout.")
        goal = build_move_group_goal(
            start_positions=start_positions,
            target_positions=target_positions,
            group_name=group_name,
            planner_id=planner_id,
            planning_time=planning_time,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
            plan_only=not execute,
            stamp=self.get_clock().now().to_msg(),
        )
        sent_at = time.monotonic()
        send_future = self.move_group_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, timeout_sec) or send_future.result() is None:
            raise TimeoutError("/move_action goal send timed out.")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise MotionProbeError("/move_action goal was rejected.")
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec) or result_future.result() is None:
            raise TimeoutError("/move_action result timed out.")
        return result_future.result().result, sent_at

    def _joint_state_cb(self, message: JointState) -> None:
        self.joint_state_message = message
        self.joint_state_received_at = time.monotonic()

    def _call(self, client, request, timeout_sec: float):
        future = client.call_async(request)
        if not self._wait_for_future(future, timeout_sec) or future.result() is None:
            raise TimeoutError("ROS service call did not finish before timeout.")
        return future.result()

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() > deadline:
                return False
        return future.done()


def build_move_group_goal(
    *,
    start_positions: Mapping[str, float],
    target_positions: Mapping[str, float],
    group_name: str,
    planner_id: str,
    planning_time: float,
    velocity_scaling: float,
    acceleration_scaling: float,
    plan_only: bool,
    stamp,
) -> MoveGroup.Goal:
    goal = MoveGroup.Goal()
    goal.request.group_name = group_name
    goal.request.planner_id = planner_id
    goal.request.num_planning_attempts = 1
    goal.request.allowed_planning_time = planning_time
    goal.request.max_velocity_scaling_factor = velocity_scaling
    goal.request.max_acceleration_scaling_factor = acceleration_scaling
    goal.request.start_state.joint_state.header.stamp = stamp
    goal.request.start_state.joint_state.name = list(ARM_JOINTS)
    goal.request.start_state.joint_state.position = [float(start_positions[joint]) for joint in ARM_JOINTS]

    constraints = Constraints()
    constraints.name = "tiny_reversible_joint_goal"
    for joint in ARM_JOINTS:
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint
        joint_constraint.position = float(target_positions[joint])
        joint_constraint.tolerance_above = 0.005
        joint_constraint.tolerance_below = 0.005
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    goal.request.goal_constraints.append(constraints)

    goal.planning_options.plan_only = plan_only
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.planning_scene_diff.is_diff = True
    return goal


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Plan or execute a tiny reversible MoveIt-native IRB-120 motion in Gazebo simulation only. "
            "This tool never uses /Robmove, /Move, LinkAttacher, gripper commands, or pick/place."
        )
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--joint", choices=ARM_JOINTS, default="joint_6")
    parser.add_argument("--delta-rad", type=float, default=0.02)
    parser.add_argument("--limit-margin-rad", type=float, default=0.05)
    parser.add_argument("--group-name", default="irb120_arm")
    parser.add_argument("--planner-id", default="PTP")
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.05)
    parser.add_argument("--acceleration-scaling", type=float, default=0.05)
    parser.add_argument("--service-timeout", type=float, default=30.0)
    parser.add_argument("--joint-state-timeout", type=float, default=10.0)
    parser.add_argument("--action-timeout", type=float, default=60.0)
    parser.add_argument("--action-server-timeout", type=float, default=2.0)
    parser.add_argument("--execute", action="store_true", help="Execute the planned Gazebo/MoveIt simulation motion.")
    parser.add_argument(
        "--return-to-start",
        dest="return_to_start",
        action="store_true",
        default=True,
        help="Return to the original arm joint state after a successful executed outbound move.",
    )
    parser.add_argument(
        "--no-return-to-start",
        dest="return_to_start",
        action="store_false",
        help="Do not send the return MoveGroup goal after an executed outbound move.",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    results: list[CheckResult] = []
    after_positions: dict[str, float] | None = None
    final_positions: dict[str, float] | None = None
    target_positions: dict[str, float] | None = None

    rclpy.init()
    node = MoveItMotionProbe()
    try:
        try:
            controller_response = node.list_controllers(args.service_timeout)
            missing = missing_expected_controllers(controller_response, EXPECTED_CONTROLLERS)
            write_evidence(
                args.output_dir,
                "controllers.txt",
                "\n".join(f"{controller.name} {controller.state}" for controller in controller_response.controller),
            )
            if missing:
                results.append(CheckResult("controllers active", "FAIL", f"missing active controllers={missing}"))
            else:
                results.append(CheckResult("controllers active", "PASS", f"active={EXPECTED_CONTROLLERS}"))
        except Exception as exc:
            results.append(CheckResult("controllers active", "FAIL", str(exc)))

        action_servers = node.wait_for_action_servers(args.action_server_timeout)
        write_json(args.output_dir, "action_server_waits.json", action_servers)
        required_actions = [
            "/move_action",
            "/execute_trajectory",
            "/irb120_controller/follow_joint_trajectory",
        ]
        missing_actions = [action for action in required_actions if not action_servers.get(action, False)]
        if missing_actions:
            results.append(CheckResult("MoveIt/controller action servers ready", "FAIL", f"missing={missing_actions}"))
        else:
            results.append(CheckResult("MoveIt/controller action servers ready", "PASS", "move_action, execute_trajectory, follow_joint_trajectory available"))
        legacy_available = [action for action in ["/Robmove", "/Move"] if action_servers.get(action, False)]
        results.append(
            CheckResult(
                "legacy action servers unused",
                "PASS",
                f"available_but_not_used={legacy_available}" if legacy_available else "/Robmove and /Move unavailable and unused",
            )
        )

        try:
            pre_message = node.wait_for_joint_state(args.joint_state_timeout)
            pre_positions = arm_positions_from_joint_state(pre_message)
            write_evidence(args.output_dir, "pre_joint_states.txt", joint_state_text("pre_joint_states", pre_positions))
            results.append(CheckResult("/joint_states baseline captured", "PASS", f"{len(pre_positions)} arm joints"))
        except Exception as exc:
            results.append(CheckResult("/joint_states baseline captured", "FAIL", str(exc)))
            summary = format_summary(results)
            write_evidence(args.output_dir, "summary.txt", summary)
            print(summary)
            return 1

        try:
            target_positions = choose_reversible_target(
                pre_positions,
                joint=args.joint,
                delta_rad=args.delta_rad,
                limit_margin_rad=args.limit_margin_rad,
            )
            write_json(args.output_dir, "target_joint_state.json", target_positions)
            target_delta = target_positions[args.joint] - pre_positions[args.joint]
            results.append(CheckResult("tiny reversible target selected", "PASS", f"{args.joint} delta={target_delta:.6f} rad"))
        except Exception as exc:
            results.append(CheckResult("tiny reversible target selected", "FAIL", str(exc)))
            summary = format_summary(results)
            write_evidence(args.output_dir, "summary.txt", summary)
            print(summary)
            return 1

        if any(not result.ok for result in results):
            summary = format_summary(results)
            write_evidence(args.output_dir, "summary.txt", summary)
            print(summary)
            return 1

        move_result, move_sent_at = node.send_move_group_goal(
            start_positions=pre_positions,
            target_positions=target_positions,
            group_name=args.group_name,
            planner_id=args.planner_id,
            planning_time=args.planning_time,
            velocity_scaling=args.velocity_scaling,
            acceleration_scaling=args.acceleration_scaling,
            execute=args.execute,
            timeout_sec=args.action_timeout,
        )
        move_summary = move_group_result_summary(move_result)
        move_summary["plan_only"] = not args.execute
        write_json(args.output_dir, "move_action_result.json", move_summary)
        if move_summary["success"]:
            mode = "executed" if args.execute else "plan_only"
            results.append(CheckResult("MoveGroup outbound goal", "PASS", mode))
        else:
            results.append(CheckResult("MoveGroup outbound goal", "FAIL", f"error_code={move_summary['error_code']}"))

        if args.execute and move_summary["success"]:
            after_message = node.wait_for_joint_state(args.joint_state_timeout, after_monotonic=move_sent_at)
            after_positions = arm_positions_from_joint_state(after_message)
            write_evidence(args.output_dir, "after_joint_states.txt", joint_state_text("after_joint_states", after_positions))
            moved_delta = after_positions[args.joint] - pre_positions[args.joint]
            expected_delta = target_positions[args.joint] - pre_positions[args.joint]
            if abs(moved_delta - expected_delta) <= 0.015:
                results.append(CheckResult("outbound joint delta observed", "PASS", f"{args.joint} delta={moved_delta:.6f} rad"))
            else:
                results.append(
                    CheckResult(
                        "outbound joint delta observed",
                        "FAIL",
                        f"{args.joint} delta={moved_delta:.6f} rad expected={expected_delta:.6f} rad",
                    )
                )

            if args.return_to_start:
                return_result, return_sent_at = node.send_move_group_goal(
                    start_positions=after_positions,
                    target_positions=pre_positions,
                    group_name=args.group_name,
                    planner_id=args.planner_id,
                    planning_time=args.planning_time,
                    velocity_scaling=args.velocity_scaling,
                    acceleration_scaling=args.acceleration_scaling,
                    execute=True,
                    timeout_sec=args.action_timeout,
                )
                return_summary = move_group_result_summary(return_result)
                return_summary["plan_only"] = False
                write_json(args.output_dir, "return_action_result.json", return_summary)
                if return_summary["success"]:
                    results.append(CheckResult("MoveGroup return goal", "PASS", "returned toward original joint state"))
                    final_message = node.wait_for_joint_state(args.joint_state_timeout, after_monotonic=return_sent_at)
                    final_positions = arm_positions_from_joint_state(final_message)
                    write_evidence(args.output_dir, "final_joint_states.txt", joint_state_text("final_joint_states", final_positions))
                    max_abs_final_delta = max(abs(final_positions[joint] - pre_positions[joint]) for joint in ARM_JOINTS)
                    if max_abs_final_delta <= 0.01:
                        results.append(CheckResult("final state near baseline", "PASS", f"max_abs_delta={max_abs_final_delta:.6f} rad"))
                    else:
                        results.append(CheckResult("final state near baseline", "FAIL", f"max_abs_delta={max_abs_final_delta:.6f} rad"))
                else:
                    results.append(CheckResult("MoveGroup return goal", "FAIL", f"error_code={return_summary['error_code']}"))
            else:
                results.append(CheckResult("return to start", "WARN", "disabled by --no-return-to-start"))
        elif not args.execute:
            write_evidence(args.output_dir, "after_joint_states.txt", "not captured: plan_only run did not execute motion")
            write_evidence(args.output_dir, "return_action_result.json", '{"not_run": "plan_only"}')
            write_evidence(args.output_dir, "final_joint_states.txt", "not captured: plan_only run did not execute motion")
            results.append(CheckResult("execution", "WARN", "plan_only default; no motion command executed"))

        write_json(
            args.output_dir,
            "delta_report.json",
            delta_report(pre_positions, after_positions, final_positions, target=target_positions, moved_joint=args.joint),
        )
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
