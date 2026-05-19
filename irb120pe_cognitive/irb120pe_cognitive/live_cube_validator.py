from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Iterable, Sequence

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node

from irb120pe_cognitive_interfaces.srv import ExecuteInstruction, GetDetectedObjects

from .gazebo_cube_helper import SUPPORTED_CUBES, expanded_cube_xml


DEFAULT_INSTRUCTION = "Pick the blue cube and place it in the right container"
DEFAULT_EXPECTED_REASONING = "dry_run: would pick blue_1 and place into slot_c."


def csv_values(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def object_id(obj) -> str:
    if isinstance(obj, dict):
        return str(obj.get("object_id", ""))
    return str(getattr(obj, "object_id", ""))


def object_label(obj) -> str:
    if isinstance(obj, dict):
        return str(obj.get("label", ""))
    return str(getattr(obj, "label", ""))


def matching_detections(objects: Iterable, *, expected_label: str, expected_ids: Sequence[str]) -> list:
    expected_id_set = set(expected_ids)
    matches = []
    for obj in objects:
        obj_id = object_id(obj)
        label = object_label(obj).lower()
        if expected_id_set and obj_id in expected_id_set:
            matches.append(obj)
        elif expected_label and expected_label.lower() in label:
            matches.append(obj)
    return matches


def planning_scene_has_ids(actual_ids: Iterable[str], expected_ids: Sequence[str]) -> bool:
    actual = set(actual_ids)
    return all(expected_id in actual for expected_id in expected_ids)


def planning_scene_is_clear(actual_ids: Iterable[str], expected_ids: Sequence[str]) -> bool:
    actual = set(actual_ids)
    return not any(expected_id in actual for expected_id in expected_ids)


def format_summary(results: Sequence[tuple[str, bool, str]]) -> str:
    lines = []
    for label, ok, detail in results:
        status = "PASS" if ok else "FAIL"
        suffix = f" - {detail}" if detail else ""
        lines.append(f"[{status}] {label}{suffix}")
    overall = "PASS" if all(ok for _, ok, _ in results) else "FAIL"
    return "\n".join([f"Live cube validation: {overall}", *lines])


def pose_from_args(args: argparse.Namespace) -> Pose:
    pose = Pose()
    pose.position.x = args.x
    pose.position.y = args.y
    pose.position.z = args.z
    pose.orientation.x = args.qx
    pose.orientation.y = args.qy
    pose.orientation.z = args.qz
    pose.orientation.w = args.qw
    return pose


def write_evidence(output_dir: Path | None, filename: str, content: str) -> None:
    if output_dir is None:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / filename).write_text(content.rstrip() + "\n", encoding="utf-8")


def detected_objects_text(objects: Sequence) -> str:
    if not objects:
        return "objects=[]"
    lines = ["objects=["]
    for obj in objects:
        pose = getattr(obj, "pose", None)
        position = getattr(getattr(pose, "pose", None), "position", None)
        if position is None:
            position_text = "pose=<unavailable>"
        else:
            position_text = f"x={position.x:.6f}, y={position.y:.6f}, z={position.z:.6f}"
        lines.append(
            f"  {object_id(obj)} label={object_label(obj)} "
            f"confidence={getattr(obj, 'confidence', 0.0):.6f} {position_text}"
        )
    lines.append("]")
    return "\n".join(lines)


class LiveCubeValidator(Node):
    def __init__(self) -> None:
        super().__init__("irb120pe_live_cube_validator")
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")
        self.detected_objects_client = self.create_client(
            GetDetectedObjects,
            "/irb120pe/perception/get_detected_objects",
        )
        self.planning_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.reasoning_client = self.create_client(
            ExecuteInstruction,
            "/irb120pe/reasoning/execute_instruction",
        )

    def spawn_cube(
        self,
        *,
        cube: str,
        entity_name: str,
        pose: Pose,
        reference_frame: str,
        timeout_sec: float,
    ) -> SpawnEntity.Response:
        if not self.spawn_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/spawn_entity service was not available before timeout.")
        request = SpawnEntity.Request()
        request.name = entity_name
        request.xml = expanded_cube_xml(cube, entity_name)
        request.initial_pose = pose
        request.reference_frame = reference_frame
        return self._call(self.spawn_client, request, timeout_sec)

    def delete_cube(self, *, entity_name: str, timeout_sec: float) -> DeleteEntity.Response:
        if not self.delete_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/delete_entity service was not available before timeout.")
        request = DeleteEntity.Request()
        request.name = entity_name
        return self._call(self.delete_client, request, timeout_sec)

    def get_detected_objects(self, timeout_sec: float) -> GetDetectedObjects.Response:
        if not self.detected_objects_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/irb120pe/perception/get_detected_objects was not available before timeout.")
        return self._call(self.detected_objects_client, GetDetectedObjects.Request(), timeout_sec)

    def get_planning_scene_ids(self, timeout_sec: float) -> list[str]:
        if not self.planning_scene_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/get_planning_scene was not available before timeout.")
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        response = self._call(self.planning_scene_client, request, timeout_sec)
        return [obj.id for obj in response.scene.world.collision_objects]

    def execute_instruction(self, instruction: str, timeout_sec: float) -> ExecuteInstruction.Response:
        if not self.reasoning_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/irb120pe/reasoning/execute_instruction was not available before timeout.")
        request = ExecuteInstruction.Request()
        request.instruction = instruction
        return self._call(self.reasoning_client, request, timeout_sec)

    def wait_for_detection(
        self,
        *,
        expected_label: str,
        expected_ids: Sequence[str],
        timeout_sec: float,
        poll_period_sec: float,
    ) -> GetDetectedObjects.Response:
        deadline = time.monotonic() + timeout_sec
        last_response = None
        while rclpy.ok() and time.monotonic() <= deadline:
            last_response = self.get_detected_objects(timeout_sec=min(3.0, timeout_sec))
            if matching_detections(last_response.objects, expected_label=expected_label, expected_ids=expected_ids):
                return last_response
            time.sleep(poll_period_sec)
        if last_response is not None:
            return last_response
        raise TimeoutError("No perception response was available before timeout.")

    def wait_for_no_detection(
        self,
        *,
        expected_label: str,
        expected_ids: Sequence[str],
        timeout_sec: float,
        poll_period_sec: float,
    ) -> GetDetectedObjects.Response:
        deadline = time.monotonic() + timeout_sec
        last_response = None
        while rclpy.ok() and time.monotonic() <= deadline:
            last_response = self.get_detected_objects(timeout_sec=min(3.0, timeout_sec))
            matches = matching_detections(
                last_response.objects,
                expected_label=expected_label,
                expected_ids=expected_ids,
            )
            if not matches:
                return last_response
            time.sleep(poll_period_sec)
        if last_response is not None:
            return last_response
        raise TimeoutError("No perception response was available before timeout.")

    def wait_for_planning_scene_ids(
        self,
        *,
        expected_ids: Sequence[str],
        should_exist: bool,
        timeout_sec: float,
        poll_period_sec: float,
    ) -> list[str]:
        deadline = time.monotonic() + timeout_sec
        last_ids: list[str] = []
        while rclpy.ok() and time.monotonic() <= deadline:
            last_ids = self.get_planning_scene_ids(timeout_sec=min(3.0, timeout_sec))
            if should_exist and planning_scene_has_ids(last_ids, expected_ids):
                return last_ids
            if not should_exist and planning_scene_is_clear(last_ids, expected_ids):
                return last_ids
            time.sleep(poll_period_sec)
        return last_ids

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
            "Validate the live Gazebo cube perception -> Planning Scene -> mock reasoning "
            "path against an already-running dry-run cognitive stack."
        )
    )
    parser.add_argument("--cube", choices=SUPPORTED_CUBES, default="BlueCube")
    parser.add_argument("--name", default=None, help="Gazebo entity name. Defaults to --cube.")
    parser.add_argument("--x", type=float, default=0.55)
    parser.add_argument("--y", type=float, default=0.52)
    parser.add_argument("--z", type=float, default=0.88)
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=1.0)
    parser.add_argument("--reference-frame", default="world")
    parser.add_argument("--no-replace", action="store_true", help="Do not delete an existing entity before spawning.")
    parser.add_argument("--service-timeout", type=float, default=30.0)
    parser.add_argument("--detection-timeout", type=float, default=45.0)
    parser.add_argument("--planning-scene-timeout", type=float, default=30.0)
    parser.add_argument("--reasoning-timeout", type=float, default=120.0)
    parser.add_argument("--post-delete-wait", type=float, default=8.0)
    parser.add_argument("--poll-period", type=float, default=1.0)
    parser.add_argument("--instruction", default=DEFAULT_INSTRUCTION)
    parser.add_argument("--expected-label", default="blue")
    parser.add_argument("--expected-object-ids", type=csv_values, default=["blue_1"])
    parser.add_argument("--expected-planning-scene-ids", type=csv_values, default=["blue_1", "sticker_1"])
    parser.add_argument("--expected-reasoning-status", default=DEFAULT_EXPECTED_REASONING)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser


def run(args: argparse.Namespace) -> int:
    output_dir = args.output_dir
    entity_name = args.name or args.cube
    results: list[tuple[str, bool, str]] = []
    spawned = False

    rclpy.init()
    node = LiveCubeValidator()
    try:
        if not args.no_replace:
            delete_response = node.delete_cube(entity_name=entity_name, timeout_sec=args.service_timeout)
            write_evidence(
                output_dir,
                "predelete_response.txt",
                f"delete {delete_response.success} {delete_response.status_message}",
            )
            ok = delete_response.success or "does not exist" in delete_response.status_message
            results.append(("predelete existing cube", ok, delete_response.status_message))
            if not ok:
                summary = format_summary(results)
                write_evidence(output_dir, "summary.txt", summary)
                print(summary)
                return 1

        spawn_response = node.spawn_cube(
            cube=args.cube,
            entity_name=entity_name,
            pose=pose_from_args(args),
            reference_frame=args.reference_frame,
            timeout_sec=args.service_timeout,
        )
        write_evidence(
            output_dir,
            "spawn_response.txt",
            f"spawn {spawn_response.success} {spawn_response.status_message}",
        )
        results.append(("spawn cube", bool(spawn_response.success), spawn_response.status_message))
        if not spawn_response.success:
            summary = format_summary(results)
            write_evidence(output_dir, "summary.txt", summary)
            print(summary)
            return 1
        spawned = True

        detected_response = node.wait_for_detection(
            expected_label=args.expected_label,
            expected_ids=args.expected_object_ids,
            timeout_sec=args.detection_timeout,
            poll_period_sec=args.poll_period,
        )
        write_evidence(output_dir, "detected_objects_after_spawn.txt", detected_objects_text(detected_response.objects))
        detection_matches = matching_detections(
            detected_response.objects,
            expected_label=args.expected_label,
            expected_ids=args.expected_object_ids,
        )
        results.append(
            (
                "wait for detections",
                bool(detection_matches),
                f"matched {[object_id(obj) for obj in detection_matches]}",
            )
        )
        detected_ids = {object_id(obj) for obj in detected_response.objects}
        expected_scene_ids = [
            expected_id for expected_id in args.expected_planning_scene_ids if expected_id in detected_ids
        ]
        if not expected_scene_ids:
            expected_scene_ids = [object_id(obj) for obj in detection_matches]

        scene_ids = node.wait_for_planning_scene_ids(
            expected_ids=expected_scene_ids,
            should_exist=True,
            timeout_sec=args.planning_scene_timeout,
            poll_period_sec=args.poll_period,
        )
        write_evidence(output_dir, "planning_scene_after_spawn_ids.txt", "\n".join(scene_ids) or "collision_object_ids=[]")
        results.append(
            (
                "verify Planning Scene add",
                planning_scene_has_ids(scene_ids, expected_scene_ids),
                f"ids={scene_ids}",
            )
        )

        reasoning_response = node.execute_instruction(args.instruction, timeout_sec=args.reasoning_timeout)
        write_evidence(
            output_dir,
            "reasoning_response.txt",
            (
                f"success={reasoning_response.success}\n"
                f"status={reasoning_response.status}\n"
                f"tool_trace={reasoning_response.tool_trace}"
            ),
        )
        reasoning_ok = bool(reasoning_response.success) and args.expected_reasoning_status in reasoning_response.status
        results.append(("mock reasoning dry-run", reasoning_ok, reasoning_response.status))

        delete_response = node.delete_cube(entity_name=entity_name, timeout_sec=args.service_timeout)
        write_evidence(
            output_dir,
            "delete_response.txt",
            f"delete {delete_response.success} {delete_response.status_message}",
        )
        results.append(("delete cube", bool(delete_response.success), delete_response.status_message))
        spawned = False
        if args.post_delete_wait > 0.0:
            time.sleep(args.post_delete_wait)

        after_delete_response = node.wait_for_no_detection(
            expected_label=args.expected_label,
            expected_ids=args.expected_object_ids,
            timeout_sec=args.detection_timeout,
            poll_period_sec=args.poll_period,
        )
        write_evidence(
            output_dir,
            "detected_objects_after_delete.txt",
            detected_objects_text(after_delete_response.objects),
        )
        stale_matches = matching_detections(
            after_delete_response.objects,
            expected_label=args.expected_label,
            expected_ids=args.expected_object_ids,
        )
        results.append(("verify perception stale removal", not stale_matches, f"remaining={[object_id(obj) for obj in stale_matches]}"))

        after_delete_scene_ids = node.wait_for_planning_scene_ids(
            expected_ids=expected_scene_ids,
            should_exist=False,
            timeout_sec=args.planning_scene_timeout,
            poll_period_sec=args.poll_period,
        )
        write_evidence(
            output_dir,
            "planning_scene_after_delete_ids.txt",
            "\n".join(after_delete_scene_ids) or "collision_object_ids=[]",
        )
        results.append(
            (
                "verify Planning Scene stale removal",
                planning_scene_is_clear(after_delete_scene_ids, expected_scene_ids),
                f"ids={after_delete_scene_ids}",
            )
        )

        summary = format_summary(results)
        write_evidence(output_dir, "summary.txt", summary)
        print(summary)
        return 0 if all(ok for _, ok, _ in results) else 1
    finally:
        if spawned:
            try:
                cleanup_response = node.delete_cube(entity_name=entity_name, timeout_sec=args.service_timeout)
                write_evidence(
                    output_dir,
                    "delete_cleanup_response.txt",
                    f"delete {cleanup_response.success} {cleanup_response.status_message}",
                )
            except Exception as exc:
                write_evidence(output_dir, "delete_cleanup_response.txt", f"cleanup failed: {exc}")
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
