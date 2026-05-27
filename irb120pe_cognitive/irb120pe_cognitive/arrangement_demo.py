"""Reproducible arrangement demo entry-point.

Spawns the three default cubes (Blue, Black, White) in Gazebo, waits for the
perception node to publish them, calls the new arrangement reasoning service
with a canonical natural-language instruction, and prints the returned plan.

Designed for the demo video and for the live verification flow — never used
inside the automated unit tests.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Iterable

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from irb120pe_cognitive_interfaces.srv import ArrangeObjects, GetDetectedObjects

from .gazebo_cube_helper import GazeboCubeClient


DEFAULT_INSTRUCTION = (
    "Arrange the cubes in a line by color from white to black to blue "
    "along Y at x=0.55, z=0.90, spacing=0.06"
)

DEFAULT_CUBES = (
    {"cube": "WhiteCube", "name": "WhiteCube", "x": 0.55, "y": 0.32, "z": 0.88},
    {"cube": "BlackCube", "name": "BlackCube", "x": 0.55, "y": 0.45, "z": 0.88},
    {"cube": "BlueCube", "name": "BlueCube", "x": 0.55, "y": 0.58, "z": 0.88},
)


class ArrangementDemoClient(Node):
    def __init__(self) -> None:
        super().__init__("irb120pe_arrangement_demo")
        self.detected_objects_client = self.create_client(
            GetDetectedObjects,
            "/irb120pe/perception/get_detected_objects",
        )
        self.arrange_client = self.create_client(
            ArrangeObjects,
            "/irb120pe/reasoning/arrange_objects",
        )

    def wait_for_objects(self, expected_labels: Iterable[str], timeout_sec: float) -> bool:
        """Wait until each expected label (or its color root) is seen by perception.

        Perception emits "blue"/"black"/"white" while spawn entities are
        "BlueCube"/"BlackCube"/"WhiteCube". We accept either direction of
        substring match so the demo works with both naming styles.
        """

        deadline = time.monotonic() + timeout_sec
        targets = {label.lower() for label in expected_labels}
        if not self.detected_objects_client.wait_for_service(timeout_sec=10.0):
            return False
        while rclpy.ok() and time.monotonic() < deadline:
            future = self.detected_objects_client.call_async(GetDetectedObjects.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None:
                seen = {obj.label.lower() for obj in future.result().objects}
                if all(any(t in s or s in t for s in seen) for t in targets):
                    return True
            time.sleep(1.0)
        return False

    def call_arrangement(self, instruction: str, timeout_sec: float) -> ArrangeObjects.Response:
        if not self.arrange_client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError("/irb120pe/reasoning/arrange_objects service is not available.")
        request = ArrangeObjects.Request()
        request.instruction = instruction
        future = self.arrange_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        result = future.result()
        if result is None:
            raise TimeoutError("arrange_objects service timed out.")
        return result


def _pose_from(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0
    return pose


def run(args: argparse.Namespace) -> int:
    rclpy.init()
    cube_client = GazeboCubeClient()
    demo = ArrangementDemoClient()
    try:
        for cube in DEFAULT_CUBES:
            try:
                cube_client.delete(entity_name=cube["name"], timeout_sec=args.spawn_timeout)
            except Exception:
                pass
            response = cube_client.spawn(
                cube=cube["cube"],
                entity_name=cube["name"],
                pose=_pose_from(cube["x"], cube["y"], cube["z"]),
                reference_frame="world",
                timeout_sec=args.spawn_timeout,
            )
            print(f"spawn {cube['name']}: success={response.success} message={response.status_message}")
            if not response.success:
                return 1

        ok = demo.wait_for_objects(
            expected_labels=[cube["cube"] for cube in DEFAULT_CUBES],
            timeout_sec=args.detection_timeout,
        )
        print(f"perception ready: {ok}")
        if not ok:
            return 2

        result = demo.call_arrangement(args.instruction, timeout_sec=args.reasoning_timeout)
        print(f"arrangement success: {result.success}")
        print(f"arrangement status: {result.status}")
        try:
            plan = json.loads(result.plan_json or "[]")
            print("arrangement plan:")
            print(json.dumps(plan, indent=2))
        except json.JSONDecodeError:
            print("arrangement plan (raw):")
            print(result.plan_json)
        return 0 if result.success else 3
    finally:
        cube_client.destroy_node()
        demo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the goal-oriented arrangement demo.")
    parser.add_argument("--instruction", default=DEFAULT_INSTRUCTION)
    parser.add_argument("--spawn-timeout", type=float, default=30.0)
    parser.add_argument("--detection-timeout", type=float, default=60.0)
    parser.add_argument("--reasoning-timeout", type=float, default=300.0)
    return parser


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
