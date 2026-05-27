"""ROS-level dry-run test for the arrangement reasoning service.

Boots a minimal in-process stack:
- FakePerceptionNode serving three deterministic cubes.
- FakeActionAdapter that immediately confirms every PickAndPlace request.
- The real LangChainReasoningNode (mock provider).

Then calls /irb120pe/reasoning/arrange_objects and asserts on the response.
Skipped automatically if rclpy or the cognitive_interfaces package are not
available (e.g. on a native-Windows machine without a sourced ROS workspace).
"""

from __future__ import annotations

import json
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
try:
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from irb120pe_cognitive_interfaces.msg import DetectedObject
    from irb120pe_cognitive_interfaces.srv import (
        ArrangeObjects,
        GetDetectedObjects,
        MoveArm,
        PickAndPlace,
    )
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    from irb120pe_cognitive.langchain_reasoning_node import LangChainReasoningNode
except Exception as exc:  # pragma: no cover - skip when ROS env missing
    pytest.skip(f"ROS 2 runtime not available: {exc}", allow_module_level=True)


CUBES = (
    ("white_1", "WhiteCube", 0.55, 0.32, 0.88),
    ("black_1", "BlackCube", 0.55, 0.45, 0.88),
    ("blue_1", "BlueCube", 0.55, 0.58, 0.88),
)


class FakePerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_perception")
        self.create_service(
            GetDetectedObjects,
            "/irb120pe/perception/get_detected_objects",
            self._get_objects_cb,
        )

    def _get_objects_cb(self, _request, response):
        for object_id, label, x, y, z in CUBES:
            obj = DetectedObject()
            obj.object_id = object_id
            obj.label = label
            obj.confidence = 0.9
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            obj.pose = pose
            response.objects.append(obj)
        response.status = "fake"
        return response


class FakeActionAdapter(Node):
    def __init__(self) -> None:
        super().__init__("fake_action_adapter")
        self.pick_calls: list[dict] = []
        self.move_calls: list[dict] = []
        self.create_service(PickAndPlace, "/irb120pe/action/pick_and_place", self._pick_cb)
        self.create_service(MoveArm, "/irb120pe/action/move_arm", self._move_cb)

    def _pick_cb(self, request, response):
        self.pick_calls.append(
            {
                "object_id": request.object_id,
                "target_slot": request.target_slot,
                "target_pose": (
                    request.target_pose.pose.position.x,
                    request.target_pose.pose.position.y,
                    request.target_pose.pose.position.z,
                ),
            }
        )
        response.success = True
        response.status = "fake pick-and-place ok"
        return response

    def _move_cb(self, request, response):
        self.move_calls.append(
            {
                "x": request.target_pose.pose.position.x,
                "y": request.target_pose.pose.position.y,
                "z": request.target_pose.pose.position.z,
            }
        )
        response.success = True
        response.status = "fake move ok"
        return response


@pytest.fixture
def stack():
    rclpy.init()
    perception = FakePerceptionNode()
    action_adapter = FakeActionAdapter()
    reasoning = LangChainReasoningNode()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (perception, action_adapter, reasoning):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield perception, action_adapter, reasoning
    finally:
        executor.shutdown(timeout_sec=2.0)
        for node in (reasoning, action_adapter, perception):
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


def test_arrange_objects_service_returns_three_step_plan(stack):
    _, action_adapter, _ = stack

    rclpy.init() if not rclpy.ok() else None  # safety no-op
    client_node = Node("test_client")
    try:
        client = client_node.create_client(ArrangeObjects, "/irb120pe/reasoning/arrange_objects")
        assert client.wait_for_service(timeout_sec=10.0), "arrange_objects service not ready"

        # Subscribe to the trace topic to confirm at least one trace per step.
        trace_messages: list[str] = []
        client_node.create_subscription(
            String,
            "/irb120pe/reasoning/trace",
            lambda msg: trace_messages.append(msg.data),
            10,
        )

        request = ArrangeObjects.Request()
        request.instruction = "Arrange in a line by color from white to black to blue along Y at x=0.55, z=1.00, spacing=0.06"
        future = client.call_async(request)

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(client_node, timeout_sec=0.2)
        assert future.done(), "arrange_objects did not return"
        response = future.result()
        assert response.success, response.status
        plan = json.loads(response.plan_json)
        assert len(plan) == 3
        colors = [step["color"] for step in plan]
        assert colors == ["white", "black", "blue"]

        # Every emitted target_pose must be inside the configured workspace.
        for step in plan:
            pose = step["target_pose"]
            assert 0.05 <= pose["x"] <= 0.75
            assert 0.05 <= pose["y"] <= 0.85
            assert 0.95 <= pose["z"] <= 1.65

        # Action adapter must have received exactly three pick-and-place calls.
        assert len(action_adapter.pick_calls) == 3
        targets = [call["target_pose"] for call in action_adapter.pick_calls]
        # y monotonically increasing
        assert sorted(targets, key=lambda t: t[1]) == targets

        # Allow trace messages a moment to flush.
        time.sleep(1.0)
        # We expect at least one trace per executed step + the initial arrange trace.
        assert len(trace_messages) >= 3
    finally:
        client_node.destroy_node()
