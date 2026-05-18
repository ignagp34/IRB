from __future__ import annotations

import math
import threading
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from irb120pe_cognitive_interfaces.srv import GetDetectedObjects, MoveArm, PickAndPlace
from linkattacher_msgs.srv import AttachLink, DetachLink
from ros2srrc_data.action import Move, Robmove
from ros2srrc_data.msg import Action

from .ros_helpers import declare_common_parameters, get_slots, get_workspace_limits, make_pose_stamped
from .validation import ValidationError, resolve_slot, select_object, validate_coordinates


class ActionAdapterNode(Node):
    """Tool-gated robot action adapter for LangChain and deterministic clients."""

    def __init__(self) -> None:
        super().__init__("irb120pe_action_adapter")
        declare_common_parameters(self)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("detected_objects_service", "/irb120pe/perception/get_detected_objects")
        self.declare_parameter("move_arm_service", "/irb120pe/action/move_arm")
        self.declare_parameter("pick_and_place_service", "/irb120pe/action/pick_and_place")
        self.declare_parameter("pick_approach_z", 1.10)
        self.declare_parameter("pick_z", 1.07)
        self.declare_parameter("place_approach_offset_z", 0.031)

        self.workspace_limits = get_workspace_limits(self)
        self.slots = get_slots(self)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.cb_group = ReentrantCallbackGroup()

        self.robmove_client = ActionClient(self, Robmove, "Robmove", callback_group=self.cb_group)
        self.move_client = ActionClient(self, Move, "Move", callback_group=self.cb_group)
        self.attach_client = self.create_client(AttachLink, "/ATTACHLINK", callback_group=self.cb_group)
        self.detach_client = self.create_client(DetachLink, "/DETACHLINK", callback_group=self.cb_group)
        self.detected_objects_client = self.create_client(
            GetDetectedObjects,
            self.get_parameter("detected_objects_service").value,
            callback_group=self.cb_group,
        )

        self.create_service(MoveArm, self.get_parameter("move_arm_service").value, self._move_arm_cb, callback_group=self.cb_group)
        self.create_service(PickAndPlace, self.get_parameter("pick_and_place_service").value, self._pick_and_place_cb, callback_group=self.cb_group)

    def _move_arm_cb(self, request: MoveArm.Request, response: MoveArm.Response) -> MoveArm.Response:
        try:
            validate_coordinates(
                request.target_pose.pose.position.x,
                request.target_pose.pose.position.y,
                request.target_pose.pose.position.z,
                self.workspace_limits,
            )
            motion_type = (request.motion_type or "PTP").upper()
            if motion_type not in {"PTP", "LIN"}:
                raise ValidationError("motion_type must be PTP or LIN.")
            speed = float(request.speed or 0.2)
            if speed <= 0.0 or speed > 1.0:
                raise ValidationError("speed must be within (0.0, 1.0].")
            if self.dry_run:
                response.success = True
                response.status = "dry_run: move_arm request validated but not executed."
                return response
            response.success, response.status = self._send_robmove(motion_type, speed, request.target_pose)
        except Exception as exc:
            response.success = False
            response.status = str(exc)
        return response

    def _pick_and_place_cb(self, request: PickAndPlace.Request, response: PickAndPlace.Response) -> PickAndPlace.Response:
        try:
            source_pose = request.source_pose
            if not self._pose_has_position(source_pose):
                source_pose = self._lookup_source_pose(request.object_id)
            slot = resolve_slot(request.target_slot, self.slots)
            x, y, _ = validate_coordinates(
                source_pose.pose.position.x,
                source_pose.pose.position.y,
                float(self.get_parameter("pick_z").value),
                self.workspace_limits,
            )
            validate_coordinates(slot.pose[0], slot.pose[1], slot.pose[2], self.workspace_limits)

            if self.dry_run:
                response.success = True
                response.status = f"dry_run: would pick {request.object_id or 'source pose'} and place into {slot.key}."
                return response

            source_model = self._gazebo_model_name(request.object_id, source_pose)
            ok, status = self._execute_pick_and_place(x, y, source_pose, slot.pose, source_model)
            response.success = ok
            response.status = status
        except Exception as exc:
            response.success = False
            response.status = str(exc)
        return response

    def _lookup_source_pose(self, object_id: str) -> PoseStamped:
        if not object_id:
            raise ValidationError("pick_and_place_tool requires source object id or source pose.")
        if not self.detected_objects_client.wait_for_service(timeout_sec=1.0):
            raise ValidationError("Detected objects service is not available.")
        future = self.detected_objects_client.call_async(GetDetectedObjects.Request())
        if not self._wait_for_future(future, 2.0) or future.result() is None:
            raise ValidationError("Timed out while querying detected objects.")
        objects = [
            {
                "object_id": obj.object_id,
                "label": obj.label,
                "confidence": obj.confidence,
                "pose": obj.pose,
            }
            for obj in future.result().objects
        ]
        selected = select_object(objects, object_id=object_id)
        return selected["pose"]

    def _execute_pick_and_place(
        self,
        x: float,
        y: float,
        source_pose: PoseStamped,
        slot_pose_values: tuple[float, float, float, float, float, float, float],
        source_model: str | None,
    ) -> tuple[bool, str]:
        planning_frame = self.get_parameter("planning_frame").value
        q = source_pose.pose.orientation
        if math.isclose(q.x, 0.0) and math.isclose(q.y, 0.0) and math.isclose(q.z, 0.0) and math.isclose(q.w, 0.0):
            q.x, q.y, q.z, q.w = 0.0, 1.0, 0.0, 0.0

        pick_approach = self._pose(planning_frame, x, y, float(self.get_parameter("pick_approach_z").value), q.x, q.y, q.z, q.w)
        pick = self._pose(planning_frame, x, y, float(self.get_parameter("pick_z").value), q.x, q.y, q.z, q.w)
        slot_place = make_pose_stamped(planning_frame, slot_pose_values)
        slot_approach = make_pose_stamped(
            planning_frame,
            (
                slot_pose_values[0],
                slot_pose_values[1],
                slot_pose_values[2] + float(self.get_parameter("place_approach_offset_z").value),
                slot_pose_values[3],
                slot_pose_values[4],
                slot_pose_values[5],
                slot_pose_values[6],
            ),
        )

        steps = [
            self._send_robmove("PTP", 0.3, pick_approach),
            self._send_robmove("LIN", 0.1, pick),
            self._send_gripper(0.008),
        ]
        if source_model:
            steps.append(self._attach(source_model))
        steps.extend(
            [
                self._send_robmove("LIN", 0.1, pick_approach),
                self._send_robmove("PTP", 0.3, slot_approach),
                self._send_robmove("LIN", 0.1, slot_place),
                self._send_gripper(0.0),
            ]
        )
        if source_model:
            steps.append(self._detach(source_model))
        steps.append(self._send_robmove("LIN", 0.1, slot_approach))

        failures = [status for ok, status in steps if not ok]
        if failures:
            return False, "pick-and-place failed: " + " | ".join(failures)
        return True, "pick-and-place completed."

    def _send_robmove(self, motion_type: str, speed: float, pose: PoseStamped) -> tuple[bool, str]:
        if not self.robmove_client.wait_for_server(timeout_sec=5.0):
            return False, "/Robmove action server is not available."
        goal = Robmove.Goal()
        goal.type = motion_type
        goal.speed = speed
        goal.x = pose.pose.position.x
        goal.y = pose.pose.position.y
        goal.z = pose.pose.position.z
        goal.qx = pose.pose.orientation.x
        goal.qy = pose.pose.orientation.y
        goal.qz = pose.pose.orientation.z
        goal.qw = pose.pose.orientation.w
        future = self.robmove_client.send_goal_async(goal)
        if not self._wait_for_future(future, 10.0) or future.result() is None or not future.result().accepted:
            return False, "/Robmove goal rejected or timed out."
        result_future = future.result().get_result_async()
        if not self._wait_for_future(result_future, 30.0) or result_future.result() is None:
            return False, "/Robmove result timed out."
        result = result_future.result().result
        return bool(result.success), str(result.message)

    def _send_gripper(self, opening_m: float) -> tuple[bool, str]:
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return False, "/Move action server is not available."
        action = Action()
        action.action = "MoveG"
        action.speed = 1.0
        action.moveg = float(opening_m)
        goal = Move.Goal()
        goal.action = action.action
        goal.speed = action.speed
        goal.moveg = action.moveg
        future = self.move_client.send_goal_async(goal)
        if not self._wait_for_future(future, 10.0) or future.result() is None or not future.result().accepted:
            return False, "/Move gripper goal rejected or timed out."
        result_future = future.result().get_result_async()
        if not self._wait_for_future(result_future, 20.0) or result_future.result() is None:
            return False, "/Move gripper result timed out."
        message = str(result_future.result().result.result)
        return "FAILED" not in message, message

    def _attach(self, model: str) -> tuple[bool, str]:
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            return False, "/ATTACHLINK service is not available."
        request = AttachLink.Request()
        request.model1_name = "irb120"
        request.link1_name = "EE_egp64"
        request.model2_name = model
        request.link2_name = model
        future = self.attach_client.call_async(request)
        if not self._wait_for_future(future, 5.0) or future.result() is None:
            return False, "/ATTACHLINK timed out."
        return bool(future.result().success), str(future.result().message)

    def _detach(self, model: str) -> tuple[bool, str]:
        if not self.detach_client.wait_for_service(timeout_sec=2.0):
            return False, "/DETACHLINK service is not available."
        request = DetachLink.Request()
        request.model1_name = "irb120"
        request.link1_name = "EE_egp64"
        request.model2_name = model
        request.link2_name = model
        future = self.detach_client.call_async(request)
        if not self._wait_for_future(future, 5.0) or future.result() is None:
            return False, "/DETACHLINK timed out."
        return bool(future.result().success), str(future.result().message)

    def _gazebo_model_name(self, object_id: str, source_pose: PoseStamped) -> str | None:
        token = (object_id or "").lower()
        if "blue" in token:
            return "BlueCube"
        if "black" in token:
            return "BlackCube"
        if "white" in token:
            return "WhiteCube"
        if "cube" in token:
            return "Cube"
        frame_token = source_pose.header.frame_id.lower()
        if "blue" in frame_token:
            return "BlueCube"
        if "black" in frame_token:
            return "BlackCube"
        if "white" in frame_token:
            return "WhiteCube"
        return None

    def _pose_has_position(self, pose: PoseStamped) -> bool:
        return any(
            abs(value) > 1e-9
            for value in (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        )

    def _pose(self, frame_id: str, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        return event.wait(timeout_sec)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActionAdapterNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.remove_node(node)
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
