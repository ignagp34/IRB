from __future__ import annotations

import math
import threading
from typing import Any, Sequence

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from irb120pe_cognitive_interfaces.srv import GetDetectedObjects, MoveArm, PickAndPlace
from linkattacher_msgs.srv import AttachLink, DetachLink
from ros2srrc_data.action import Move, Robmove
from ros2srrc_data.msg import Action

from .ros_helpers import declare_common_parameters, get_slots, get_workspace_limits, make_pose_stamped
from .validation import ValidationError, resolve_slot, select_object, validate_coordinates


MOVEIT_SUCCESS = 1
SUPPORTED_EXECUTION_BACKENDS = {"legacy", "moveit_sim"}


def normalize_execution_backend(value: Any) -> str:
    backend = str(value or "legacy").strip().lower()
    if backend not in SUPPORTED_EXECUTION_BACKENDS:
        valid = ", ".join(sorted(SUPPORTED_EXECUTION_BACKENDS))
        raise ValidationError(f"execution_backend must be one of: {valid}.")
    return backend


def build_move_group_pose_goal(
    pose: PoseStamped,
    *,
    group_name: str,
    end_effector_link: str,
    planner_id: str,
    planning_time: float,
    velocity_scaling: float,
    acceleration_scaling: float,
    position_tolerance: float,
    orientation_tolerance: float,
    collision_objects_to_remove: Sequence[str] = (),
) -> MoveGroup.Goal:
    goal = MoveGroup.Goal()
    goal.request.group_name = group_name
    goal.request.planner_id = planner_id
    goal.request.num_planning_attempts = 1
    goal.request.allowed_planning_time = planning_time
    goal.request.max_velocity_scaling_factor = velocity_scaling
    goal.request.max_acceleration_scaling_factor = acceleration_scaling

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [position_tolerance, position_tolerance, position_tolerance]

    position = PositionConstraint()
    position.header = pose.header
    position.link_name = end_effector_link
    position.constraint_region.primitives.append(primitive)
    position.constraint_region.primitive_poses.append(pose.pose)
    position.weight = 1.0

    orientation = OrientationConstraint()
    orientation.header = pose.header
    orientation.link_name = end_effector_link
    orientation.orientation = pose.pose.orientation
    orientation.absolute_x_axis_tolerance = orientation_tolerance
    orientation.absolute_y_axis_tolerance = orientation_tolerance
    orientation.absolute_z_axis_tolerance = orientation_tolerance
    orientation.weight = 1.0

    constraints = Constraints()
    constraints.name = f"{end_effector_link}_pose_goal"
    constraints.position_constraints.append(position)
    constraints.orientation_constraints.append(orientation)
    goal.request.goal_constraints.append(constraints)

    goal.planning_options.plan_only = False
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.planning_scene_diff.is_diff = True
    for object_id in collision_objects_to_remove:
        collision_object = CollisionObject()
        collision_object.header.frame_id = pose.header.frame_id
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE
        goal.planning_options.planning_scene_diff.world.collision_objects.append(collision_object)
    return goal


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
        self.declare_parameter("execution_backend", "legacy")
        self.declare_parameter("moveit_group_name", "irb120_arm")
        self.declare_parameter("moveit_end_effector_link", "tool0")
        self.declare_parameter("moveit_planner_id", "PTP")
        self.declare_parameter("moveit_planning_time", 10.0)
        self.declare_parameter("moveit_velocity_scaling", 0.05)
        self.declare_parameter("moveit_acceleration_scaling", 0.05)
        self.declare_parameter("moveit_action_timeout", 60.0)
        self.declare_parameter("moveit_position_tolerance", 0.01)
        self.declare_parameter("moveit_orientation_tolerance", 0.25)
        self.declare_parameter("moveit_grasp_orientation", [0.707, 0.707, 0.0, 0.0])
        self.declare_parameter("moveit_extra_collision_object_ids", ["sticker_1", "sticker_2", "sticker_3"])
        self.declare_parameter("moveit_use_gripper_controllers", True)
        self.declare_parameter("gripper_left_action", "/egp64_finger_left_controller/gripper_cmd")
        self.declare_parameter("gripper_right_action", "/egp64_finger_right_controller/gripper_cmd")
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_closed_position", 0.01)
        self.declare_parameter("gripper_max_effort", 10.0)
        self.declare_parameter("robot_model_name", "irb120")
        self.declare_parameter("robot_attach_link", "EE_egp64")

        self.workspace_limits = get_workspace_limits(self)
        self.slots = get_slots(self)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.execution_backend = normalize_execution_backend(self.get_parameter("execution_backend").value)
        self._moveit_sim_collision_objects_to_remove: list[str] = []
        self.cb_group = ReentrantCallbackGroup()

        self.robmove_client = ActionClient(self, Robmove, "Robmove", callback_group=self.cb_group)
        self.move_client = ActionClient(self, Move, "Move", callback_group=self.cb_group)
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self.left_gripper_client = ActionClient(
            self,
            GripperCommand,
            self.get_parameter("gripper_left_action").value,
            callback_group=self.cb_group,
        )
        self.right_gripper_client = ActionClient(
            self,
            GripperCommand,
            self.get_parameter("gripper_right_action").value,
            callback_group=self.cb_group,
        )
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
            if self.execution_backend == "moveit_sim":
                response.success, response.status = self._send_moveit_pose(request.target_pose)
            else:
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

            # Free target_pose overrides target_slot when populated. This is the path
            # used by the arrangement reasoning tool — the LLM picks the coordinates.
            target_pose_values: tuple[float, float, float, float, float, float, float]
            target_label: str
            if self._pose_has_position(request.target_pose):
                tx, ty, tz = validate_coordinates(
                    request.target_pose.pose.position.x,
                    request.target_pose.pose.position.y,
                    request.target_pose.pose.position.z,
                    self.workspace_limits,
                )
                tq = request.target_pose.pose.orientation
                if math.isclose(tq.x, 0.0) and math.isclose(tq.y, 0.0) and math.isclose(tq.z, 0.0) and math.isclose(tq.w, 0.0):
                    qx, qy, qz, qw = self._moveit_grasp_orientation()
                else:
                    qx, qy, qz, qw = tq.x, tq.y, tq.z, tq.w
                target_pose_values = (tx, ty, tz, qx, qy, qz, qw)
                target_label = f"free_pose({tx:.3f},{ty:.3f},{tz:.3f})"
            else:
                slot = resolve_slot(request.target_slot, self.slots)
                validate_coordinates(slot.pose[0], slot.pose[1], slot.pose[2], self.workspace_limits)
                target_pose_values = slot.pose
                target_label = slot.key

            x, y, _ = validate_coordinates(
                source_pose.pose.position.x,
                source_pose.pose.position.y,
                float(self.get_parameter("pick_z").value),
                self.workspace_limits,
            )

            if self.dry_run:
                response.success = True
                response.status = f"dry_run: would pick {request.object_id or 'source pose'} and place into {target_label}."
                return response

            source_model = self._gazebo_model_name(request.object_id, source_pose)
            if self.execution_backend == "moveit_sim" and request.object_id:
                extra_ids = [str(value) for value in self.get_parameter("moveit_extra_collision_object_ids").value]
                self._moveit_sim_collision_objects_to_remove = [request.object_id, *extra_ids]
            ok, status = self._execute_pick_and_place(x, y, source_pose, target_pose_values, source_model)
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
        if self.execution_backend == "moveit_sim":
            return self._execute_moveit_sim_pick_and_place(x, y, slot_pose_values, source_model)
        return self._execute_legacy_pick_and_place(x, y, source_pose, slot_pose_values, source_model)

    def _execute_legacy_pick_and_place(
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

    def _execute_moveit_sim_pick_and_place(
        self,
        x: float,
        y: float,
        slot_pose_values: tuple[float, float, float, float, float, float, float],
        source_model: str | None,
    ) -> tuple[bool, str]:
        if source_model is None:
            return False, "moveit_sim backend requires a Gazebo cube model name derived from object_id or frame_id."

        planning_frame = self.get_parameter("planning_frame").value
        qx, qy, qz, qw = self._moveit_grasp_orientation()
        pick_approach = self._pose(planning_frame, x, y, float(self.get_parameter("pick_approach_z").value), qx, qy, qz, qw)
        pick = self._pose(planning_frame, x, y, float(self.get_parameter("pick_z").value), qx, qy, qz, qw)
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
            ("approach source", lambda: self._send_moveit_pose(pick_approach)),
            ("descend to grasp", lambda: self._send_moveit_pose(pick)),
        ]
        if bool(self.get_parameter("moveit_use_gripper_controllers").value):
            steps.append(("close gripper", lambda: self._send_gripper_trajectory(float(self.get_parameter("gripper_closed_position").value))))
        steps.extend(
            [
                ("attach cube", lambda: self._attach(source_model)),
                ("lift cube", lambda: self._send_moveit_pose(pick_approach)),
                ("approach slot", lambda: self._send_moveit_pose(slot_approach)),
                ("descend to slot", lambda: self._send_moveit_pose(slot_place)),
            ]
        )
        if bool(self.get_parameter("moveit_use_gripper_controllers").value):
            steps.append(("open gripper", lambda: self._send_gripper_trajectory(float(self.get_parameter("gripper_open_position").value))))
        steps.extend(
            [
                ("detach cube", lambda: self._detach(source_model)),
                ("retreat from slot", lambda: self._send_moveit_pose(slot_approach)),
            ]
        )

        completed: list[str] = []
        for label, step in steps:
            ok, status = step()
            if not ok:
                prefix = f"completed={completed}; " if completed else ""
                return False, f"moveit_sim pick-and-place failed: {prefix}{label}: {status}"
            completed.append(label)
        return True, f"moveit_sim pick-and-place completed for {source_model}."

    def _send_moveit_pose(self, pose: PoseStamped) -> tuple[bool, str]:
        timeout = float(self.get_parameter("moveit_action_timeout").value)
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            return False, "/move_action action server is not available."
        goal = build_move_group_pose_goal(
            pose,
            group_name=str(self.get_parameter("moveit_group_name").value),
            end_effector_link=str(self.get_parameter("moveit_end_effector_link").value),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
            acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            position_tolerance=float(self.get_parameter("moveit_position_tolerance").value),
            orientation_tolerance=float(self.get_parameter("moveit_orientation_tolerance").value),
            collision_objects_to_remove=self._moveit_sim_collision_objects_to_remove,
        )
        future = self.move_group_client.send_goal_async(goal)
        if not self._wait_for_future(future, 10.0) or future.result() is None or not future.result().accepted:
            return False, "/move_action pose goal rejected or timed out."
        result_future = future.result().get_result_async()
        if not self._wait_for_future(result_future, timeout) or result_future.result() is None:
            return False, "/move_action pose result timed out."
        result = result_future.result().result
        error_code = int(result.error_code.val)
        if error_code != MOVEIT_SUCCESS:
            return False, f"/move_action pose goal failed with MoveIt error_code={error_code}."
        point_count = len(result.executed_trajectory.joint_trajectory.points)
        return True, f"/move_action pose goal executed with {point_count} trajectory points."

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

    def _send_gripper_trajectory(self, position: float) -> tuple[bool, str]:
        left = self._send_single_gripper_command(
            self.left_gripper_client,
            str(self.get_parameter("gripper_left_action").value),
            position,
        )
        right = self._send_single_gripper_command(
            self.right_gripper_client,
            str(self.get_parameter("gripper_right_action").value),
            position,
        )
        if left[0] and right[0]:
            return True, f"gripper controllers moved to {position:.3f} m."
        return False, " | ".join(status for ok, status in (left, right) if not ok)

    def _send_single_gripper_command(self, client: ActionClient, action_name: str, position: float) -> tuple[bool, str]:
        if not client.wait_for_server(timeout_sec=5.0):
            return False, f"{action_name} gripper action server is not available."
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.get_parameter("gripper_max_effort").value)
        future = client.send_goal_async(goal)
        if not self._wait_for_future(future, 10.0) or future.result() is None or not future.result().accepted:
            return False, f"{action_name} gripper goal rejected or timed out."
        result_future = future.result().get_result_async()
        if not self._wait_for_future(result_future, 20.0) or result_future.result() is None:
            return False, f"{action_name} gripper result timed out."
        result = result_future.result().result
        if bool(result.stalled):
            return False, f"{action_name} gripper action stalled."
        return True, f"{action_name} moved to {position:.3f} m."

    def _attach(self, model: str) -> tuple[bool, str]:
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            return False, "/ATTACHLINK service is not available."
        request = AttachLink.Request()
        request.model1_name = str(self.get_parameter("robot_model_name").value)
        request.link1_name = str(self.get_parameter("robot_attach_link").value)
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
        request.model1_name = str(self.get_parameter("robot_model_name").value)
        request.link1_name = str(self.get_parameter("robot_attach_link").value)
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

    def _moveit_grasp_orientation(self) -> tuple[float, float, float, float]:
        values = list(self.get_parameter("moveit_grasp_orientation").value)
        if len(values) != 4:
            raise ValidationError("moveit_grasp_orientation must contain qx, qy, qz, qw.")
        return tuple(float(value) for value in values)

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
