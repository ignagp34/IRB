from __future__ import annotations

import json
import os
import threading
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from irb120pe_cognitive_interfaces.srv import ExecuteInstruction, GetDetectedObjects, MoveArm, PickAndPlace

from .ros_helpers import declare_common_parameters, get_slots, get_workspace_limits, pose_to_dict
from .validation import (
    ValidationError,
    resolve_slot,
    select_object,
    slots_as_tool_payload,
    validate_coordinates,
)


class LangChainReasoningNode(Node):
    """LLM-facing cognitive brain constrained to explicit ROS tools."""

    def __init__(self) -> None:
        super().__init__("irb120pe_langchain_reasoning")
        declare_common_parameters(self)
        self.declare_parameter("llm_provider", "mock")
        self.declare_parameter("llm_model", "gpt-4o-mini")
        self.declare_parameter("ollama_base_url", "http://localhost:11434")
        self.declare_parameter("huggingface_endpoint_url", "")
        self.declare_parameter("detected_objects_service", "/irb120pe/perception/get_detected_objects")
        self.declare_parameter("move_arm_service", "/irb120pe/action/move_arm")
        self.declare_parameter("pick_and_place_service", "/irb120pe/action/pick_and_place")
        self.declare_parameter("execute_instruction_service", "/irb120pe/reasoning/execute_instruction")

        self.workspace_limits = get_workspace_limits(self)
        self.slots = get_slots(self)
        self.cb_group = ReentrantCallbackGroup()
        self.detected_objects_client = self.create_client(
            GetDetectedObjects,
            self.get_parameter("detected_objects_service").value,
            callback_group=self.cb_group,
        )
        self.move_arm_client = self.create_client(MoveArm, self.get_parameter("move_arm_service").value, callback_group=self.cb_group)
        self.pick_and_place_client = self.create_client(
            PickAndPlace,
            self.get_parameter("pick_and_place_service").value,
            callback_group=self.cb_group,
        )
        self.create_service(
            ExecuteInstruction,
            self.get_parameter("execute_instruction_service").value,
            self._execute_instruction_cb,
            callback_group=self.cb_group,
        )

    def _execute_instruction_cb(
        self,
        request: ExecuteInstruction.Request,
        response: ExecuteInstruction.Response,
    ) -> ExecuteInstruction.Response:
        instruction = request.instruction.strip()
        if not instruction:
            response.success = False
            response.status = "Instruction is empty."
            response.tool_trace = "[]"
            return response
        try:
            provider = str(self.get_parameter("llm_provider").value).lower()
            if provider == "mock":
                result = self._run_mock_agent(instruction)
            else:
                result = self._run_langchain_agent(instruction, provider)
            response.success = bool(result["success"])
            response.status = str(result["status"])
            response.tool_trace = json.dumps(result["tool_trace"], indent=2)
        except Exception as exc:
            response.success = False
            response.status = str(exc)
            response.tool_trace = "[]"
        return response

    def tool_get_detected_objects(self) -> list[dict[str, Any]]:
        """Read-only tool: returns current perception objects and never moves the robot."""
        if not self.detected_objects_client.wait_for_service(timeout_sec=2.0):
            raise ValidationError("Detected objects service is not available.")
        future = self.detected_objects_client.call_async(GetDetectedObjects.Request())
        if not self._wait_for_future(future, 3.0) or future.result() is None:
            raise ValidationError("Timed out while reading detected objects.")
        return [
            {
                "object_id": obj.object_id,
                "label": obj.label,
                "confidence": float(obj.confidence),
                "pose": pose_to_dict(obj.pose),
                "timestamp": {
                    "sec": obj.stamp.sec,
                    "nanosec": obj.stamp.nanosec,
                },
                "_pose_msg": obj.pose,
            }
            for obj in future.result().objects
        ]

    def tool_get_available_slots(self) -> list[dict[str, Any]]:
        """Read-only tool: returns valid destination slots/containers."""
        return slots_as_tool_payload(self.slots)

    def tool_move_arm(self, x: Any, y: Any, z: Any, qx: float = 0.0, qy: float = 1.0, qz: float = 0.0, qw: float = 0.0) -> dict[str, Any]:
        """Motion tool: validates explicit target coordinates before calling MoveIt."""
        x, y, z = validate_coordinates(x, y, z, self.workspace_limits)
        request = MoveArm.Request()
        request.motion_type = "PTP"
        request.speed = 0.2
        request.target_pose = PoseStamped()
        request.target_pose.header.frame_id = self.get_parameter("planning_frame").value
        request.target_pose.pose.position.x = x
        request.target_pose.pose.position.y = y
        request.target_pose.pose.position.z = z
        request.target_pose.pose.orientation.x = float(qx)
        request.target_pose.pose.orientation.y = float(qy)
        request.target_pose.pose.orientation.z = float(qz)
        request.target_pose.pose.orientation.w = float(qw)
        result = self._call_move_arm(request)
        return {"success": result.success, "status": result.status}

    def tool_pick_and_place(self, object_id: str, target_slot: str, source_pose: PoseStamped | None = None) -> dict[str, Any]:
        """Pick-place tool: executes only for a valid object id/source pose and known slot."""
        slot = resolve_slot(target_slot, self.slots)
        request = PickAndPlace.Request()
        request.object_id = object_id
        request.target_slot = slot.key
        if source_pose is not None:
            request.source_pose = source_pose
        result = self._call_pick_and_place(request)
        return {"success": result.success, "status": result.status}

    def _run_mock_agent(self, instruction: str) -> dict[str, Any]:
        trace: list[dict[str, Any]] = []
        objects = self.tool_get_detected_objects()
        public_objects = [{k: v for k, v in obj.items() if k != "_pose_msg"} for obj in objects]
        trace.append({"tool": "get_detected_objects", "output": public_objects})
        slots = self.tool_get_available_slots()
        trace.append({"tool": "get_available_slots", "output": slots})

        lower = instruction.lower()
        if "sort all" in lower or "all visible" in lower:
            results = []
            for obj in objects:
                target_slot = self._slot_for_label(obj["label"])
                result = self.tool_pick_and_place(obj["object_id"], target_slot, obj["_pose_msg"])
                results.append({"object_id": obj["object_id"], "target_slot": target_slot, "result": result})
            trace.append({"tool": "pick_and_place_tool", "output": results})
            success = all(item["result"]["success"] for item in results)
            return {"success": success, "status": "Sort-all command completed." if success else "Sort-all command failed.", "tool_trace": trace}

        requested_label = self._label_from_instruction(lower)
        selected = select_object(objects, label=requested_label)
        target_slot = self._slot_from_instruction(lower) or self._slot_for_label(requested_label)
        result = self.tool_pick_and_place(selected["object_id"], target_slot, selected["_pose_msg"])
        trace.append(
            {
                "tool": "pick_and_place_tool",
                "input": {"object_id": selected["object_id"], "target_slot": target_slot},
                "output": result,
            }
        )
        return {"success": result["success"], "status": result["status"], "tool_trace": trace}

    def _run_langchain_agent(self, instruction: str, provider: str) -> dict[str, Any]:
        try:
            from langchain.agents import AgentType, initialize_agent
            from langchain.tools import StructuredTool
        except Exception as exc:
            raise ValidationError("LangChain is not installed. Use llm_provider:=mock or install LangChain dependencies.") from exc

        llm = self._build_llm(provider)
        tools = [
            StructuredTool.from_function(
                func=lambda: json.dumps([{k: v for k, v in obj.items() if k != "_pose_msg"} for obj in self.tool_get_detected_objects()]),
                name="get_detected_objects",
                description=(
                    "Read-only. Returns detected objects with object_id, label, confidence, pose, and timestamp. "
                    "It must not move the robot or modify the Planning Scene."
                ),
            ),
            StructuredTool.from_function(
                func=lambda: json.dumps(self.tool_get_available_slots()),
                name="get_available_slots",
                description=(
                    "Read-only. Returns valid destination slots/containers and coordinates. "
                    "Use this before selecting a destination; never invent slot names."
                ),
            ),
            StructuredTool.from_function(
                func=lambda x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0: json.dumps(self.tool_move_arm(x, y, z, qx, qy, qz, qw)),
                name="move_arm_tool",
                description=(
                    "Moves the robot only to explicit numeric x, y, z coordinates in the planning frame. "
                    "Rejects missing, nonnumeric, or out-of-bounds coordinates."
                ),
            ),
            StructuredTool.from_function(
                func=lambda object_id, target_slot: json.dumps(self.tool_pick_and_place(object_id, target_slot)),
                name="pick_and_place_tool",
                description=(
                    "Executes a complete pick-and-place operation for an existing object_id and valid target slot. "
                    "Use only after reading detections and available slots."
                ),
            ),
        ]
        prompt = (
            "You are the safe cognitive planner for an ABB IRB-120 sorting cell. "
            "You may only use the provided tools. Reject ambiguous or unsafe instructions. "
            f"Instruction: {instruction}"
        )
        agent = initialize_agent(tools, llm, agent=AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION, verbose=False)
        output = agent.run(prompt)
        return {"success": True, "status": str(output), "tool_trace": [{"provider": provider, "agent_output": str(output)}]}

    def _build_llm(self, provider: str) -> Any:
        model = str(self.get_parameter("llm_model").value)
        if provider == "openai":
            if not os.environ.get("OPENAI_API_KEY"):
                raise ValidationError("OPENAI_API_KEY is not set.")
            from langchain_openai import ChatOpenAI

            return ChatOpenAI(model=model, temperature=0)
        if provider == "ollama":
            from langchain_ollama import ChatOllama

            return ChatOllama(model=model, base_url=str(self.get_parameter("ollama_base_url").value), temperature=0)
        if provider == "huggingface":
            endpoint = str(self.get_parameter("huggingface_endpoint_url").value) or os.environ.get("HUGGINGFACE_ENDPOINT_URL", "")
            if not endpoint:
                raise ValidationError("Hugging Face endpoint URL is not configured.")
            from langchain_huggingface import HuggingFaceEndpoint

            return HuggingFaceEndpoint(endpoint_url=endpoint, temperature=0)
        raise ValidationError(f"Unsupported llm_provider '{provider}'.")

    def _call_move_arm(self, request: MoveArm.Request) -> MoveArm.Response:
        if not self.move_arm_client.wait_for_service(timeout_sec=2.0):
            raise ValidationError("move_arm service is not available.")
        future = self.move_arm_client.call_async(request)
        if not self._wait_for_future(future, 60.0) or future.result() is None:
            raise ValidationError("move_arm service timed out.")
        return future.result()

    def _call_pick_and_place(self, request: PickAndPlace.Request) -> PickAndPlace.Response:
        if not self.pick_and_place_client.wait_for_service(timeout_sec=2.0):
            raise ValidationError("pick_and_place service is not available.")
        future = self.pick_and_place_client.call_async(request)
        if not self._wait_for_future(future, 120.0) or future.result() is None:
            raise ValidationError("pick_and_place service timed out.")
        return future.result()

    def _label_from_instruction(self, lower_instruction: str) -> str:
        for label in ("blue", "black", "white", "red", "cube"):
            if label in lower_instruction:
                return label
        raise ValidationError("Instruction does not identify which cube to pick.")

    def _slot_from_instruction(self, lower_instruction: str) -> str | None:
        for slot in self.slots.values():
            for alias in slot.aliases:
                if alias.replace("_", " ") in lower_instruction or alias in lower_instruction:
                    return slot.key
        return None

    def _slot_for_label(self, label: str) -> str:
        label = label.lower()
        if "white" in label:
            return "slot_a"
        if "black" in label:
            return "slot_b"
        if "blue" in label:
            return "slot_c"
        return "unknown_slot"

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        return event.wait(timeout_sec)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LangChainReasoningNode()
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
