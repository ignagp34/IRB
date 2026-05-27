# Cognitive Industrial Sorting Cobot Architecture

The system extends the original ABB IRB-120 pose-estimation demo with a tool-gated reasoning layer. The legacy deterministic scripts remain available, while the cognitive stack is launched through separate ROS 2 nodes.

## Runtime Flow

1. Gazebo and MoveIt 2 start the ABB IRB-120, Schunk gripper, camera, controllers, and MoveIt action servers. Legacy `/Move` and `/Robmove` interfaces are enabled only when their dependency chain is available.
2. `perception_node` subscribes to the simulated camera, runs YOLO/OpenCV detection, publishes `vision_msgs/Detection2DArray`, and serves structured detected objects.
3. `planning_scene_sync_node` converts detected cubes into MoveIt collision objects through `/apply_planning_scene`.
4. `langchain_reasoning_node` receives natural-language instructions and may only call predefined tools.
5. `action_adapter_node` validates all movement requests and either returns a dry-run status, calls the legacy action pipeline, or uses the Gazebo-only `moveit_sim` backend for simulated pick/place.

## Safety Boundary

The LLM never executes arbitrary code and never writes directly to robot topics. Its only route to robot motion is through validated ROS tools backed by services.

Simulation motion validation is handled separately by `moveit_motion_probe`, which sends a tiny reversible MoveIt-native goal through `/move_action` in Gazebo only. The cognitive pick/place demo uses `execution_backend:=moveit_sim`, which stays Gazebo-only and uses MoveIt `/move_action`, simulated gripper controllers, and IFRA LinkAttacher services instead of `/Robmove` and `/Move`.

## Primary Interfaces

- `/irb120pe/perception/detections_2d`
- `/irb120pe/perception/get_detected_objects`
- `/irb120pe/action/move_arm`
- `/irb120pe/action/pick_and_place`
- `/irb120pe/reasoning/execute_instruction`

## Validation Interfaces

- `/move_action`
- `/execute_trajectory`
- `/irb120_controller/follow_joint_trajectory`
- `/egp64_finger_left_controller/gripper_cmd`
- `/egp64_finger_right_controller/gripper_cmd`
- `/ATTACHLINK`
- `/DETACHLINK`
- `/controller_manager/list_controllers`
- `/get_planning_scene`

## Provider Modes

The default provider is `mock`, which makes the demo deterministic when API keys or local models are unavailable. OpenAI, OpenRouter, Ollama, and Hugging Face endpoint support can be enabled through parameters and environment variables. OpenRouter is the recommended path for a real-LLM demo because it exposes Anthropic, OpenAI, and Google models behind a single OpenAI-API-compatible client (`base_url=https://openrouter.ai/api/v1`).

## Goal-Oriented Arrangement Extension

The headline RI_26 mission adds a *goal-oriented arrangement* flow on top of the
single-pick `execute_instruction` service. The reasoning node accepts a
natural-language layout instruction (e.g. *"arrange the cubes in a line by
color from white to black to blue along Y at x=0.50, z=0.90, spacing=0.06"*),
queries perception, plans per-cube destinations, validates every destination
against the configured workspace, and drives the action adapter through a
sequence of pick-and-place calls. The LLM thus produces target coordinates the
deterministic code cannot derive on its own.

New runtime surface:

- Service `/irb120pe/reasoning/arrange_objects` (`irb120pe_cognitive_interfaces/srv/ArrangeObjects`) — entry point. Returns `plan_json` with the ordered `(object_id, target_pose)` plan.
- Topic `/irb120pe/reasoning/trace` (`std_msgs/String`, JSON) — one message per LLM tool call. Lets a demo viewer watch the AI's reasoning live (`ros2 topic echo`).
- Extended service `/irb120pe/action/pick_and_place` — `target_pose` is now optional. When populated, the action adapter treats it as the **cube destination**, validates it against `workspace_limits`, then converts the destination Z into a `tool0` waypoint using `place_z_offset_from_object` (default `0.18 m`). The direct `tool0` waypoint is independently validated against `tool0_workspace_limits`.

The destination-vs-`tool0` distinction matters: an earlier bug sent the cube
destination Z directly to MoveIt as a `tool0` goal, which produced
`NO_IK_SOLUTION` during the place phase. The fix is described in
`SESSION_LOG.md` and lives in `action_adapter_node.py` and `cognitive.yaml`.

## Verification Layers

Three layers of automated coverage:

1. **Pure-Python unit tests** (no ROS) — workspace bounds, slot resolution, arrangement planner output, free-target-pose validation. Run with `colcon test --packages-select irb120pe_cognitive`.
2. **ROS dry-run e2e** (`test/test_arrangement_e2e_dry_run.py`) — boots a `FakePerceptionNode` + `FakeActionAdapter` + the real `LangChainReasoningNode`, calls `/irb120pe/reasoning/arrange_objects`, asserts the returned plan and the per-step service calls.
3. **Live runtime e2e** (`arrangement_e2e_validator`) — spawns cubes in Gazebo, records `tool0_trajectory.csv` and `cube_trajectory.csv`, checks final cube poses vs the LLM-generated targets, writes a single `summary.txt` PASS/FAIL artifact for grading.
