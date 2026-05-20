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

The default provider is `mock`, which makes the demo deterministic when API keys or local models are unavailable. OpenAI, Ollama, and Hugging Face endpoint support can be enabled through parameters and environment variables.
