# Cognitive Industrial Sorting Cobot Architecture

The system extends the original ABB IRB-120 pose-estimation demo with a tool-gated reasoning layer. The legacy deterministic scripts remain available, while the cognitive stack is launched through separate ROS 2 nodes.

## Runtime Flow

1. Gazebo and MoveIt 2 start the ABB IRB-120, Schunk gripper, camera, `/Move`, and `/Robmove` interfaces.
2. `perception_node` subscribes to the simulated camera, runs YOLO/OpenCV detection, publishes `vision_msgs/Detection2DArray`, and serves structured detected objects.
3. `planning_scene_sync_node` converts detected cubes into MoveIt collision objects through `/apply_planning_scene`.
4. `langchain_reasoning_node` receives natural-language instructions and may only call predefined tools.
5. `action_adapter_node` validates all movement requests and calls the existing robot action pipeline.

## Safety Boundary

The LLM never executes arbitrary code and never writes directly to robot topics. Its only route to robot motion is through validated ROS tools backed by services.

## Primary Interfaces

- `/irb120pe/perception/detections_2d`
- `/irb120pe/perception/get_detected_objects`
- `/irb120pe/action/move_arm`
- `/irb120pe/action/pick_and_place`
- `/irb120pe/reasoning/execute_instruction`

## Provider Modes

The default provider is `mock`, which makes the demo deterministic when API keys or local models are unavailable. OpenAI, Ollama, and Hugging Face endpoint support can be enabled through parameters and environment variables.
