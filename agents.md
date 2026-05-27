# AGENTS.md

Project guide for AI agents working on the ABB IRB-120 Cognitive Robot Cell,
the RI_26 master's final project built on IFRA-Cranfield's
`irb120_PoseEstimation` repository.

Keep changes aligned with the cognitive demo: read the existing ROS package
boundaries first, preserve safety validation, and verify behavior in WSL when
motion or perception is affected.

---

## Mission

This is a ROS 2 Humble sorting cell where a LangChain reasoning node reads
YOLOv8 detections, interprets a natural-language arrangement goal, emits
per-cube target poses, and executes pick-and-place through MoveIt 2 in
Gazebo Classic. The headline flow is goal-oriented arrangement of white,
black, and blue cubes, not only the legacy deterministic pick-and-place demo.

---

## Environment

- Use Ubuntu 22.04 on WSL 2 for ROS 2 Humble, MoveIt 2, and Gazebo Classic.
- Keep ROS builds in the Linux filesystem, normally `~/irb120_ws`.
- Source `/opt/ros/humble/setup.bash` before building and
  `~/irb120_ws/install/setup.bash` before launching or testing.
- Pin perception dependencies together:

```bash
pip install --user "numpy<2" "opencv-python<4.12" ultralytics langchain langchain-openai
pip install --user --upgrade "pytest>=7.4" pytest-asyncio
```

ROS Humble `cv_bridge` is incompatible with NumPy 2.x in this environment.

---

## Repository Layout

| Package | Purpose |
|---|---|
| `irb120pe_gazebo` | Gazebo world, cube models, robot simulation and controllers |
| `irb120pe_moveit2` | MoveIt configuration and launch integration |
| `irb120pe_bringup` | Real-robot bringup; not used by the cognitive Gazebo demo |
| `irb120pe_detection` | Legacy IFRA detection/scripts and YOLO weights |
| `irb120pe_cognitive_interfaces` | Cognitive msgs/srvs, including `ArrangeObjects` and `PickAndPlace` |
| `irb120pe_cognitive` | Perception, planning-scene sync, action adapter, reasoning, demos and validators |

---

## Architecture

```text
perception_node -- Detection2DArray / GetDetectedObjects --> reasoning tools
       |
       +-- structured detections --> planning_scene_sync_node --> MoveIt Planning Scene

langchain_reasoning_node -- MoveArm / PickAndPlace / ArrangeObjects --> action_adapter_node
                                                                    |-- /move_action
                                                                    |-- gripper actions
                                                                    +-- /ATTACHLINK, /DETACHLINK
```

Agent rules:

- Preserve node isolation and ROS communication; do not fold the cognitive
  stack into a monolithic script.
- Validate every requested motion target through
  `validation.validate_coordinates` before execution.
- Keep tool schemas and descriptions synchronized with action validation and
  refusal behavior.
- Publish reasoning-tool activity to `/irb120pe/reasoning/trace` when adding
  user-visible reasoning actions.

---

## Entry Points

| Command | Purpose |
|---|---|
| `ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py dry_run:=false llm_provider:=mock execution_backend:=moveit_sim` | Full arrangement stack and primary demo baseline |
| `ros2 launch irb120pe_cognitive cognitive_demo.launch.py` | Older single-cube cognitive demo |
| `ros2 run irb120pe_cognitive arrangement_demo` | Spawn three cubes and call the arrangement service |
| `ros2 run irb120pe_cognitive arrangement_e2e_validator --output-dir ./evidence/arrangement` | Live evidence collection and `summary.txt` |
| `ros2 run irb120pe_cognitive gazebo_cube_helper spawn --cube BlueCube ...` | Manual spawn probe |
| `ros2 topic echo /irb120pe/reasoning/trace` | Observe reasoning actions during a demo |

Start validation with `llm_provider:=mock`; add OpenAI/OpenRouter/Ollama or
Hugging Face only after the ROS tool path is verified.

---

## Verified Runtime Facts

- Planning frame is `world`; the MoveIt end-effector planning link is `tool0`.
- The IRB-120 is mounted on a pedestal in the world: `base_link` is near
  `z=0.861`, not at floor height. Do not subtract that offset from world poses.
- Gazebo publishes a valid TF for `camera_link`, not the configured legacy
  name `camera`; perception must use `camera_frame: camera_link`.
- Camera subscriptions use sensor-data QoS, and image rays must be converted
  to the URDF `camera_link` convention before world projection.
- Pick targets are based on detected object Z plus calibrated
  `tool0`-to-gripper offsets:
  `pick_z_offset_from_object: 0.17` and
  `pick_approach_offset_from_object: 0.20`.
- `place_approach_offset_z: 0.10` is the current MoveIt simulation setting.

Do not claim full live arrangement success without a newly produced passing
`summary.txt`. Perception, reasoning, and pick/lift execution have been
observed live; place IK and the IFRA LinkAttacher plugin have shown
intermittent failures under repeated runs.

---

## Verification Strategy

1. Pure-Python tests validate parsing, workspace bounds and target selection.
2. ROS dry-run tests validate service contracts without commanding Gazebo.
3. Live runtime validation launches the full stack, records trajectories and
   writes `evidence/arrangement/summary.txt`.

For code touching motion, perception, collision objects or launch behavior,
run all applicable layers. Before starting or stopping a live stack, check
for existing ROS/Gazebo processes so another experiment is not interrupted.

Build and test in WSL:

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select irb120pe_cognitive_interfaces
source install/setup.bash
colcon build --symlink-install --packages-select irb120pe_cognitive
source install/setup.bash
colcon test --packages-select irb120pe_cognitive_interfaces irb120pe_cognitive
colcon test-result --verbose
```

When `.srv` or `.msg` interfaces change, always rebuild
`irb120pe_cognitive_interfaces` before consumers.

---

## LLM Providers

| Provider | Required configuration | Notes |
|---|---|---|
| `mock` | None | Deterministic test/demo baseline |
| `openai` | `OPENAI_API_KEY` | Uses `langchain-openai` |
| `openrouter` | `OPENROUTER_API_KEY` | Optional live model path via the OpenAI-compatible endpoint |
| `ollama` | Local Ollama server | Local-provider option |
| `huggingface` | `HUGGINGFACE_ENDPOINT_URL` | Endpoint-backed option |

Do not commit API keys, tokens or captured environments containing secrets.

---

## Engineering Conventions

- Use Python 3.10-compatible syntax.
- Read workspace bounds and slot aliases from
  `irb120pe_cognitive/config/cognitive.yaml` through existing helpers; do not
  introduce duplicate bounds in node code.
- Keep the `moveit_sim` backend as the Gazebo cognitive-demo execution path.
  Legacy `/Robmove` and `/Move` behavior is reference compatibility, not the
  preferred new implementation path.
- Use IFRA `/ATTACHLINK` and `/DETACHLINK` where the existing simulation path
  expects attachment; document or isolate experiments that replace it.
- Keep legacy deterministic demos as references while preserving the
  arrangement mission as the main cognitive demonstration.

---

## Read First

- `README.md` for installation and overview.
- `STEP_BY_STEP.md` for the current WSL runbook.
- `SESSION_LOG.md` for the most recent live-debug handoff and known failures.
- `demo_commands.md` for demo/video execution.
- `irb120pe_cognitive/config/cognitive.yaml` before changing runtime behavior.
