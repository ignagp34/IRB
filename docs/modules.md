# Module Notes

## Perception

The perception module owns camera subscription, YOLO inference, 2D detection publishing, and the structured object cache. It does not move the robot and does not modify the Planning Scene.

## Planning Scene

The planning-scene synchronizer reads perceived objects and applies collision objects to MoveIt 2. Current detections are added or updated; stale detections are removed so old objects do not block future plans.

## Action Adapter

The action adapter is the only cognitive-stack node that commands robot motion. It validates numeric target poses, workspace limits, motion type, gripper calls, execution backend, and destination slots before execution.

Backends:

- `legacy`: calls `/Robmove`, `/Move`, `/ATTACHLINK`, and `/DETACHLINK` when the legacy execution dependency chain is available.
- `moveit_sim`: Gazebo-only backend that calls MoveIt `/move_action`, the simulated gripper `gripper_cmd` actions, and IFRA `/ATTACHLINK`/`/DETACHLINK`. It removes the target cube and marker collision IDs from each MoveGroup goal so the simulated gripper can approach the perceived cube.

Keep `dry_run:=true` for normal cognitive demonstrations. Use `dry_run:=false execution_backend:=moveit_sim` only for controlled Gazebo simulation validation.

## Reasoning

The reasoning node exposes a natural-language instruction service and implements strict LangChain tools:

- `get_detected_objects`: read-only perception query.
- `get_available_slots`: read-only slot query.
- `move_arm_tool`: explicit validated coordinate motion.
- `pick_and_place_tool`: validated full pick-and-place operation.

## Legacy Scripts

The original `main.py`, `main_Gz.py`, and `main_GzSimplified.py` remain as deterministic reference demos. They are useful for validating the base simulation before using the cognitive loop.

## Motion Validation Tools

`motion_readiness_validator` captures non-moving evidence for controller, joint-state, MoveIt action, legacy action, and Planning Scene readiness.

`moveit_motion_probe` validates the simulation-only MoveIt path with a tiny reversible `/move_action` goal. It is a diagnostic tool, not a cognitive action-adapter replacement, and it does not call `/Robmove`, `/Move`, LinkAttacher, gripper commands, or pick/place.

The Gazebo pick/place validation path uses `action_adapter_node` with `execution_backend:=moveit_sim`; see `docs/gazebo_pick_place_moveit_sim.md`.
