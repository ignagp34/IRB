# Module Notes

## Perception

The perception module owns camera subscription, YOLO inference, 2D detection publishing, and the structured object cache. It does not move the robot and does not modify the Planning Scene.

## Planning Scene

The planning-scene synchronizer reads perceived objects and applies collision objects to MoveIt 2. Current detections are added or updated; stale detections are removed so old objects do not block future plans.

## Action Adapter

The action adapter is the only cognitive-stack node that commands robot motion. It validates numeric target poses, workspace limits, motion type, gripper calls, and destination slots before calling `/Robmove`, `/Move`, `/ATTACHLINK`, and `/DETACHLINK`.

## Reasoning

The reasoning node exposes a natural-language instruction service and implements strict LangChain tools:

- `get_detected_objects`: read-only perception query.
- `get_available_slots`: read-only slot query.
- `move_arm_tool`: explicit validated coordinate motion.
- `pick_and_place_tool`: validated full pick-and-place operation.

## Legacy Scripts

The original `main.py`, `main_Gz.py`, and `main_GzSimplified.py` remain as deterministic reference demos. They are useful for validating the base simulation before using the cognitive loop.
