# 15-30 Minute Demo Script

## 1. Team Introduction

Each team member introduces their role: simulation, perception, planning, reasoning, integration, or testing.

## 2. Architecture Overview

Show the ROS graph and explain the loop:

Natural language instruction -> LangChain reasoning -> perception query -> slot validation -> action adapter -> dry-run status or Gazebo-only MoveIt simulation execution.

The normal presentation should start in `dry_run:=true` and `llm_provider:=mock`. For the master's Gazebo demo, simulation execution is enabled with `execution_backend:=moveit_sim`; it uses MoveIt `/move_action`, the gripper controllers, and IFRA LinkAttacher in Gazebo. The legacy `/Robmove` and `/Move` interfaces are not required.

## 3. Baseline Gazebo And MoveIt

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch irb120pe_moveit2 moveit2.launch.py \
  spawn_timeout:=120.0 start_legacy_interfaces:=false rviz_file:=True
```

Show the ABB IRB-120, gripper, camera, and MoveIt planning environment.

## 4. Perception Demo

```bash
ros2 launch irb120pe_cognitive cognitive_perception.launch.py
ros2 service call /irb120pe/perception/get_detected_objects irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
```

Show detected cube labels, confidence, and estimated pose.

## 5. Planning Scene Demo

```bash
ros2 launch irb120pe_cognitive planning_scene.launch.py
```

Show collision objects in RViz/MoveIt and explain add, update, and removal.

## 6. LangChain Reasoning Demo

Start in mock mode:

```bash
ros2 launch irb120pe_cognitive reasoning.launch.py \
  dry_run:=true llm_provider:=mock
```

Run:

```bash
ros2 service call /irb120pe/reasoning/execute_instruction irb120pe_cognitive_interfaces/srv/ExecuteInstruction "{instruction: 'Pick the blue cube and place it in the left container'}"
```

Show the returned tool trace.

## 7. MoveIt Simulation Motion Probe

Use this only in Gazebo simulation. It validates a tiny reversible MoveIt-native motion through `/move_action`; it does not use `/Robmove`, `/Move`, LinkAttacher, gripper commands, pick/place, or cognitive non-dry-run execution.

First capture readiness:

```bash
ros2 run irb120pe_cognitive motion_readiness_validator \
  --output-dir /root/irb120_ws/src/irb120_PoseEstimation/docs/validation/demo-motion-probe/readiness
```

Then execute the reversible simulation probe:

```bash
ros2 run irb120pe_cognitive moveit_motion_probe \
  --execute --return-to-start \
  --output-dir /root/irb120_ws/src/irb120_PoseEstimation/docs/validation/demo-motion-probe/probe
```

Expected validated behavior: `joint_6` moves about `0.02` rad and returns close to the original joint state.

For the full rerun checklist and expected evidence files, see `docs/moveit_simulation_motion_probe.md`.

## 8. Full Cognitive Dry-Run Demo

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py \
  dry_run:=true llm_provider:=mock execution_backend:=moveit_sim spawn_timeout:=120.0
```

Example commands:

- `Pick the blue cube and place it in the left container`
- `Classify the black cube into slot B`
- `Sort all visible cubes by color`

## 9. Gazebo Pick/Place Execution

Use this only in Gazebo simulation:

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py \
  dry_run:=false llm_provider:=mock execution_backend:=moveit_sim \
  spawn_timeout:=120.0 start_legacy_interfaces:=false rviz_file:=True
```

Then spawn a visible cube and call the reasoning service:

```bash
ros2 run irb120pe_cognitive gazebo_cube_helper spawn \
  --cube BlueCube --name BlueCube --x 0.55 --y 0.52 --z 0.88 --replace

ros2 service call /irb120pe/reasoning/execute_instruction \
  irb120pe_cognitive_interfaces/srv/ExecuteInstruction \
  "{instruction: 'Pick the blue cube and place it in the right container'}"
```

Expected validated status: `moveit_sim pick-and-place completed for BlueCube.`

For the full rerun checklist and evidence files, see `docs/gazebo_pick_place_moveit_sim.md`.

## 10. Limitations And Conclusions

Discuss YOLO class limitations, workspace bounds, model/provider availability, the validated MoveIt-native simulation probe, and the validated Gazebo-only cognitive pick/place path. The old `/Robmove` and `/Move` blocker is no longer a blocker for the master's demo because the project uses the MoveIt-native simulation backend.
