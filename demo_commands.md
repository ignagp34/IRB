# 15-30 Minute Demo Script

## 1. Team Introduction

Each team member introduces their role: simulation, perception, planning, reasoning, integration, or testing.

## 2. Architecture Overview

Show the ROS graph and explain the loop:

Natural language instruction -> LangChain reasoning -> perception query -> slot validation -> action adapter -> MoveIt/Gazebo execution -> status report.

## 3. Baseline Gazebo And MoveIt

```bash
ros2 launch irb120pe_moveit2 moveit2.launch.py
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
ros2 launch irb120pe_cognitive reasoning.launch.py
```

Run:

```bash
ros2 service call /irb120pe/reasoning/execute_instruction irb120pe_cognitive_interfaces/srv/ExecuteInstruction "{instruction: 'Pick the blue cube and place it in the left container'}"
```

Show the returned tool trace.

## 7. Full Demo

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py
```

Example commands:

- `Pick the blue cube and place it in the left container`
- `Classify the black cube into slot B`
- `Sort all visible cubes by color`

## 8. Limitations And Conclusions

Discuss YOLO class limitations, simulated grasping through LinkAttacher, workspace bounds, model/provider availability, and next steps toward real-cell deployment.
