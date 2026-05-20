# MoveIt Simulation Motion Probe

This runbook validates a small reversible arm motion in Gazebo through MoveIt-native actions only. It is intended for WSL 2 Ubuntu 22.04 with ROS 2 Humble, MoveIt 2, and Gazebo Classic.

The probe is simulation-only. It does not use `/Robmove`, `/Move`, LinkAttacher, gripper commands, pick/place commands, or the cognitive action adapter non-dry-run path.

## Preconditions

- Work inside the WSL Linux filesystem, for example `/root/irb120_ws`.
- Build the workspace while continuing to skip `ros2srrc_execution` until `abb_robot_msgs` is available.
- Source ROS 2 and the workspace install before launching or validating.

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip ros2srrc_execution
source install/setup.bash
```

## Launch The Simulation Stack

Start Gazebo and MoveIt with legacy interfaces disabled:

```bash
ros2 launch irb120pe_moveit2 moveit2.launch.py \
  spawn_timeout:=120.0 start_legacy_interfaces:=false rviz_file:=True
```

Wait until `move_group` reports that planning is ready.

## Capture Non-Moving Readiness Evidence

Run the readiness validator before any motion probe:

```bash
ros2 run irb120pe_cognitive motion_readiness_validator \
  --output-dir /root/irb120_ws/src/irb120_PoseEstimation/docs/validation/YYYY-MM-DD-moveit-motion-probe/readiness
```

Expected current result while `ros2srrc_execution` is skipped:

```text
Motion readiness validation: BLOCKED
[PASS] controllers active
[PASS] /joint_states publishes
[PASS] MoveIt/controller action servers ready
[BLOCKED] existing action adapter execution path
[PASS] MoveIt planning scene service ready
```

`BLOCKED` is acceptable here only because `/Robmove` and `/Move` are unavailable. The MoveIt-native simulation probe can still proceed when `/move_action`, `/execute_trajectory`, and `/irb120_controller/follow_joint_trajectory` are ready.

## Optional Plan-Only Probe

The default probe mode plans a tiny joint-space move but sends no trajectory:

```bash
ros2 run irb120pe_cognitive moveit_motion_probe \
  --output-dir /root/irb120_ws/src/irb120_PoseEstimation/docs/validation/YYYY-MM-DD-moveit-motion-probe/plan_only
```

This is useful before an execution run if the launch state changed or the planner configuration needs a quick check.

## Execute The Reversible Simulation Move

Run with explicit execution enabled:

```bash
ros2 run irb120pe_cognitive moveit_motion_probe \
  --execute --return-to-start \
  --output-dir /root/irb120_ws/src/irb120_PoseEstimation/docs/validation/YYYY-MM-DD-moveit-motion-probe/probe
```

Expected result:

```text
MoveIt motion probe: PASS
[PASS] controllers active
[PASS] MoveIt/controller action servers ready
[PASS] legacy action servers unused
[PASS] /joint_states baseline captured
[PASS] tiny reversible target selected
[PASS] MoveGroup outbound goal
[PASS] outbound joint delta observed
[PASS] MoveGroup return goal
[PASS] final state near baseline
```

By default the probe moves `joint_6` by `0.02` radians and returns to the original joint state. Keep the default small delta unless there is a specific reason to change it.

## Evidence To Keep

The output directory should contain:

- `summary.txt`
- `controllers.txt`
- `action_server_waits.json`
- `pre_joint_states.txt`
- `target_joint_state.json`
- `move_action_result.json`
- `after_joint_states.txt`
- `return_action_result.json`
- `final_joint_states.txt`
- `delta_report.json`

For the 2026-05-19 validated run, `joint_6` moved `0.01999663162515919` rad and returned with `max_abs_final_delta=0.00002768264625974126` rad.

## Stop Conditions

Stop and investigate before executing the probe if any of these happen:

- `/move_action`, `/execute_trajectory`, or `/irb120_controller/follow_joint_trajectory` is unavailable.
- Required controllers are inactive.
- `/joint_states` does not publish all arm joints.
- The selected target cannot fit inside the configured joint limits.
- The environment is connected to a real robot or the operator is unsure whether it is simulation-only.
