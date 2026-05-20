# Gazebo Pick/Place With MoveIt Simulation Backend

This runbook validates the Gazebo-only cognitive pick/place path. It does not require `/Robmove` or `/Move`; those legacy interfaces can remain unavailable while `ros2srrc_execution` is skipped.

## Preconditions

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip ros2srrc_execution
source install/setup.bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

Expected current result: `28 tests, 0 errors, 0 failures`.

## Launch

Start the full stack with simulation execution enabled:

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py \
  dry_run:=false llm_provider:=mock execution_backend:=moveit_sim \
  spawn_timeout:=120.0 start_legacy_interfaces:=false rviz_file:=True
```

Wait for `/move_action`, `/ATTACHLINK`, `/DETACHLINK`, `/irb120pe/perception/get_detected_objects`, and `/irb120pe/reasoning/execute_instruction`.

## Manual Smoke Test

```bash
ros2 run irb120pe_cognitive gazebo_cube_helper spawn \
  --cube BlueCube --name BlueCube --x 0.55 --y 0.52 --z 0.88 --replace

ros2 service call /irb120pe/reasoning/execute_instruction \
  irb120pe_cognitive_interfaces/srv/ExecuteInstruction \
  "{instruction: 'Pick the blue cube and place it in the right container'}"
```

Expected status:

```text
moveit_sim pick-and-place completed for BlueCube.
```

Clean up:

```bash
ros2 run irb120pe_cognitive gazebo_cube_helper delete --name BlueCube
```

## Validated Evidence

The 2026-05-19 validation artifacts are under:

```text
docs/validation/2026-05-19-gazebo-pick-place/
```

Summary:

```text
Live cube validation: PASS
[PASS] spawn cube
[PASS] wait for detections
[PASS] verify Planning Scene add
[PASS] mock reasoning dry-run - moveit_sim pick-and-place completed for BlueCube.
[PASS] delete cube
[PASS] verify perception stale removal
[PASS] verify Planning Scene stale removal
```

The wording `mock reasoning dry-run` is from the reusable validator label; the recorded reasoning response confirms non-dry-run simulation execution through `execution_backend:=moveit_sim`.

## Notes

- The backend removes the target cube and known sticker marker collision objects from each MoveGroup goal so the simulated gripper can approach the perceived cube.
- IFRA LinkAttacher is still used, but only through Gazebo services.
- This path is for Gazebo simulation only.
