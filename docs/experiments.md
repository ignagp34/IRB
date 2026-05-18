# Experiments

## Baseline

Goal: verify the original repository still builds and launches.

Commands:

```bash
cd ~/irb120_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch irb120pe_moveit2 moveit2.launch.py
```

Expected result: Gazebo, MoveIt 2, and the existing `/Move` and `/Robmove` interfaces start without cognitive nodes.

## Perception

Goal: verify YOLO detections are exposed as structured ROS data.

Commands:

```bash
ros2 launch irb120pe_cognitive cognitive_perception.launch.py
ros2 topic echo /irb120pe/perception/detections_2d --once
ros2 service call /irb120pe/perception/get_detected_objects irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
```

Expected result: detected cubes include object id, label, confidence, pose, and timestamp.

## Planning Scene

Goal: verify perceived cubes become MoveIt collision objects.

Commands:

```bash
ros2 launch irb120pe_cognitive planning_scene.launch.py
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1}}"
```

Expected result: cube collision objects are present while visible and removed after they disappear.

## Reasoning

Goal: verify the tool-gated reasoning loop in mock mode.

Commands:

```bash
ros2 launch irb120pe_cognitive reasoning.launch.py
ros2 service call /irb120pe/reasoning/execute_instruction irb120pe_cognitive_interfaces/srv/ExecuteInstruction "{instruction: 'Pick the blue cube and place it in the left container'}"
```

Expected result: the reasoning node calls perception, validates destination slots, and triggers pick-and-place through the adapter.

## 2026-05-14 WSL Step 0 Continuation

Environment:

```text
Windows workspace: C:\Users\Ignacio González\OneDrive\Escritorio\IRB
WSL distro: Ubuntu-22.04, Ubuntu 22.04.5 LTS
ROS workspace: /root/irb120_ws
ROS distro: Humble
```

Commands and results:

```powershell
wsl.exe -l -v
wsl.exe --install Ubuntu-22.04 --no-launch
```

Result: `Ubuntu-22.04` was installed. The first `wsl --install -d Ubuntu-22.04` attempt timed out without registering the distro; the distro-name form completed successfully.

```bash
apt update
apt install -y ros-humble-desktop ros-humble-moveit ros-humble-gazebo-ros-pkgs \
  ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-tf2-ros \
  python3-colcon-common-extensions python3-rosdep python3-vcstool
```

Result: initial ROS package downloads from `packages.ros.org` hit HTTP 403/reset errors, and HTTPS on `packages.ros.org` failed certificate hostname validation. The ROS 2 apt source was switched to the official UMD mirror:

```text
https://mirror.umd.edu/packages.ros.org/ros2/ubuntu
```

After switching mirrors, `apt --fix-broken install`, ROS Humble, MoveIt 2, Gazebo Classic, `ros2_control`, and `gazebo_ros2_control` installed successfully.

```bash
rsync -a --delete --exclude build --exclude install --exclude log --exclude .git \
  --exclude "__pycache__" --exclude "*.pyc" \
  "/mnt/c/Users/Ignacio González/OneDrive/Escritorio/IRB/" \
  /root/irb120_ws/src/irb120_PoseEstimation/
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Result: the repository's six packages built successfully:

```text
irb120pe_bringup
irb120pe_cognitive
irb120pe_cognitive_interfaces
irb120pe_detection
irb120pe_gazebo
irb120pe_moveit2
```

`rosdep` still reported unresolved non-apt/source keys from the upstream project: `ros2_linkpose`, `ros2srrc_data`, `linkattacher_msgs`, `irb120pe_data`, and `ament_pytest`.

```bash
cd /root/irb120_ws/src
git clone --branch humble https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
git clone --branch humble https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.git
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip ros2srrc_execution
```

Result: `linkattacher_msgs`, `ros2_linkattacher`, and `ros2srrc_data` built, allowing the cognitive action adapter to import its runtime interfaces. `ros2srrc_execution` was skipped because it requires `abb_robot_msgs`, which was not available from the configured Humble apt repositories.

```bash
source /root/irb120_ws/install/setup.bash
ros2 interface show irb120pe_cognitive_interfaces/msg/DetectedObject
ros2 interface show irb120pe_cognitive_interfaces/srv/GetDetectedObjects
ros2 interface show irb120pe_cognitive_interfaces/srv/MoveArm
ros2 interface show irb120pe_cognitive_interfaces/srv/PickAndPlace
ros2 interface show irb120pe_cognitive_interfaces/srv/ExecuteInstruction
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

Result: generated interfaces are visible. Tests passed with `8 tests, 0 errors, 0 failures, 0 skipped`.

```bash
ros2 launch irb120pe_cognitive cognitive_perception.launch.py
ros2 service call /irb120pe/perception/get_detected_objects \
  irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
```

Result: perception node starts and the service returns a structured empty object list. YOLO inference is blocked because `ultralytics` is not installed. A `pip install ultralytics` attempt exceeded 15 minutes and was stopped.

```bash
ros2 launch irb120pe_cognitive planning_scene.launch.py
```

Result: planning-scene sync node starts against the perception service. No collision objects are applied while perception has zero detections.

```bash
ros2 launch irb120pe_cognitive reasoning.launch.py dry_run:=true llm_provider:=mock
ros2 service call /irb120pe/reasoning/execute_instruction \
  irb120pe_cognitive_interfaces/srv/ExecuteInstruction \
  "{instruction: 'Pick the blue cube and place it in the left container'}"
```

Result: reasoning and action adapter launch successfully in dry-run/mock mode. With no detections, the reasoning service safely rejects the command with `No detected object matches the requested source.`

```bash
ros2 service call /irb120pe/action/pick_and_place \
  irb120pe_cognitive_interfaces/srv/PickAndPlace \
  "{object_id: 'BlueCube_1', source_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.2, y: 0.2, z: 1.0}, orientation: {w: 1.0}}}, target_slot: 'slot_c'}"
```

Result: direct action-adapter dry-run validation succeeds: `dry_run: would pick BlueCube_1 and place into slot_c.`

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py dry_run:=true llm_provider:=mock
```

Result: full-stack launch starts Gazebo, robot state publisher, and all cognitive nodes. Remaining blocker: in this WSL session Gazebo is slow to expose `/spawn_entity`; `spawn_entity.py` times out after 30 seconds, so the robot entity is not inserted and `/controller_manager/list_controllers` never becomes available.

Fixes applied:

- Installed Ubuntu-22.04 WSL and ROS Humble dependencies in the WSL filesystem.
- Switched ROS apt source to the UMD ROS mirror after official host download failures.
- Added IFRA source dependencies in WSL for `linkattacher_msgs` and `ros2srrc_data`.
- Installed `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-controller-manager`, and `ros-humble-gazebo-ros2-control`.
- Updated cognitive node shutdown handling so launch interrupts do not raise `rclpy.shutdown()` tracebacks.

Remaining blockers:

- `ultralytics` is still missing, so YOLO perception does not run yet.
- `cognitive_demo.launch.py` is blocked by Gazebo `/spawn_entity` startup timing in WSL.
- `ros2srrc_execution` is skipped unless `abb_robot_msgs` is provided.
- Live Planning Scene object add/remove behavior still needs real detections.
- Real `/Robmove`, `/Move`, and LinkAttacher execution remains untested; dry-run validation passed.

## 2026-05-17 WSL Continuation

Environment:

```text
Windows workspace: C:\Users\Ignacio González\OneDrive\Escritorio\IRB
WSL distro: Ubuntu-22.04
ROS workspace: /root/irb120_ws
ROS distro: Humble
Validation mode: dry_run:=true, llm_provider:=mock
```

Commands and results:

```powershell
wsl -d Ubuntu-22.04 --user root -- bash -lc "source /opt/ros/humble/setup.bash && ros2 --help >/dev/null && echo ok"
```

Result: ROS 2 in Ubuntu-22.04 WSL still works and printed `ok`. WSL also printed `Failed to start the systemd user session for 'root'`; this did not block ROS commands.

```powershell
wsl -d Ubuntu-22.04 --user root -- bash -lc "rsync -a --delete --exclude build --exclude install --exclude log --exclude .git --exclude '__pycache__' --exclude '*.pyc' '/mnt/c/Users/Ignacio González/OneDrive/Escritorio/IRB/' /root/irb120_ws/src/irb120_PoseEstimation/"
```

Result: Windows repo changes were synced back into the WSL source workspace.

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip ros2srrc_execution
source install/setup.bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

Result: build passed with `31 packages finished` while continuing to skip `ros2srrc_execution`. Cognitive tests passed: `8 tests, 0 errors, 0 failures, 0 skipped`.

```bash
python3 -m pip install --no-cache-dir --progress-bar off --timeout 60 torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install --no-cache-dir --progress-bar off --timeout 60 --no-deps ultralytics
```

Result: controlled install succeeded. Installed versions verified by import:

```text
ultralytics 8.4.51
torch 2.12.0+cpu
torchvision 0.27.0+cpu
```

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
model_path = Path(get_package_share_directory('irb120pe_detection')) / 'yolov8' / 'cubeDETECTION_Gz.pt'
print(model_path)
print('exists', model_path.exists())
model = YOLO(str(model_path))
print('names', model.names)
PY
```

Result: YOLO model loads from the installed ROS package share path:

```text
/root/irb120_ws/install/irb120pe_detection/share/irb120pe_detection/yolov8/cubeDETECTION_Gz.pt
exists True
names {0: 'black', 1: 'blue', 2: 'cube', 3: 'sticker', 4: 'white'}
```

```bash
ros2 launch irb120pe_cognitive cognitive_perception.launch.py
ros2 service call /irb120pe/perception/get_detected_objects \
  irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
```

Result: perception node starts without the previous `YOLO model unavailable` warning. With no image/cube stimulus in this isolated check, the service returned a structured empty list: `0 detected object(s) available.`

```bash
timeout -s INT 60s ros2 launch irb120pe_cognitive cognitive_demo.launch.py \
  dry_run:=true llm_provider:=mock rviz_file:=True
```

Result: staged cognitive demo now gets past the previous Gazebo spawn and controller blockers:

- `spawn_entity.py` waits with `timeout = 120` and successfully spawns `irb120`.
- `gazebo_ros2_control` parses the robot description, loads `irb120egp64_controller.yaml`, and starts `controller_manager`.
- `joint_state_broadcaster`, `irb120_controller`, `egp64_finger_left_controller`, and `egp64_finger_right_controller` all load, configure, and activate.
- Cognitive perception, planning-scene sync, action adapter, and mock reasoning nodes start.
- MoveIt `move_group` starts and prints `You can start planning now!`

The final wrapper command included a bad shell `test ${PIPESTATUS...}` expression after logging, so the command itself ended with `bash: test: too many arguments`. The launch log still confirms the staged stack reached MoveIt readiness before the intentional timeout interrupted it.

Fixes applied:

- Installed `irb120pe_detection/yolov8/` into the package share tree so the trained `.pt` model is available after `colcon build`.
- Changed the cognitive perception node's default model lookup to resolve `irb120pe_detection/yolov8/cubeDETECTION_Gz.pt` through `ament_index_python`, with a WSL workspace fallback.
- Added `spawn_timeout` launch arguments to `irb120pe_moveit2` and `irb120pe_gazebo`; default is `120.0` seconds.
- Added `start_legacy_interfaces` to `irb120pe_moveit2`; it defaults to `true` there to preserve upstream behavior.
- Changed `cognitive_demo.launch.py` defaults to `dry_run:=true` and `start_legacy_interfaces:=false` so the staged cognitive demo can run while `ros2srrc_execution` is still skipped.
- Stripped generated xacro XML comments before publishing `robot_description`; this fixed the `gazebo_ros2_control` parameter parser error caused by XML comments in the robot description parameter.
- Hardened cognitive node teardown so forced SIGINT validation does not emit Python `KeyboardInterrupt` tracebacks during `destroy_node()`.

Remaining blockers and cautions:

- `ros2srrc_execution` is still skipped until `abb_robot_msgs` is provided; therefore real `/Move`, `/Robmove`, and sequence interface execution remains unvalidated.
- `cognitive_demo.launch.py` reaches staged dry-run/mock readiness, but a long-running manual session should still be used for interactive demos rather than the bounded `timeout` command.
- The forced timeout still reports SIGINT/SIGTERM exit codes for ROS/Gazebo processes; Gazebo may need escalation after controller shutdown in WSL. No stale Gazebo or ROS launch processes remained after validation.
- Live Planning Scene add/remove behavior still needs real or simulated cube detections.
- Real robot movement and LinkAttacher execution remain untested; continue using `dry_run:=true` and `llm_provider:=mock` until the ROS tool path is validated end to end.

## 2026-05-17 Staged Demo Evidence Capture

Validation mode remained `dry_run:=true` and `llm_provider:=mock`. No real `/Move`, `/Robmove`, or LinkAttacher execution was attempted.

Evidence artifacts were saved under:

```text
docs/validation/2026-05-17-cognitive-demo/
```

Key files:

- `cognitive_demo_launch.log`: full bounded launch log.
- `ros_checks.txt`: live ROS node, service, topic, controller, perception, reasoning, and Planning Scene probes.
- `gazebo_desktop.png`: Windows desktop screenshot showing the Gazebo WSLg window with the spawned IRB-120 cell.
- `windows_before_screenshot.txt`: window list captured immediately before the screenshot; includes `Gazebo (Ubuntu-22.04)`.
- `post_shutdown_processes.txt`: post-cleanup process check.

Commands and results:

```powershell
wsl -d Ubuntu-22.04 --user root -- bash -lc "rsync -a --delete --exclude build --exclude install --exclude log --exclude .git --exclude '__pycache__' --exclude '*.pyc' '/mnt/c/Users/Ignacio González/OneDrive/Escritorio/IRB/' /root/irb120_ws/src/irb120_PoseEstimation/ && cd /root/irb120_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-skip ros2srrc_execution"
```

Result: build passed again with `31 packages finished`, continuing to skip only `ros2srrc_execution`.

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

Result: cognitive tests passed again: `8 tests, 0 errors, 0 failures, 0 skipped`.

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py \
  dry_run:=true llm_provider:=mock spawn_timeout:=120.0
```

Result: bounded launch reached staged readiness in 33 seconds. The launch log confirms:

- `spawn_entity.py` successfully spawned entity `irb120`.
- `gazebo_ros2_control` loaded `controller_manager`.
- `joint_state_broadcaster`, `irb120_controller`, `egp64_finger_left_controller`, and `egp64_finger_right_controller` configured and activated.
- Cognitive perception, Planning Scene sync, action adapter, and mock reasoning processes started.
- MoveIt `move_group` printed `You can start planning now!`.

Live ROS probes while the launch was up confirmed:

```text
joint_state_broadcaster       active
irb120_controller             active
egp64_finger_left_controller  active
egp64_finger_right_controller active
```

`/joint_states` produced a live sample for the IRB-120, gripper, base, TCP, and end-effector joints.

```bash
ros2 service call /irb120pe/perception/get_detected_objects \
  irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
```

Result: perception returned a structured empty list because no image/cube stimulus was supplied:

```text
objects=[]
status='0 detected object(s) available.'
```

```bash
ros2 service call /irb120pe/reasoning/execute_instruction \
  irb120pe_cognitive_interfaces/srv/ExecuteInstruction \
  "{instruction: 'Pick the blue cube and place it in the left container'}"
```

Result: mock reasoning safely rejected the instruction without triggering motion:

```text
success=False
status='No detected object matches the requested source.'
tool_trace='[]'
```

Planning Scene probe result:

```text
collision_objects=0 ids=[]
```

This matches the no-stimulus perception state. Live Planning Scene add/remove behavior still needs a real or simulated cube detection stimulus.

Screenshot result: `gazebo_desktop.png` captured the Gazebo WSLg window and shows the spawned IRB-120 robot cell. The window list recorded `msrdc Gazebo (Ubuntu-22.04)` immediately before capture.

Cleanup result: a surviving `gzserver` process from the interrupted validation was explicitly stopped after screenshot capture. A final process check found no remaining `ros2 launch`, `gzserver`, `gzclient`, or `gazebo` process other than the check command itself.

## 2026-05-17 Cube Stimulus Validation

Validation mode remained `dry_run:=true` and `llm_provider:=mock`. `ros2srrc_execution` remained skipped because `abb_robot_msgs` is still unavailable. No real `/Move`, `/Robmove`, or LinkAttacher execution was attempted.

Evidence artifacts were saved under:

```text
docs/validation/2026-05-17-cube-stimulus/
```

Build and test commands:

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip ros2srrc_execution
source install/setup.bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

Result: build passed with `31 packages finished`, continuing to skip `ros2srrc_execution`. Cognitive tests passed: `8 tests, 0 errors, 0 failures, 0 skipped`.

Live Gazebo stimulus attempt:

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py \
  dry_run:=true llm_provider:=mock spawn_timeout:=120.0
```

Result: the staged stack reached readiness and Gazebo accepted a non-motion `BlueCube` stimulus at `x=0.55, y=0.52, z=0.88`:

```text
spawn True SpawnEntity: Successfully spawned entity [BlueCube]
delete True Successfully deleted entity [BlueCube]
```

The first positive YOLO frame exposed a compatibility bug in the cognitive perception node: Humble's `vision_msgs/BoundingBox2D.center` is a `Pose2D`, so the node must assign `bbox.center.position.x/y`. This was fixed in `irb120pe_cognitive/perception_node.py` and the workspace was rebuilt.

Controlled image-stimulus fallback:

```bash
ros2 launch irb120pe_moveit2 moveit2.launch.py \
  spawn_timeout:=120.0 start_legacy_interfaces:=false
ros2 run irb120pe_cognitive perception_node --ros-args \
  --params-file install/irb120pe_cognitive/share/irb120pe_cognitive/config/cognitive.yaml \
  -p camera_topic:=/cube_stimulus/image_raw \
  -p camera_info_topic:=/cube_stimulus/camera_info \
  -p detection_period_sec:=3.0
ros2 run irb120pe_cognitive planning_scene_sync_node --ros-args \
  --params-file install/irb120pe_cognitive/share/irb120pe_cognitive/config/cognitive.yaml
ros2 run irb120pe_cognitive action_adapter_node --ros-args \
  --params-file install/irb120pe_cognitive/share/irb120pe_cognitive/config/cognitive.yaml \
  -p dry_run:=true
ros2 run irb120pe_cognitive langchain_reasoning_node --ros-args \
  --params-file install/irb120pe_cognitive/share/irb120pe_cognitive/config/cognitive.yaml \
  -p llm_provider:=mock
```

The image publisher streamed `irb120pe_detection/samples/Blue_1.jpg`, then a blank frame of the same size.

Perception result:

```text
objects=[
  object_id='blue_1', label='blue', confidence=0.904931366443634,
  object_id='sticker_1', label='sticker', confidence=0.867638111114502
]
status='2 detected object(s) available.'
```

Planning Scene add result:

```text
world.collision_objects=[id='blue_1', id='sticker_1']
```

Mock reasoning result:

```text
success=True
status='dry_run: would pick blue_1 and place into slot_c.'
```

Planning Scene removal result after blank stimulus:

```text
objects=[]
status='0 detected object(s) available.'
world.collision_objects=[]
```

Remaining notes:

- The cognitive ROS path is validated end-to-end with controlled image stimulus: perception, structured objects, Planning Scene add/remove, mock reasoning, and validated dry-run pick/place.
- The live Gazebo cube path still needs a follow-up run after the bounding-box fix, because the original live run hit the perception crash before it could prove non-empty detections from `/camera/image_raw`.
- Running perception at `detection_period_sec:=3.0` made service probes and mock reasoning reliable on the CPU-only WSL validation environment.
