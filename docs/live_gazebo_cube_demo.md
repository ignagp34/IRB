# Live Gazebo Cube Demo

This sequence reruns the validated live camera path in WSL with dry-run actions and mock reasoning only.

## Sync And Build

Run from Windows PowerShell:

```powershell
wsl -d Ubuntu-22.04 --user root -- bash -lc "rsync -a --delete --exclude build --exclude install --exclude log --exclude .git --exclude '__pycache__' --exclude '*.pyc' '/mnt/c/Users/Ignacio González/OneDrive/Escritorio/IRB/' /root/irb120_ws/src/irb120_PoseEstimation/"
```

Run inside Ubuntu-22.04 WSL:

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip ros2srrc_execution
source install/setup.bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

## Launch The Stack

Terminal 1:

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch irb120pe_moveit2 moveit2.launch.py \
  spawn_timeout:=120.0 start_legacy_interfaces:=false
```

Terminal 2:

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irb120pe_cognitive perception_node --ros-args \
  --params-file install/irb120pe_cognitive/share/irb120pe_cognitive/config/cognitive.yaml \
  -p detection_period_sec:=3.0
```

Terminal 3:

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irb120pe_cognitive planning_scene_sync_node --ros-args \
  --params-file install/irb120pe_cognitive/share/irb120pe_cognitive/config/cognitive.yaml
```

Terminal 4:

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irb120pe_cognitive action_adapter_node --ros-args \
  --params-file install/irb120pe_cognitive/share/irb120pe_cognitive/config/cognitive.yaml \
  -p dry_run:=true
```

Terminal 5:

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irb120pe_cognitive langchain_reasoning_node --ros-args \
  --params-file install/irb120pe_cognitive/share/irb120pe_cognitive/config/cognitive.yaml \
  -p llm_provider:=mock
```

## Spawn The Cube Stimulus

Use the cognitive helper so the cube URDF is expanded with xacro before calling `/spawn_entity`. Do not use raw `spawn_entity.py -file BlueCube.urdf` for this validation path.

```bash
cd /root/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irb120pe_cognitive gazebo_cube_helper spawn \
  --cube BlueCube --name BlueCube \
  --x 0.55 --y 0.52 --z 0.88 --qw 1.0 \
  --replace
```

Expected result:

```text
delete False Entity [BlueCube] does not exist
spawn True SpawnEntity: Successfully spawned entity [BlueCube]
```

If `BlueCube` was already present, the first line should instead report a successful delete.

## Probe The Validated Path

```bash
ros2 topic info /camera/image_raw
timeout 7s ros2 topic hz /camera/image_raw
ros2 topic echo /irb120pe/perception/detections_2d --once
ros2 service call /irb120pe/perception/get_detected_objects \
  irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  "{components: {components: 1}}"
ros2 service call /irb120pe/reasoning/execute_instruction \
  irb120pe_cognitive_interfaces/srv/ExecuteInstruction \
  "{instruction: 'Pick the blue cube and place it in the left container'}"
```

Expected result:

```text
/camera/image_raw publishes around 15.9 Hz.
Perception reports blue_1 and sticker_1.
Planning Scene contains blue_1 and sticker_1.
Mock reasoning returns dry_run: would pick blue_1 and place into slot_c.
```

## Delete And Verify Stale Removal

```bash
ros2 run irb120pe_cognitive gazebo_cube_helper delete --name BlueCube
sleep 8
ros2 service call /irb120pe/perception/get_detected_objects \
  irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  "{components: {components: 1}}"
```

Expected result:

```text
detected objects=[]
Planning Scene collision_object_ids=[]
```

Keep `dry_run:=true` and `llm_provider:=mock` until the ROS tool path is ready for non-dry-run validation. Do not command `/Move`, `/Robmove`, or LinkAttacher directly during this demo.
