# Step 0 - Handoff Summary

This folder summarizes the work completed before ROS 2/WSL integration testing. Use it as the starting point when continuing from another terminal.

## Current State

- The upstream `IFRA-Cranfield/irb120_PoseEstimation` repository was restored locally on branch `humble`.
- The local Windows workspace is:

```text
C:\Users\Ignacio González\OneDrive\Escritorio\IRB
```

- WSL exists, but only `Ubuntu-24.04` is currently registered.
- The target runtime is still **Ubuntu 22.04 WSL**, because the project targets ROS 2 Humble, MoveIt 2, and Gazebo Classic.
- Code has been added, but the full ROS build/demo has **not** been validated yet.

## What Was Added

### New ROS 2 Interface Package

Folder:

```text
irb120pe_cognitive_interfaces/
```

Adds:

- `msg/DetectedObject.msg`
- `srv/GetDetectedObjects.srv`
- `srv/ExecuteInstruction.srv`
- `srv/MoveArm.srv`
- `srv/PickAndPlace.srv`

Purpose: typed ROS interfaces between perception, reasoning, and action modules.

### New Cognitive ROS 2 Package

Folder:

```text
irb120pe_cognitive/
```

Adds:

- `perception_node.py`
- `planning_scene_sync_node.py`
- `action_adapter_node.py`
- `langchain_reasoning_node.py`
- shared validation/config helpers
- launch files
- lightweight tests
- `config/cognitive.yaml`

Purpose: separate perception, planning-scene updates, validated robot action tools, and LangChain/mock reasoning.

### Documentation And Project Support

Added or updated:

- `README.md`
- `.env.example`
- `.gitignore`
- `agents.md`
- `demo_commands.md`
- `docs/architecture.md`
- `docs/modules.md`
- `docs/experiments.md`
- `docs/generative_ai_usage.md`
- `docs/setup_wsl_ubuntu22.md`

## Important Design Decisions

- The original deterministic scripts were preserved:
  - `irb120pe_detection/python/main.py`
  - `irb120pe_detection/python/main_Gz.py`
  - `irb120pe_detection/python/main_GzSimplified.py`
- The LLM does not command robot topics directly.
- Robot motion must go through strict tools/services:
  - `get_detected_objects`
  - `get_available_slots`
  - `move_arm_tool`
  - `pick_and_place_tool`
- Default LLM provider is `mock`, so the demo can run without API keys.
- OpenAI/Ollama/Hugging Face support is configurable but untested in ROS.
- Red cube commands are intentionally unsupported until the YOLO model and simulation assets include a red class.

## Verification Already Done

From Windows Python:

- Python compile check passed for new Python nodes and launch files.
- Validation smoke test passed.
- XML package files parsed successfully.
- YAML config parsed successfully.

Not done yet:

- `colcon build`
- `colcon test`
- Gazebo launch
- MoveIt launch
- Perception live test
- Planning Scene live test
- LangChain/mock reasoning service live test
- Full pick-and-place demo

## Next Terminal Steps

### 1. Install Ubuntu 22.04 WSL

From Windows PowerShell:

```powershell
wsl --install -d Ubuntu-22.04
wsl -d Ubuntu-22.04
```

If Windows requests a reboot, reboot first and then run:

```powershell
wsl -d Ubuntu-22.04
```

### 2. Install ROS 2 Humble Environment

Inside Ubuntu 22.04:

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release git python3-pip python3-venv
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-moveit ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init || true
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

### 3. Create WSL Workspace

Recommended path:

```bash
mkdir -p ~/irb120_ws/src
```

Copy or clone the repo into:

```text
~/irb120_ws/src/irb120_PoseEstimation
```

Do not build from OneDrive or `/mnt/c`; builds are slower and less reliable there.

### 4. Install Dependencies And Build

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 5. Run Tests

```bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

### 6. Validate The Demo In Stages

Base Gazebo + MoveIt:

```bash
ros2 launch irb120pe_moveit2 moveit2.launch.py
```

Perception:

```bash
ros2 launch irb120pe_cognitive cognitive_perception.launch.py
ros2 service call /irb120pe/perception/get_detected_objects irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
```

Planning Scene:

```bash
ros2 launch irb120pe_cognitive planning_scene.launch.py
```

Reasoning/action adapter in mock mode:

```bash
ros2 launch irb120pe_cognitive reasoning.launch.py dry_run:=true
```

Natural-language command:

```bash
ros2 service call /irb120pe/reasoning/execute_instruction irb120pe_cognitive_interfaces/srv/ExecuteInstruction "{instruction: 'Pick the blue cube and place it in the left container'}"
```

Full stack:

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py
```

## Expected Integration Work

The first `colcon build` may reveal missing IFRA dependencies such as:

- `ros2srrc_data`
- `ros2srrc_execution`
- `objectpose_msgs`
- `linkpose_msgs`
- `linkattacher_msgs`
- ABB support packages

Install or clone those into the same WSL workspace as needed.

## Files To Read First

1. `agents.md`
2. `docs/setup_wsl_ubuntu22.md`
3. `README.md` cognitive extension section
4. `irb120pe_cognitive/config/cognitive.yaml`
5. `docs/experiments.md`

## Bottom Line

The code and documentation for the cognitive extension are in place. The project is now ready for WSL/ROS integration testing, but it is not yet proven as a complete working robot demo.
