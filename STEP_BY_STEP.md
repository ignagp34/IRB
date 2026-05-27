# Step-by-Step Guide (For Dummies)

A literal copy-paste walkthrough to take this repository from zero to a
working arrangement demo with evidence files. No prior ROS knowledge assumed.

If a command fails, jump to **[Troubleshooting](#troubleshooting)** at the bottom.

---

## Step 0 — What you'll end up with

After ~30 minutes you will have:

- An ABB IRB-120 robotic arm simulating in Gazebo.
- A YOLOv8 camera detecting three colored cubes (white, black, blue).
- An LLM (mock by default, OpenRouter optional) that takes a natural-language
  goal like *"arrange the cubes in a line by color along Y at x=0.55"*,
  reasons about it, and executes pick-and-place.
- A `summary.txt` file that proves PASS/FAIL for every check
  (perception, planning scene, end-effector motion, final cube positions).

---

## Step 1 — Open WSL Ubuntu 22.04

> ROS 2 Humble only runs on Ubuntu 22.04. On Windows, that means WSL 2.

**Windows PowerShell** (one-time, only if you don't already have it):

```powershell
wsl --install -d Ubuntu-22.04
```

Then open it:

```powershell
wsl -d Ubuntu-22.04
```

From now on **every command runs inside WSL** unless it says "PowerShell".

---

## Step 2 — Install ROS 2 Humble and dependencies (one-time)

Inside WSL:

```bash
sudo apt update
sudo apt install -y \
  curl gnupg lsb-release locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add the ROS 2 apt repository
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble + MoveIt + Gazebo
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-opencv \
  git

# Initialize rosdep
sudo rosdep init || true
rosdep update
```

Install the Python packages. **Pin NumPy and OpenCV below 2.0** —
ROS 2 Humble's `cv_bridge`, `matplotlib`, and the system OpenCV were
compiled against NumPy 1.x, so a newer NumPy breaks perception with
`_ARRAY_API not found`. `ultralytics` may otherwise pull NumPy 2.x
and a NumPy-2-only OpenCV. Install them in one command so `pip`
resolves the constraints together:

```bash
pip install --user "numpy<2" "opencv-python<4.12" ultralytics langchain langchain-openai
```

If you already installed `ultralytics` and Gazebo's perception node dies
with `numpy.core.multiarray failed to import`, run:

```bash
pip install --user --upgrade "numpy<2" "opencv-python<4.12"
```

Then relaunch the stack.

Also make sure `pytest` is recent enough (the system 6.2 conflicts with
the user-installed `anyio` plugin that requires pytest ≥ 7.0):

```bash
pip install --user --upgrade "pytest>=7.4" pytest-asyncio
```

---

## Step 3 — Get the source code

```bash
cd ~
mkdir -p irb120_ws/src
cd irb120_ws/src

# Main repository (this project)
git clone -b humble https://github.com/ignagp34/IRB.git irb120_PoseEstimation

# Required IFRA dependencies
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
git clone https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.git
```

If the clone of this project fails, double-check the URL (it should be the
fork you pushed to: `ignagp34/IRB`).

### If git asks for credentials (private repo)

GitHub no longer accepts passwords over HTTPS. You need a **Personal Access
Token** (PAT):

1. GitHub → **Settings → Developer settings → Personal access tokens →
   Tokens (classic) → Generate new token (classic)**.
2. Scope: tick `repo` (and only that, unless you know you need more).
3. Copy the token **once**. Treat it like a password — never paste it into
   docs, commits, or chat. If it leaks, **revoke it immediately** from the
   same page.
4. When git asks for a password, paste the token. Cache it for the session:

```bash
git config --global credential.helper "cache --timeout=3600"
```

> Public repos (like `IFRA-Cranfield/...`) do **not** require a PAT — no
> credentials needed at all. You only need one if you're cloning a private
> fork or pushing changes back.

---

## Step 4 — Build the workspace

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash

# Resolve missing system deps
rosdep install --from-paths src --ignore-src -r -y || true

# Build (skip the legacy executor, it is not needed for the cognitive demo)
colcon build --symlink-install --packages-skip ros2srrc_execution
source install/setup.bash
```

Expected end of output: `Summary: N packages finished` with no failures.

> If `colcon build` fails on the cognitive package, **rebuild the interfaces
> first**, then the cognitive package:
>
> ```bash
> colcon build --symlink-install --packages-select irb120pe_cognitive_interfaces
> source install/setup.bash
> colcon build --symlink-install --packages-select irb120pe_cognitive
> source install/setup.bash
> ```

---

## Step 5 — Run the unit tests (fast sanity check, no Gazebo needed)

```bash
cd ~/irb120_ws
source install/setup.bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

You want to see `0 errors, 0 failures`.

---

## Step 6 — Launch the full demo stack

This is the **Terminal A** command. Keep it running.

```bash
cd ~/irb120_ws
source install/setup.bash
ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py \
  dry_run:=false llm_provider:=mock execution_backend:=moveit_sim
```

After ~30 seconds you should see:

- A Gazebo window with the IRB-120 arm and a camera.
- An RViz window showing the robot, TF frames, and (initially empty) Planning
  Scene.
- Terminal logs ending with `irb120pe_langchain_reasoning` initialized.

Leave this terminal running for everything that follows.

---

## Step 7 — Watch the LLM think (optional but cool)

Open **Terminal B** in WSL:

```bash
cd ~/irb120_ws
source install/setup.bash
ros2 topic echo /irb120pe/reasoning/trace
```

Each LLM tool call publishes a JSON message here. Leave it streaming.

---

## Step 8 — Run the arrangement demo

Open **Terminal C** in WSL:

```bash
cd ~/irb120_ws
source install/setup.bash
ros2 run irb120pe_cognitive arrangement_demo
```

What will happen:

1. Three cubes (White, Black, Blue) spawn in Gazebo.
2. The perception node detects them.
3. The reasoning node receives the canonical instruction
   *"arrange the cubes in a line by color from white to black to blue along Y
   at x=0.55, z=0.90, spacing=0.06"*.
4. The arm picks each cube and places it at the LLM-generated coordinates.
5. The script prints the JSON plan and exits with `arrangement success: True`.

You should see the cubes line up along the Y axis in Gazebo.

---

## Step 9 — Run the live verification (the artifact for grading)

Still in **Terminal C** (Terminal A must still be running):

```bash
cd ~/irb120_ws
source install/setup.bash
ros2 run irb120pe_cognitive arrangement_e2e_validator \
  --output-dir ./evidence/arrangement
```

This takes ~3–5 minutes. When it finishes:

```bash
cat ./evidence/arrangement/summary.txt
```

You want the first line to say **`Arrangement E2E validation: PASS`**.

Inside `./evidence/arrangement/` you will find:

| File | What it proves |
|---|---|
| `summary.txt` | PASS/FAIL per check — the single grading artifact |
| `spawn_*.txt` | Each cube was placed in Gazebo |
| `detections_initial.txt` | Perception read each cube's pose within 5 cm |
| `planning_scene_initial.txt` | MoveIt knows about the cubes as collision objects |
| `plan_json.txt` | The ordered LLM-generated arrangement plan |
| `reasoning_response.txt` | The reasoning service returned success |
| `tool0_trajectory.csv` | End-effector position over time (from TF) |
| `cube_trajectory.csv` | Each cube's position over time |
| `final_state.txt` | Final cube poses vs the planned targets (delta per cube) |
| `planning_scene_final.txt` | Planning scene was updated after the arrangement |

Show `summary.txt` in the video. That's enough.

---

## Step 10 — (Optional) Use a real LLM via OpenRouter

If you want the LLM to actually do the reasoning (instead of the deterministic
mock fallback), point the demo at OpenRouter:

```bash
# In WSL
export OPENROUTER_API_KEY=sk-or-XXXXXXXXXXXXXXXXXXXX
pip install langchain-openai

# Stop Terminal A (Ctrl-C) and relaunch with the openrouter provider:
ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py \
  dry_run:=false execution_backend:=moveit_sim \
  llm_provider:=openrouter llm_model:=anthropic/claude-3.5-sonnet
```

Then repeat **Step 8** (`arrangement_demo`). The trace topic will now show
the real LLM's tool calls. **Recommended models:**

- `anthropic/claude-3.5-sonnet` — best tool-use; use for the video.
- `openai/gpt-4o-mini` — cheaper, still reliable.
- `google/gemini-2.0-flash-exp:free` — free tier for testing.

---

## Step 11 — Demo recipe for the video (15-30 min)

A clean recording order:

1. Show the architecture diagram from `README.md`.
2. Run **Step 6** (launch full stack).
3. Open **Terminal B** with `ros2 topic echo /irb120pe/reasoning/trace`.
4. Run a single-cube command first to warm up:
   ```bash
   ros2 service call /irb120pe/reasoning/execute_instruction \
     irb120pe_cognitive_interfaces/srv/ExecuteInstruction \
     "{instruction: 'Pick the blue cube and place it in the right container'}"
   ```
5. Run **Step 8** (`arrangement_demo`) for the headline LLM-driven mission.
6. Run **Step 9** (`arrangement_e2e_validator`) and `cat summary.txt`.
7. Show RViz Planning Scene updating with the cubes during the arrangement.

---

## Troubleshooting

### "command not found: ros2" or "colcon"
You forgot to `source install/setup.bash` (or `/opt/ros/humble/setup.bash`)
in this terminal. Source it again.

### Gazebo window does not open
WSL needs WSLg. From PowerShell: `wsl --update`. Reboot Windows. Try
`gazebo --version` in WSL — if it fails, reinstall: `sudo apt install --reinstall ros-humble-gazebo-ros-pkgs`.

### `perception_node` dies with `_ARRAY_API not found` / `numpy.core.multiarray failed to import`

Your user-level NumPy or OpenCV is incompatible with ROS Humble's
`cv_bridge`. Fix both constraints:

```bash
pip install --user --upgrade "numpy<2" "opencv-python<4.12"
```

Then Ctrl-C the launch and restart it.

### `colcon test` fails with `ModuleNotFoundError: No module named '_pytest.scope'`

System pytest is 6.2, the user-installed `anyio` plugin needs ≥ 7.0. Fix:

```bash
pip install --user --upgrade "pytest>=7.4" pytest-asyncio
```

### YOLO model not found
Check that `irb120pe_detection/yolov8/cubeDETECTION_Gz.pt` exists in
`install/irb120pe_detection/share/...`. If not, rebuild
`irb120pe_detection` with `colcon build --symlink-install --packages-select irb120pe_detection`.

### Reasoning service times out
Make sure the launch in Terminal A finished initializing. Wait until you
see `irb120pe_langchain_reasoning` and `irb120pe_action_adapter` in the
logs before calling any service.

### Perception sees no cubes
Spawn them manually:

```bash
ros2 run irb120pe_cognitive gazebo_cube_helper spawn \
  --cube BlueCube --name BlueCube --x 0.55 --y 0.52 --z 0.88 --replace
```

Then check:

```bash
ros2 service call /irb120pe/perception/get_detected_objects \
  irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
```

### Arm doesn't move
Check that `execution_backend:=moveit_sim` is set in your launch command
(not `legacy`). Confirm `/move_action` exists: `ros2 action list | grep move_action`.

### Pick height looks wrong
The IRB-120 is mounted on the cell pedestal: in this world `base_link` is
near `z=0.861`, not on the floor. Do not subtract that height from world-frame
poses. The action adapter calculates source goals from detected object Z plus
the configured `tool0`-to-gripper offsets (`pick_z_offset_from_object` and
`pick_approach_offset_from_object`).

Arrangement and named-slot `target_pose` values are cube destinations, not
`tool0` goals. A tabletop line uses `z=0.90`; the adapter adds
`place_z_offset_from_object` before asking MoveIt to descend.

### Cube doesn't drop in the slot (gripper missed)
The `summary.txt` will mark `cubes reached planned targets` as FAIL.
The tolerance is 6 cm. If you are consistently off, lower
`moveit_velocity_scaling` in `irb120pe_cognitive/config/cognitive.yaml`
and rebuild.

### OpenRouter rejects the request
The free models have rate limits. Switch to `anthropic/claude-3.5-sonnet`
or check your account balance.

### `colcon test` fails on the dry-run e2e test
That test boots a ROS context — if you have another `ros2` process
running on the same `ROS_DOMAIN_ID`, kill it first.

---

## Glossary (one-line each)

- **ROS 2 Humble** — robotics middleware. Provides nodes / topics / services / actions.
- **MoveIt 2** — motion planner. Computes safe joint trajectories.
- **Gazebo Classic** — physics simulator. Provides the camera image and the cube physics.
- **YOLOv8** — vision model. Detects cubes in the camera image.
- **LangChain** — LLM agent framework. We use it to expose ROS calls as "tools" to an LLM.
- **OpenRouter** — LLM API gateway (OpenAI-compatible). Lets you swap models like Claude/GPT/Gemini with one env var.
- **Planning Scene** — MoveIt's world model. We sync perceived cubes into it as collision objects.
- **TF / TF2** — coordinate-frame tree. We use `world -> tool0` to read the end-effector pose at runtime.
