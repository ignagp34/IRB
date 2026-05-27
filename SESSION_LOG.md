# Session Log — Live Debugging the Cognitive Stack in WSL

Use this as a hand-off log if you continue in a different terminal. All code
is on your `ignagp34/IRB` fork on the `humble` branch. To catch up locally:

```bash
cd ~/irb120_ws/src/irb120_PoseEstimation
git pull            # uses your cached PAT
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select irb120pe_cognitive_interfaces
colcon build --symlink-install --packages-select irb120pe_cognitive
source install/setup.bash
```

This document summarizes the earlier live WSL debugging session. During the
final publishing pass, Codex could access the Windows checkout but WSL was not
exposed in that app context, so the live ROS/Gazebo results below are recorded
results from that session rather than a fresh rerun.

---

## TL;DR

The live WSL session took the project from "compiled but never run" to:

- **40/40** unit + ROS dry-run tests passing
- Full stack launches cleanly (Gazebo + MoveIt + 4 cognitive nodes)
- Perception detects cubes within **5 mm** of spawn pose
- LLM reasoning service generates valid multi-step arrangement plans from
  natural-language goals
- MoveIt actually executes — arm picks up, attaches, and lifts cubes
  (recorded ~1.5 m of end-effector motion in one run)

**Outstanding**: the place phase intermittently fails on IK or Gazebo's
LinkAttacher destabilises after a few attach/detach cycles. The
cognitive layer is solid; this is a Gazebo/plugin tuning problem.

---

## What we changed in this session (newest → oldest)

| Commit | What it does |
|---|---|
| latest hand-off commit | Compute source pick targets from perceived Z plus calibrated `tool0`-to-gripper offsets; update the guide and this summary |
| `7bfd980` | Fix perception 3D back-projection: SensorDataQoS on camera topics; convert image ray from OPTICAL convention to camera_link convention (X-forward) before rotating; bump `place_approach_offset_z` 0.031 → 0.10 |
| `043e5de` | Fix `camera_frame: camera` → `camera_link` (the original frame name had no TF); patch `Recorder.stop_recording` attribute; fix `arrangement_demo` label-matching bug |
| `b026d2e` | Pin `numpy<2` and `pytest>=7.4` in install instructions + troubleshooting section |
| `4ee0009` | Use `~/irb120_ws` everywhere; add GitHub PAT subsection (no token literals) |
| `5437d3a` | Add `STEP_BY_STEP.md` |
| `043e5de`..`ca80c59` | Earlier RI_26 cognitive scaffolding (interfaces, reasoning node, action adapter, planner, e2e validator, README, demo_commands) |

---

## Environment fixes (apply once on a fresh WSL Ubuntu 22.04)

```bash
pip install --user "numpy<2" "opencv-python<4.12" ultralytics langchain langchain-openai
pip install --user --upgrade "pytest>=7.4" pytest-asyncio
```

These are also baked into **Step 2** of `STEP_BY_STEP.md`.

---

## What we proved end-to-end (live in WSL)

### 1. Tests
```
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
# → 40 tests, 0 errors, 0 failures, 0 skipped
```

### 2. Stack launch
```
ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py \
  dry_run:=false llm_provider:=mock execution_backend:=moveit_sim
```
All 4 cognitive nodes alive, services advertised:
- `/irb120pe/perception/get_detected_objects`
- `/irb120pe/action/move_arm`
- `/irb120pe/action/pick_and_place`
- `/irb120pe/reasoning/execute_instruction`
- `/irb120pe/reasoning/arrange_objects` (new)
- Topic `/irb120pe/reasoning/trace` (new)

### 3. Perception (after fixes)
Spawning a `BlueCube` at `(0.55, 0.55, 0.88)` →
**detected at (0.555, 0.550, 0.900)** — within 5 mm.

### 4. Robot geometry (verified via TF)
```
world → base_link    : x=+0.275  y=+0.525  z=+0.861   ← robot is on a pedestal
world → tool0        : x=+0.577  y=+0.525  z=+1.402
world → EE_egp64     : x=+0.566  y=+0.525  z=+1.212
world → CubeTray     : x=+0.080  y=+0.071  z=+0.860
world → camera_link  : x=+0.776  y=+0.526  z=+1.900
```
**Important**: the IRB-120 base is at `z=0.861` m, **not** at floor. All
coordinates in `cognitive.yaml` are in world frame and already account
for this. Don't try to subtract the offset.

### 5. Reasoning produces a valid plan
```
ros2 service call /irb120pe/reasoning/arrange_objects \
  irb120pe_cognitive_interfaces/srv/ArrangeObjects \
  "{instruction: 'Arrange the cubes in a line by color from white to black to blue along Y at x=0.50, z=1.00, spacing=0.06, start=0.40'}"
# Returns 3 ordered (object_id, target_pose) steps inside workspace_limits.
```

### 6. MoveIt executes
Best run: pick → descend → close gripper → attach cube → lift →
approach place slot all completed. `tool0_trajectory.csv` recorded
**~1.5 m of arm motion**.

---

## What still fails intermittently

1. **Place IK** — `error_code=-31` (NO_IK_SOLUTION) when descending to the
   place pose while holding a cube. The source-height fix preserves the
   calibrated table pick behavior and tracks elevated detections; it does not
   by itself prove that the place IK issue is resolved. If place still fails,
   raise the arrangement's `z=1.00` to `z=1.05` in the instruction.
2. **Gazebo / LinkAttacher SIGABRT** — `gzserver` exit `-6` after a few
   attach/detach cycles. Pre-existing IFRA plugin instability. Workarounds:
   - Restart the full stack between arrangement runs.
   - Test with 1 cube before 3.

---

## Commands you'll want in the new terminal

```bash
# Terminal A — full stack
cd ~/irb120_ws && source install/setup.bash
ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py \
  dry_run:=false llm_provider:=mock execution_backend:=moveit_sim

# Terminal B — watch LLM thinking
source ~/irb120_ws/install/setup.bash
ros2 topic echo /irb120pe/reasoning/trace

# Terminal C — quick perception probe
source ~/irb120_ws/install/setup.bash
ros2 run irb120pe_cognitive gazebo_cube_helper spawn \
  --cube BlueCube --name BlueCube --x 0.55 --y 0.55 --z 0.88 --replace
ros2 service call /irb120pe/perception/get_detected_objects \
  irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
# Expect detection within 5 mm of spawn pose.

# Terminal C — run the headline mission
ros2 run irb120pe_cognitive arrangement_demo

# Terminal C — run the e2e validator (writes evidence + summary.txt)
cd ~/irb120_ws
rm -rf evidence/arrangement
ros2 run irb120pe_cognitive arrangement_e2e_validator \
  --output-dir ./evidence/arrangement \
  --reasoning-timeout 600 \
  --instruction 'Arrange the cubes in a line by color from white to black to blue along Y at x=0.50, z=1.00, spacing=0.06, start=0.40'
cat ./evidence/arrangement/summary.txt
```

If `summary.txt` still has FAIL on `arrange_objects success`, the next
debug actions are:

1. Try `z=1.05` instead of `z=1.00` in the instruction (more clearance).
2. Try `start=0.45` instead of `0.40` (further from robot base for IK).
3. Try a single cube first via:
   ```bash
   ros2 service call /irb120pe/reasoning/execute_instruction \
     irb120pe_cognitive_interfaces/srv/ExecuteInstruction \
     "{instruction: 'Pick the blue cube and place it in the right container'}"
   ```

---

## File map of what was added this project

```
irb120pe_cognitive_interfaces/
  srv/ArrangeObjects.srv          ← new srv for the LLM-driven mission
  srv/PickAndPlace.srv            ← added optional target_pose

irb120pe_cognitive/
  irb120pe_cognitive/
    arrangement_planner.py        ← deterministic layout parser (pure Python)
    arrangement_demo.py           ← spawn 3 cubes + call arrange_objects
    arrangement_e2e_validator.py  ← live runtime evidence collector
    langchain_reasoning_node.py   ← +arrange_objects service, +OpenRouter,
                                    +trace publisher, +system prompt,
                                    +move_object_to_pose_tool,
                                    +arrange_objects_tool
    action_adapter_node.py        ← honor free target_pose; detection-relative pick targets with tool offsets
    perception_node.py            ← QoS + optical-frame ray fix
  config/
    cognitive.yaml                ← arrangement defaults, openrouter,
                                    camera_frame:=camera_link,
                                    place_approach_offset_z:=0.10
  launch/
    cognitive_arrangement_demo.launch.py
  test/
    test_arrangement_planner.py     (7 tests)
    test_target_pose_validation.py  (4 tests)
    test_arrangement_e2e_dry_run.py (ROS dry-run with FakePerceptionNode)

CLAUDE.md            ← agent guide
STEP_BY_STEP.md      ← human walkthrough
SESSION_LOG.md       ← this file
README.md / demo_commands.md ← updated with arrangement demo
```

---

## OpenRouter (optional, for the demo video)

```bash
export OPENROUTER_API_KEY=sk-or-...     # set from your shell, never commit
pip install --user langchain-openai      # if not already installed

# Stop the launch (Ctrl-C) and relaunch with:
ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py \
  dry_run:=false execution_backend:=moveit_sim \
  llm_provider:=openrouter \
  llm_model:=anthropic/claude-3.5-sonnet
```

Recommended models:
- `anthropic/claude-3.5-sonnet` — best structured tool use
- `openai/gpt-4o-mini` — cheaper, reliable
- `google/gemini-2.0-flash-exp:free` — free tier

Default stays on `mock` for deterministic CI/recorded demos.
