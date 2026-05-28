# CLAUDE.md

Project guide for Claude Code (and other AI assistants) working on the
ABB IRB-120 Cognitive Robot Cell — the RI_26 master's final project built on
top of IFRA-Cranfield's `irb120_PoseEstimation` repository.

Keep this file short and load-bearing: only facts an agent needs to make good
decisions before reading the code.

---

## Mission in one paragraph

A ROS 2 (Humble) sorting cell where an LLM (LangChain) actually drives the
behavior — it reads YOLOv8 detections, reasons about a natural-language goal
(e.g. *"arrange the cubes in a line by color from white to black to blue along
Y at x=0.55, z=0.90"*), emits per-cube **target poses**, and executes
pick-and-place via MoveIt 2 in Gazebo. The deterministic code cannot produce
those target poses on its own — the LLM is mandatory, not decorative.

---

## Repository layout

| Package | Purpose |
|---|---|
| `irb120pe_gazebo` | URDF/SDF, Gazebo world, cube models, ROS 2 controllers |
| `irb120pe_moveit2` | MoveIt 2 configuration and the `moveit2.launch.py` umbrella |
| `irb120pe_bringup` | Real-robot bringup (not used by the cognitive demo) |
| `irb120pe_detection` | Legacy IFRA pick-and-place scripts + YOLOv8 weights |
| `irb120pe_cognitive_interfaces` | Custom msgs/srvs (`DetectedObject`, `ExecuteInstruction`, `GetDetectedObjects`, `MoveArm`, `PickAndPlace`, **`ArrangeObjects`**) |
| `irb120pe_cognitive` | The cognitive layer: perception node, planning-scene sync, action adapter, **arrangement planner**, LangChain reasoning, demo + e2e validator |

---

## Architecture (strict module boundaries)

```
perception_node  ──(Detection2DArray topic + GetDetectedObjects service)──┐
                                                                          ▼
planning_scene_sync_node  ──(ApplyPlanningScene)──▶  MoveIt Planning Scene
                                                                          ▲
langchain_reasoning_node  ──(MoveArm / PickAndPlace / ArrangeObjects)─────┘
        │
        └── ExecuteInstruction srv  ──▶  action_adapter_node
                                          ├── /move_action (MoveIt)
                                          ├── gripper_cmd actions
                                          └── /ATTACHLINK & /DETACHLINK
```

**Rules an agent must respect:**

- Never collapse the modules. The rubric grades on isolation + ROS communication.
- Every LLM tool must validate coordinates against `workspace_limits` before
  calling the action layer (`validation.validate_coordinates`).
- Tool descriptions in `langchain_reasoning_node` are part of the safety
  contract — they must list every parameter and every refusal condition.

---

## Key entry points

| Command | What it does |
|---|---|
| `ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py` | **Headline demo.** Brings up Gazebo + MoveIt + perception + planning-scene sync + action adapter + LangChain reasoning. |
| `ros2 launch irb120pe_cognitive cognitive_demo.launch.py` | Older single-cube demo (still works). |
| `ros2 run irb120pe_cognitive arrangement_demo` | Spawns three cubes and calls `/irb120pe/reasoning/arrange_objects`. |
| `ros2 run irb120pe_cognitive arrangement_e2e_validator --output-dir ./evidence/arrangement` | **Live runtime verification** — produces `summary.txt` (PASS/FAIL) plus CSV trajectories. |
| `ros2 run irb120pe_cognitive gazebo_cube_helper spawn --cube BlueCube ...` | Manual cube spawn for ad-hoc tests. Use canonical `--name BlueCube/BlackCube/WhiteCube` (see runtime notes). |
| `ros2 run irb120pe_cognitive chat_client` | **Interactive natural-language chat** (Spanish) → `ExecuteInstruction` service. Drives the arm by typing orders. |
| `ros2 topic echo /irb120pe/reasoning/trace` | Watch LLM tool calls live during the demo. |

---

## Verification strategy (three layers)

1. **Pure-Python unit tests** (no ROS, runs anywhere):
   - `test_validation.py`, `test_arrangement_planner.py`, `test_target_pose_validation.py`
   - Run with `colcon test --packages-select irb120pe_cognitive` or
     `python -m pytest test/`.
2. **ROS dry-run e2e**: `test/test_arrangement_e2e_dry_run.py` boots a
   `FakePerceptionNode` + `FakeActionAdapter` + real reasoning node and asserts
   the full arrange service contract.
3. **Live runtime e2e**: `arrangement_e2e_validator` against the full launched
   stack — spawns cubes, records `tool0_trajectory.csv` and
   `cube_trajectory.csv`, checks final cube poses vs LLM-generated targets,
   writes `summary.txt`. **This is the artifact for grading.**

A change is "done" only when Layer 1 + Layer 2 pass, and (for changes that
touch motion) Layer 3 produces a PASS summary.

---

## LLM providers

`llm_provider` parameter selects the backend; default is `mock` for
deterministic CI and recorded demos.

| Provider | Env var | Notes |
|---|---|---|
| `mock` | — | Deterministic; uses the same `arrangement_planner` the tools use. |
| `openai` | `OPENAI_API_KEY` | `langchain-openai` |
| `openrouter` | `OPENROUTER_API_KEY` | **Recommended for the video demo.** Uses `langchain-openai` against `https://openrouter.ai/api/v1`. Model: `anthropic/claude-sonnet-4.5` (the old `claude-3.5-sonnet` slug is retired). Requires `langchain<1` — see *Live-demo runtime notes*. |
| `ollama` | — | Local; `langchain-ollama` |
| `huggingface` | `HUGGINGFACE_ENDPOINT_URL` | `langchain-huggingface` |

---

## Live-demo runtime notes (hard-won, 2026-05-28)

These bit us during a live OpenRouter run. Honor them before touching the demo path:

- **LangChain must be pinned `<1`.** The reasoning node uses the classic
  `langchain.agents.initialize_agent` / `AgentType` API that LangChain 1.x removed.
  Unpinned `pip install langchain ...` pulls 1.x and fails with a misleading
  *"LangChain is not installed"*. Install `langchain<1 langchain-openai<1`.
- **OpenRouter:** model `anthropic/claude-sonnet-4.5`. The node sets `llm_max_tokens`
  (default 1500); without a cap the client reserves the model's full 64000-token
  output and OpenRouter rejects low balances with **HTTP 402**.
- **Spawn cubes with canonical names** `BlueCube`/`BlackCube`/`WhiteCube` (the
  `gazebo_cube_helper` default). `/ATTACHLINK` asks Gazebo for the model named after
  the cube colour, so a custom `--name blue_1` makes attach fail (*model not found*).
  Perception still labels detections `blue_1` etc. — that decoupling is expected.
- **Grasp order is attach-then-close** (not close-then-attach): bonding the cube at the
  clean grasp pose before the fingers squeeze prevents the physics jolt that used to
  "teleport"/displace the cube and ruin downstream placement.
- **Placement/stacking params** in `cognitive.yaml`: `moveit_transit_z` (carry height,
  clears neighbour cubes when traversing), `place_release_clearance` (drop the cube a
  hair so it settles by gravity instead of being driven into what's below — essential
  for stacking), `yolo_confidence: 0.5` (high threshold rejects duplicate/noise
  detections; lowering it spawns phantom `*_2` objects).
- **Chat memory:** the reasoning node keeps `conversation_history_turns` (default 6) of
  context, so references like "yes, put it on the right" resolve across turns.
- **Stickers** (`sticker_1..3`) are YOLO-detected table markers added to the MoveIt
  planning scene as collision obstacles via `moveit_extra_collision_object_ids`.

---

## Conventions

- **Python**: 3.10+ syntax allowed (`X | None`). No comments unless the *why*
  is non-obvious. Don't paraphrase what code already says.
- **Workspace limits** live in `irb120pe_cognitive/config/cognitive.yaml`
  (`workspace_limits.{x,y,z}`, `tool0_workspace_limits.{x,y,z}`, and
  `slots.*`). Cube destinations use `workspace_limits`; direct MoveIt/tool0
  waypoints use `tool0_workspace_limits`.
- **Frame**: planning frame is `world`; end-effector link is `tool0`; grasp
  orientation default is `(0.707, 0.707, 0, 0)`.
- **Target pose semantics**: arrangement and slot poses are cube destinations.
  The action adapter converts destination Z to a `tool0` waypoint with
  `place_z_offset_from_object`.
- **Gripper attach**: `moveit_sim` backend uses IFRA `/ATTACHLINK` and
  `/DETACHLINK` for the cube; do not add a custom attach mechanism.
- **Reasoning trace**: every tool call is published as JSON to
  `/irb120pe/reasoning/trace`. Add `_publish_trace(...)` calls when adding new
  tools so the demo video can show the LLM thinking live.

---

## What NOT to do

- Don't bypass `validate_coordinates`. Every motion target must be checked.
- Don't add legacy `/Robmove` / `/Move` calls — `moveit_sim` is the validated
  Gazebo path; `legacy` exists only for the IFRA reference scripts.
- Don't introduce a new monolithic node. New behavior either lives behind a
  service exposed by an existing node or in a clearly separated new node.
- Don't commit API keys. Use env vars only.
- Don't change interface `.srv` files without also updating
  `CMakeLists.txt` *and* rebuilding the interfaces package before the
  consumer.

---

## Where to read first

- Plan that produced the current arrangement feature:
  `C:\Users\ignag\.claude\plans\i-will-provide-you-luminous-toucan.md`
- `README.md` — install, build, full runbook including the arrangement demo
  and OpenRouter setup.
- `demo_commands.md` — step-by-step video script.
- `agents.md` — original milestone breakdown for the cognitive stack.
- `docs/` — validation evidence from prior runs.

---

## Build & test commands (WSL Ubuntu 22.04, ROS 2 Humble)

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
# Always rebuild interfaces first when .srv files change.
colcon build --symlink-install --packages-select irb120pe_cognitive_interfaces
colcon build --symlink-install --packages-select irb120pe_cognitive
source install/setup.bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```
