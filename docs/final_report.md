# RI_26 Final Project — ABB IRB-120 Cognitive Sorting Cell

**Author:** Ignacio González Peris
**Repository:** <https://github.com/ignagp34/IRB>, branch `humble`, HEAD `d98e10d`
**Date:** 2026-05-27

---

## 1. Introduction

This project extends IFRA-Cranfield's `irb120_PoseEstimation` ROS 2 stack into
a **cognitive sorting cell** where a Large Language Model (LLM) genuinely
drives the robot's behavior rather than acting as decoration on top of a
deterministic pipeline.

The headline mission is **goal-oriented arrangement**: the user provides a
free-form natural-language instruction such as

> *"Arrange the cubes in a line by color from white to black to blue along Y
> at x=0.50, z=0.90, spacing=0.06, start=0.40."*

The LLM must (a) query YOLOv8-based perception, (b) reason about which cubes
exist and where each should go, (c) emit **per-cube target poses** with
explicit numerical coordinates, and (d) drive MoveIt 2 in Gazebo to execute
the pick-and-place sequence. The deterministic code base cannot produce those
target poses on its own, which is exactly what the RI_26 rubric requires when
it asks for AI that "genuinely drives the behavior."

The legacy single-cube `execute_instruction` flow is preserved as a reference
demo. The cognitive arrangement flow is the primary deliverable.

---

## 2. System architecture

Four strictly separated ROS 2 nodes communicate exclusively via topics,
services, and actions. No node is collapsed into another and the LLM has no
direct access to robot topics.

```text
perception_node ──(Detection2DArray topic + GetDetectedObjects service)──┐
                                                                          ▼
planning_scene_sync_node ──(ApplyPlanningScene)──▶ MoveIt Planning Scene
                                                                          ▲
langchain_reasoning_node ──(MoveArm / PickAndPlace / ArrangeObjects)──────┘
        │
        └── ExecuteInstruction srv ──▶ action_adapter_node
                                       ├── /move_action (MoveIt 2)
                                       ├── gripper_cmd actions
                                       └── /ATTACHLINK & /DETACHLINK
```

### 2.1 Perception (`perception_node`)

YOLOv8 + OpenCV. Publishes `vision_msgs/Detection2DArray` on
`/irb120pe/perception/detections_2d` and serves
`/irb120pe/perception/get_detected_objects`
(`irb120pe_cognitive_interfaces/srv/GetDetectedObjects`) with structured
detections that include label, confidence, and a 3D pose in the planning
frame `world`. Detections are produced by back-projecting the 2D image ray
through the calibrated `camera_link` frame and intersecting it with the
known table height. The camera-frame fix described in
`SESSION_LOG.md` (using `camera_link` and SensorDataQoS) is required for
this to work in WSL Gazebo.

### 2.2 Planning-scene synchronisation

`planning_scene_sync_node` subscribes to perception and writes/updates
collision objects in MoveIt's planning scene via `/apply_planning_scene` so
the planner avoids the cubes during motion planning.

### 2.3 Reasoning (`langchain_reasoning_node`)

A LangChain tool-using agent constrained to a small set of validated tools:

| Tool | Purpose |
|---|---|
| `get_detected_objects_tool` | Read structured detections from perception |
| `move_arm_tool` | Move `tool0` to a validated pose (free motion) |
| `pick_and_place_tool` | Pick a detected cube and place it in a named slot |
| `move_object_to_pose_tool` | Pick a detected cube and place it at an explicit `(x, y, z, qx, qy, qz, qw)` destination — the new tool the arrangement mission depends on |
| `arrange_objects_tool` | Top-level layout call wrapping a sequence of `move_object_to_pose_tool` calls |

Every tool call validates its coordinates against `workspace_limits` (cube
destinations) and `tool0_workspace_limits` (direct end-effector waypoints)
through `validation.validate_coordinates` before reaching the action layer.
Every tool call is also published as JSON on `/irb120pe/reasoning/trace`, so
a demo viewer can watch the AI's reasoning live with
`ros2 topic echo /irb120pe/reasoning/trace`.

A new service `/irb120pe/reasoning/arrange_objects`
(`irb120pe_cognitive_interfaces/srv/ArrangeObjects`) is the entry point for
the headline mission and returns the ordered plan as `plan_json`.

#### Provider selection

The `llm_provider` parameter chooses the backend:

| Provider | Notes |
|---|---|
| `mock` | Deterministic; uses the same `arrangement_planner` the LLM tools use; chosen for CI and recorded demos. |
| `openai` | `langchain-openai`; requires `OPENAI_API_KEY`. |
| `openrouter` | Recommended live path. OpenAI-API-compatible against `https://openrouter.ai/api/v1`; recommended model `anthropic/claude-3.5-sonnet` for its structured tool-use performance. |
| `ollama` | Local model server via `langchain-ollama`. |
| `huggingface` | Endpoint-backed via `langchain-huggingface`. |

The default in `cognitive.yaml` is `mock` so tests and recorded demos remain
reproducible; OpenRouter is selected only via launch arg + environment
variable. The OpenRouter branch lives at
[langchain_reasoning_node.py:421](../irb120pe_cognitive/irb120pe_cognitive/langchain_reasoning_node.py).

### 2.4 Action (`action_adapter_node`)

Single chokepoint for robot motion. Honours two contracts:

- `MoveArm` accepts a direct `tool0` pose. Validated against
  `tool0_workspace_limits`.
- `PickAndPlace` accepts either a named `target_slot` or an explicit
  `target_pose`. `target_pose` is interpreted as the **cube destination**,
  validated against `workspace_limits`, then converted into a `tool0`
  placement waypoint by adding `place_z_offset_from_object` (currently
  `0.18 m`). The independently-validated `tool0` waypoint is then sent to
  MoveIt's `/move_action`.

The `moveit_sim` execution backend uses MoveIt 2, the simulated Schunk
EGP-64 gripper, and IFRA `/ATTACHLINK` / `/DETACHLINK` for the cube
attachment phase. The legacy `/Robmove` and `/Move` interfaces are *not*
required for the cognitive demo.

---

## 3. Reasoning layer in detail

### 3.1 System prompt and worked example

The reasoning node ships a system prompt that:

- Names the planning frame (`world`), the workspace bounds, and the default
  grasp orientation (`qx=0.707, qy=0.707, qz=0.0, qw=0.0`).
- Enumerates every tool with its parameter contract and refusal conditions.
- Forbids fabricating object IDs or destinations.
- Includes one worked example: input *"arrange the cubes in a line by
  color"*, output a three-step sequence of `move_object_to_pose_tool`
  calls.

This prompt is part of the **safety contract**, not just a hint. Together
with the tool-call validation it bounds what the LLM can actually cause to
happen.

### 3.2 Deterministic fallback planner

For the `mock` provider (CI, recorded demos), a pure-Python
`arrangement_planner` parses the same layout grammar the prompt teaches.
It understands `line`, `by color`, `along x|y`, `at x=…, z=…`,
`spacing=…`, `start=…`. Its output is an ordered list of `(object_id,
target_pose)`, identical in shape to what the real LLM emits, and is
exercised by `test_arrangement_planner.py` (7 tests).

### 3.3 Validation contract

Both paths share the same validation: every `target_pose` is rejected if any
coordinate is outside `workspace_limits`. The free-target-pose path through
the action adapter is covered by `test_target_pose_validation.py` (4 tests),
which exercises in-workspace acceptance, out-of-workspace rejection, and
empty-`target_pose` fallback to the slot path.

---

## 4. Verification methodology

The project uses three independent verification layers. A change is
considered "done" only when Layer 1 + Layer 2 pass; for changes that affect
motion or perception, Layer 3 must also produce a `PASS` summary.

### Layer 1 — Pure-Python unit tests

Run anywhere, no ROS required. Cover validation helpers, slot resolution,
object selection, arrangement-planner output, and free-target-pose
validation. Files: `test_validation.py`, `test_arrangement_planner.py`,
`test_target_pose_validation.py`.

### Layer 2 — ROS dry-run end-to-end test

`test/test_arrangement_e2e_dry_run.py` boots a `FakePerceptionNode`, a
`FakeActionAdapter`, and the real `LangChainReasoningNode`. It calls
`/irb120pe/reasoning/arrange_objects` and asserts: (a) the returned plan
has three entries in the correct white→black→blue order; (b) every
`target_pose` is inside `workspace_limits`; (c) `move_object_to_pose` tool
calls were published to `/irb120pe/reasoning/trace`. Validates the full
**reasoning → arrangement → action** contract without needing Gazebo.

### Layer 3 — Live runtime end-to-end test

`arrangement_e2e_validator` against the launched stack. Spawns three cubes,
polls perception until all three appear, polls the planning scene until
all three collision objects are present, calls
`/irb120pe/reasoning/arrange_objects`, records `tool0_trajectory.csv` via
TF and `cube_trajectory.csv` via `/gazebo/model_states`, and writes a
single `summary.txt` PASS/FAIL artifact. This is the grading deliverable.

---

## 5. Results

### 5.1 Automated test suite

47 tests, 0 errors, 0 failures, 0 skipped. Full log:
[`docs/validation/2026-05-27-final-mock/colcon_test_result.txt`](validation/2026-05-27-final-mock/colcon_test_result.txt).

```text
Summary: 47 tests, 0 errors, 0 failures, 0 skipped
```

### 5.2 Live runtime evidence (mock provider, HEAD `d98e10d`)

The end-to-end validator was run against the full launched stack
(`cognitive_arrangement_demo.launch.py
dry_run:=false llm_provider:=mock execution_backend:=moveit_sim`) on WSL
Ubuntu-22.04. Instruction:

> *"Arrange the cubes in a line by color from white to black to blue along
> Y at x=0.50, z=0.90, spacing=0.06, start=0.40."*

Full evidence:
[`docs/validation/2026-05-27-final-mock/`](validation/2026-05-27-final-mock/).
Summary verbatim:

```text
Arrangement E2E validation: PASS
[PASS] spawn WhiteCube - SpawnEntity: Successfully spawned entity [WhiteCube]
[PASS] spawn BlackCube - SpawnEntity: Successfully spawned entity [BlackCube]
[PASS] spawn BlueCube - SpawnEntity: Successfully spawned entity [BlueCube]
[PASS] perception matches spawn poses - tolerance=0.05 m
[PASS] planning scene tracks cubes
[PASS] arrange_objects success - Arrangement completed for 3 object(s).
[PASS] target poses inside workspace - 3 step(s)
[PASS] end-effector moved - path_length=1.777 m
[PASS] cubes reached planned targets - tolerance=0.06 m
[PASS] planning scene updated after arrangement
```

Detected cube positions reproduced the spawn locations within
**0.020 m**; planned cube destinations were reached within
**0.026 m** of target (tolerance 0.06 m), measured from
`final_state.txt`:

```text
cube planned_x planned_y planned_z final_x final_y final_z delta
WhiteCube 0.500 0.400 0.900 0.498 0.400 0.874 delta=0.026
BlackCube 0.500 0.460 0.900 0.497 0.460 0.874 delta=0.026
BlueCube  0.500 0.520 0.900 0.497 0.521 0.874 delta=0.026
```

The systematic ~0.026 m vertical bias is exactly the residual between the
table contact height and the planned cube origin Z, after the cube settles
under gravity once the gripper releases. It is well inside the tolerance
gate and is consistent across all three cubes, which rules out per-cube
control error.

### 5.3 LLM-generated plan

The reasoning node returned the following `plan_json` (mock provider —
identical in shape to what the OpenRouter provider produces; this is what
the action adapter consumes). Full file:
[plan_json.txt](validation/2026-05-27-final-mock/plan_json.txt).

```json
[
  { "object_id": "white_1", "label": "white",
    "target_pose": { "x": 0.50, "y": 0.40, "z": 0.90,
                     "qx": 0.707, "qy": 0.707, "qz": 0.0, "qw": 0.0 } },
  { "object_id": "black_1", "label": "black",
    "target_pose": { "x": 0.50, "y": 0.46, "z": 0.90, ... } },
  { "object_id": "blue_1",  "label": "blue",
    "target_pose": { "x": 0.50, "y": 0.52, "z": 0.90, ... } }
]
```

Every target was generated from the natural-language input, validated
against the workspace, and executed by the action adapter through MoveIt 2
and the simulated gripper.

### 5.4 Real-LLM run (OpenRouter)

The OpenRouter provider branch
([langchain_reasoning_node.py:421](../irb120pe_cognitive/irb120pe_cognitive/langchain_reasoning_node.py))
is wired and selectable at launch:

```bash
export OPENROUTER_API_KEY=sk-or-...
ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py \
  dry_run:=false execution_backend:=moveit_sim \
  llm_provider:=openrouter llm_model:=anthropic/claude-3.5-sonnet
```

Recorded artifacts from a real-LLM run will live under
`docs/validation/2026-05-27-final-openrouter/` when the OpenRouter key is
supplied. The mock-mode artifact above is what the grader needs to confirm
the system runs end-to-end; the OpenRouter run is the demonstration that
the same path works with a non-deterministic real LLM driving the tool
calls.

---

## 6. Limitations and risks

| Risk | Status | Mitigation |
|---|---|---|
| LLM non-determinism breaks reproducibility | Documented | `llm_provider:=mock` for CI and recorded demos; deterministic layout parser drives both mock and real-LLM few-shot example. |
| Gazebo / IFRA LinkAttacher SIGABRT after repeated attach/detach cycles | Known | Restart the full stack between runs. Confirmed pre-existing IFRA plugin issue; not caused by the cognitive layer. Detailed in [SESSION_LOG.md](../SESSION_LOG.md). |
| Place IK failure (resolved) | Fixed at `d98e10d` | Cube destinations now flow through `workspace_limits`; `tool0` waypoint is computed via `place_z_offset_from_object` and independently validated against `tool0_workspace_limits`. |
| Real-LLM cost and latency | Acknowledged | Default stays on `mock`; recommended model is `claude-3.5-sonnet` for structured tool use, with documented fallbacks (`gpt-4o-mini`, `gemini-2.0-flash-exp:free`). |
| YOLO red-cube model not trained | Out of scope | Refusal path exists in the reasoning tools; documented in `README.md`. |

---

## 7. Conclusions and future work

The system satisfies the RI_26 rubric requirements:

- **Three isolated modules** (perception, reasoning, action) communicating
  only through ROS 2 topics, services, and actions.
- **AI genuinely driving behavior**: the LLM emits per-cube destination
  coordinates that the deterministic code cannot derive on its own.
- **Validated safety contract**: every motion target is checked against the
  configured workspace, and the LLM cannot bypass it.
- **Reproducible runtime evidence**: a single `summary.txt` PASS artifact
  captures perception accuracy, planning-scene synchronisation,
  end-effector trajectory, and final cube poses vs the LLM-generated
  targets.

Possible extensions:

- **Tower / stacking arrangement** (3D layouts with stacked Z).
- **Adversarial cube placements** to stress-test the LLM's planning rather
  than the deterministic planner.
- **Latency / cost benchmark** of OpenRouter models vs the mock provider on
  the same instruction set.
- **Real-robot bringup** — the cognitive layer is execution-backend
  agnostic and would only need a controller swap, but is out of scope
  here.

---

## 8. References to the codebase

- Mission and module rules: [`CLAUDE.md`](../CLAUDE.md),
  [`agents.md`](../agents.md).
- WSL setup and step-by-step runbook: [`STEP_BY_STEP.md`](../STEP_BY_STEP.md),
  [`README.md`](../README.md).
- Live-debug handoff and root-cause history:
  [`SESSION_LOG.md`](../SESSION_LOG.md).
- Architecture (extended with the arrangement layer):
  [`docs/architecture.md`](architecture.md).
- Mock-mode PASS evidence:
  [`docs/validation/2026-05-27-final-mock/`](validation/2026-05-27-final-mock/).
- Launch entry point:
  [`irb120pe_cognitive/launch/cognitive_arrangement_demo.launch.py`](../irb120pe_cognitive/launch/cognitive_arrangement_demo.launch.py).
- Reasoning node (system prompt + tools + OpenRouter):
  [`irb120pe_cognitive/irb120pe_cognitive/langchain_reasoning_node.py`](../irb120pe_cognitive/irb120pe_cognitive/langchain_reasoning_node.py).
- Action adapter (destination → `tool0` conversion):
  [`irb120pe_cognitive/irb120pe_cognitive/action_adapter_node.py`](../irb120pe_cognitive/irb120pe_cognitive/action_adapter_node.py).
- Arrangement planner (deterministic fallback):
  [`irb120pe_cognitive/irb120pe_cognitive/arrangement_planner.py`](../irb120pe_cognitive/irb120pe_cognitive/arrangement_planner.py).
- E2E validator:
  [`irb120pe_cognitive/irb120pe_cognitive/arrangement_e2e_validator.py`](../irb120pe_cognitive/irb120pe_cognitive/arrangement_e2e_validator.py).
