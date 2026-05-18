# Cognitive IRB-120 Agent Milestones

1. Environment validation
   - Use Ubuntu 22.04 on WSL 2 for ROS 2 Humble, MoveIt 2, and Gazebo Classic.
   - Keep builds inside the WSL Linux filesystem, for example `~/irb120_ws`.

2. Baseline build and launch verification
   - Build the upstream project without cognitive additions first.
   - Verify Gazebo and MoveIt launch before running LLM nodes.

3. Interface package
   - Build `irb120pe_cognitive_interfaces`.
   - Confirm generated messages and services are visible with `ros2 interface show`.

4. Perception node
   - Run YOLO/OpenCV perception independently.
   - Confirm `vision_msgs/Detection2DArray` and structured detected objects are available.

5. Planning Scene synchronization
   - Confirm perceived cubes are added to MoveIt as collision objects.
   - Confirm stale perceived objects are removed.

6. Action adapter
   - Validate workspace limits and slot aliases.
   - Test dry-run mode before commanding `/Robmove`, `/Move`, and LinkAttacher.

7. LangChain reasoning node
   - Start with `llm_provider:=mock`.
   - Add OpenAI/Ollama/Hugging Face only after the ROS tool path is proven.

8. Launch integration
   - Use `cognitive_demo.launch.py` for the full stack.
   - Keep legacy deterministic scripts as reference demos.

9. Tests and documentation
   - Run unit tests for validation and mock reasoning.
   - Update demo notes after each experiment.
