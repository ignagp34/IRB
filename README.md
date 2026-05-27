<div id="top"></div>

<!-- 

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso         - Mikel.Bueno-Viso@cranfield.ac.uk                 #
#           Irene Bernardino Sanchez - i.bernardinosanchez.854@cranfield.ac.uk          #
#           Seemal Asif              - s.asif@cranfield.ac.uk                           #
#           Phil Webb                - p.f.webb@cranfield.ac.uk                         #
#                                                                                       #
#  Date: November, 2023.                                                                #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023). Object Detection and Pose Estimation within a Robot Cell. URL: https://github.com/IFRA-Cranfield/irb120_PoseEstimation

-->

<!--

  README.md TEMPLATE obtined from:
      https://github.com/othneildrew/Best-README-Template
      AUTHOR: OTHNEIL DREW 

-->

<!-- HEADER -->
<br />
<div align="center">
  <a>
    <img src="media/header.jpg" alt="header" width="680" height="190">
  </a>

  <br />

  <h2 align="center">Object Detection and Pose Estimation within a Robot Cell</h2>

  <h3 align="center">ABB IRB-120 Robot - ROS2 Humble</h3>

  <p align="center">
    IFRA (Intelligent Flexible Robotics and Assembly) Group
    <br />
    Centre for Robotics and Assembly
    <br />
    Cranfield University
  </p>
</div>

<br />
<br />

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about">About</a>
      <ul>
        <li><a href="#intelligent-flexible-robotics-and-assembly-group">IFRA Group</a></li>
        <li><a href="#irb120_poseestimation-repository">irb120_PoseEstimation Repository</a></li>
      </ul>
    </li>
    <li>
      <a href="#installation">Installation</a>
      <ul>
        <li><a href="#ros2-humble-setup">ROS2 Humble Setup</a></li>
        <li><a href="#yolov8-and-opencv">YOLOv8 and OpenCV</a></li>
        <li><a href="#irb120_poseestimation-repository">irb120_PoseEstimation Repository</a></li>
      </ul>
    </li>
    <li>
      <a href="#ros2-packages">ROS2 Packages</a>
      <ul>
        <li><a href="#irb120pe_gazebo">irb120pe_gazebo</a></li>
        <li><a href="#irb120pe_moveit2">irb120pe_moveit2</a></li>
        <li><a href="#irb120pe_bringup">irb120pe_bringup</a></li>
        <li><a href="#irb120pe_detection">irb120pe_detection</a></li>
      </ul>
    </li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#cite-our-work">Cite our work</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<br />

<!-- ABOUT THE PROJECT -->
## About

### Intelligent Flexible Robotics and Assembly Group

The IFRA (Intelligent Flexible Robotics and Assembly) Group is part of the Centre for Robotics and Assembly at Cranfield University.

IFRA Group pushes technical boundaries. At IFRA we provide high tech automation & assembly solutions, and we support smart manufacturing with Smart Industry technologies and solutions. Flexible Manufacturing Systems (FMS) are a clear example. They can improve overall operations and throughput quality by adapting to real-time changes and situations, supporting and pushing the transition towards flexible, intelligent and responsive automation, which we continuously seek and support.

The IFRA Group undertakes innovative research to design, create and improve Intelligent, Responsive and Flexible automation & assembly solutions, and this series of GitHub repositories provide background information and resources of how these developments are supported.

__SOCIAL MEDIA__:

IFRA-Cranfield:
- YouTube: https://www.youtube.com/@IFRACranfield
- LinkedIn: https://www.linkedin.com/in/ifra-cranfield/

Centre for Robotics and Assembly:
- Instagram: https://www.instagram.com/cranfieldrobotics/
- Facebook: https://www.facebook.com/cranfieldunirobotics/
- YouTube: https://www.youtube.com/@CranfieldRobotics
- LinkedIn: https://www.linkedin.com/company/cranfieldrobotics/
- Website: https://www.cranfield.ac.uk/centres/centre-for-robotics-and-assembly 

### irb120_PoseEstimation Repository

Welcome to the irb120_PoseEstimation repository! This repository contains a comprehensive solution for executing a cube pick-and-place task with the ABB IRB-120 industrial robot and the Schunk EGP-64 parallel gripper. The ROS 2 packages provided here enable seamless execution in both simulated and real environments. Leveraging ROS 2 Humble on an Ubuntu 22.04 PC, this repository furnishes all necessary source code, ensuring compatibility and ease of deployment. 

To enhance the perception capabilities of the ABB IRB120 robot for the application, a combination of YOLOv8, OpenCV, and ROS 2 has been employed. This integration empowers the system with advanced object detection and recognition capabilities, allowing for efficient and reliable execution of the cube pick-and-place task.

__Cube Pick-and-Place Task:__

The system, equipped with a standard web camera, adeptly detects the randomly spawned cube within the workspace and proceeds to classify it based on its distinct colored feature. The intricate task of manipulating the cube involves precise pose estimation, ensuring its placement within the designated tray and alignment with the corresponding slot, with the identified feature positioned upwards.

Explore the packages and get started with your cube pick-and-place tasks today!

__Video Demonstration: Program Execution (Simulation + Real Robot)__

[![Alt text](https://img.youtube.com/vi/1TVL727UhDk/0.jpg)](https://www.youtube.com/watch?v=1TVL727UhDk)

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- INSTALLATION -->
## Installation

The installation and execution of the irb120_PoseEstimation ROS2 packages requires the previous installation and set-up of the following components:
- Ubuntu 22.04 machine with ROS2 Humble.
- ROS2 Packages for the Simulation and Control of Robots using Gazebo and MoveIt!2.
- YOLOv8 for the object detection feature.
- OpenCV for the Image Processing feature.

### ROS2 Humble Setup

The packages used to run the Simulation and Control of the ABB IRB-120 Robot in irb120_PoseEstimation are based on the [IFRA-Cranfield/ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl) repository. Therefore, it is recommended to follow the installation steps defined in that repository in order to properly set-up a ROS2 Humble machine for Robot Simulation and Control. To facilitate your work, those steps have been summarized and outlined in the INSTALLATION.md document [here](https://github.com/IFRA-Cranfield/irb120_PoseEstimation/blob/main/INSTALLATION.md).

### YOLOv8 and OpenCV

The latest version (v8) of [YOLO (You-Only-Look-Once)](https://github.com/ultralytics/ultralytics) can be installed using the following pip command, which installs YOLOv8 along with the requirements for a Python>=3.8 environment with PyTorch>=1.8:
```sh
pip install ultralytics
```

[OpenCV-Python](https://docs.opencv.org/3.4/d2/de6/tutorial_py_setup_in_ubuntu.html) is needed for this repo, and it can be installed with the following command:
```sh
sudo apt-get install python3-opencv
```

### irb120_PoseEstimation Repository

Once the ROS2 Humble environment has been properly set up, and all the required packages have been installed, the ROS2 Packages of the irb120_PoseEstimation repository can be installed by executing the following commands in the terminal:
```sh
cd ~/dev_ws/src
git clone https://github.com/IFRA-Cranfield/irb120_PoseEstimation.git
cd ~/dev_ws
colcon build
```

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- ROS 2 Packages: Explanation -->
## ROS2 Packages 

Within this repository, you'll find a comprehensive integration of ROS 2 Robot Simulation and Control packages, including Gazebo, MoveIt!2, and Robot Bringup packages, meticulously combined with a custom ROS 2 package housing the YOLOv8 and OpenCV modules, aptly named the detection package. This amalgamation provides a robust foundation for executing the cube pick-and-place task seamlessly across both simulated and real environments. The ROS 2 Robot Simulation and Control packages facilitate simulation and control of the ABB IRB-120 industrial robot, while the custom detection package empowers the system with advanced object detection and recognition capabilities, essential for precise manipulation and pose estimation of the target cube. Explore these packages to delve into the intricacies of robotic perception and control, accelerating the development and deployment of robotic applications.

### irb120pe_gazebo

The Gazebo package within this repository is a pivotal component, encompassing crucial data related to the robot's representation in the Unified Robot Description Format (.urdf). Beyond merely describing the robot's physical structure, this package also houses essential parameters necessary for fine-tuning ROS 2 controllers, ensuring precise control over the ABB IRB-120 industrial robot's movements. Furthermore, it includes detailed CAD files offering insight into the robot's design, alongside pertinent information concerning the Gazebo environment. This comprehensive package serves as the cornerstone for simulating and accurately replicating the robot's behavior within the simulated environment, facilitating seamless development and testing of robotic applications.

You can access all the Gazebo ROS 2 Package source code [here](https://github.com/IFRA-Cranfield/irb120_PoseEstimation/tree/main/irb120pe_gazebo).

### irb120pe_moveit2

The MoveIt!2 package within this repository is essential for motion control of the ABB IRB-120 industrial robot. It serves as a foundational element for motion planning and execution, offering comprehensive information required by the MoveIt!2 tool. This package includes crucial data such as robot specifications, obtained from the Gazebo package, ensuring consistency and accuracy in robot representation. Moreover, it encapsulates controller parameters necessary for fine-tuning and optimizing the robot's movements, enhancing its precision and efficiency. Additionally, the package provides a seamless Motion Planning interface, empowering users to effortlessly generate and execute motion plans, thereby streamlining the development and deployment of complex robotic applications.

You can access all the MoveIt!2 ROS 2 Package source code [here](https://github.com/IFRA-Cranfield/irb120_PoseEstimation/tree/main/irb120pe_moveit2).

### irb120pe_bringup

The Robot Bringup package within this repository acts as the main link between the ROS 2 system and the real robot arm. It facilitates seamless integration of the robot arm control with ROS 2, requiring the presence of a robust ROS 2 driver. This driver operates concurrently within both the robot controller and ROS 2 environment, enabling comprehensive control over the robot's motion and state through various ROS 2 nodes. By establishing this vital connection, the Robot Bringup package enables seamless communication and coordination between the ROS 2 system and the real robot arm, laying the groundwork for efficient and reliable execution of robotic tasks in real-world environments.

You can access all the Robot Bringup ROS 2 Package source code [here](https://github.com/IFRA-Cranfield/irb120_PoseEstimation/tree/main/irb120pe_bringup).

### irb120pe_detection

The detection package within this repository encompasses essential functionalities crucial for effective cube detection, classification, pose estimation, and pick-and-place tasks. It incorporates the YOLOv8 module for robust cube detection and classification, enabling the system to accurately identify the target object within the workspace. Additionally, the package integrates the OpenCV module for precise 6DoF Pose Estimation of the cube, ensuring accurate placement and alignment during manipulation tasks. Furthermore, the robot and gripper modules housed within this package contain ROS 2 Action Clients responsible for communicating with MoveIt!2, facilitating seamless operation of the robot movements. Lastly, the detection package encompasses the entire program logic for the application, orchestrating the coordination and interaction of various modules for efficient cube detection, pose estimation, and pick-and-place tasks, thereby enhancing the overall functionality and performance of the system.

You can access all the Robot Bringup ROS 2 Package source code [here](https://github.com/IFRA-Cranfield/irb120_PoseEstimation/tree/main/irb120pe_detection).

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**. If you have a suggestion that would make this better, or you find a solution to any of the issues/improvements presented above, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks you very much!

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- LICENSE -->
## License

<p>
  Intelligent Flexible Robotics and Assembly Group
  <br />
  Created on behalf of the IFRA Group at Cranfield University, United Kingdom
  <br />
  E-mail: IFRA@cranfield.ac.uk 
  <br />
  <br />
  Licensed under the Apache-2.0 License.
  <br />
  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
  <br />
  <br />
  <a href="https://www.cranfield.ac.uk/">Cranfield University</a>
  <br />
  School of Aerospace, Transport and Manufacturing (SATM)
  <br />
    <a href="https://www.cranfield.ac.uk/centres/centre-for-robotics-and-assembly">Centre for Robotics and Assembly</a>
  <br />
  College Road, Cranfield
  <br />
  MK43 0AL, Bedfordshire, UK
  <br />
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CITE OUR WORK -->
## Cite our work

<p>
  You can cite our work with the following statement:
  <br />
  IFRA-Cranfield (2023). Object Detection and Pose Estimation within a Robot Cell. URL: https://github.com/IFRA-Cranfield/irb120_PoseEstimation
  <br />
  <br />
  Reference to CONFERENCE PAPER (if published) to be added here.
  <br />
  Submitted to <a href="https://2024.ieeecase.org/">IEEE-CASE2024</a> conference.
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

<p>
  Mikel Bueno Viso - Research Assistant in Intelligent Automation at Cranfield University
  <br />
  E-mail: Mikel.Bueno-Viso@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/mikel-bueno-viso/
  <br />
  Profile: https://www.cranfield.ac.uk/people/mikel-bueno-viso-32884399
  <br />
  <br />
  Irene Bernardino Sanchez - Robotics MSc Student
  <br />
  E-mail: i.bernardinosanchez.854@cranfield.ac.uk 
  <br />
  LinkedIn: https://www.linkedin.com/in/irene-bernardino-sanchez-08bbb1195/
  <br />
  <br />
  Dr. Seemal Asif - Lecturer in Artificial Intelligence and Robotics at Cranfield University
  <br />
  E-mail: s.asif@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/dr-seemal-asif-ceng-fhea-miet-9370515a/
  <br />
  Profile: https://www.cranfield.ac.uk/people/dr-seemal-asif-695915
  <br />
  <br />
  Professor Phil Webb - Professor of Aero-Structure Design and Assembly at Cranfield University
  <br />
  E-mail: p.f.webb@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/phil-webb-64283223/
  <br />
  Profile: https://www.cranfield.ac.uk/people/professor-phil-webb-746415 
  <br />
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [README.md template - Othneil Drew](https://github.com/othneildrew/Best-README-Template).
* [ROS 2 Documentation - Humble](https://docs.ros.org/en/humble/index.html).
* [PicNik Robotics - MoveIt!2 Documentation](https://moveit.picknik.ai/humble/index.html).
* [ABB - ROS Repositories](http://wiki.ros.org/abb).
* [ABB - ROS 2 Driver (PickNik Robotics)](https://github.com/PickNikRobotics/abb_ros2).
* [YOLOv8 - GitHub Repository](https://github.com/ultralytics/ultralytics)
* [OpenCV - Computer Vision Library](https://opencv.org/)

<p align="right">(<a href="#top">back to top</a>)</p>

---

# Cognitive Industrial Sorting Cobot Extension

This repository now includes a cognitive ROS 2 extension for the project **Cognitive Industrial Sorting Cobot - ABB IRB-120 Vision-Action Loop**. The original deterministic IFRA-Cranfield demos are preserved, and the new packages add a separated perception, planning-scene, action-adapter, and LangChain reasoning loop.

## Resumen En Espanol: Instalacion Y Preparacion

Esta extension esta preparada para una demo **solo en Gazebo** con ROS 2 Humble, MoveIt 2 y Gazebo Classic dentro de WSL 2 Ubuntu 22.04. La ruta validada no depende de `/Robmove` ni de `/Move`; usa `execution_backend:=moveit_sim`, MoveIt `/move_action`, los controladores simulados del gripper y los servicios Gazebo de IFRA LinkAttacher.

Workspace recomendado:

```bash
mkdir -p ~/irb120_ws/src
cd ~/irb120_ws/src
```

Instala la base de ROS 2 Humble, MoveIt 2, Gazebo y herramientas de compilacion:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-opencv \
  python3-pip
```

Instala dependencias Python usadas por percepcion y razonamiento:

```bash
pip install ultralytics langchain langchain-openai langchain-ollama langchain-huggingface
```

Clona o copia este repositorio en `~/irb120_ws/src/irb120_PoseEstimation`. Asegura tambien las dependencias fuente usadas por el proyecto, especialmente `IFRA_LinkAttacher` y `ros2_SimRealRobotControl`, dentro de `~/irb120_ws/src`.

Compila la ruta validada saltando `ros2srrc_execution`:

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip ros2srrc_execution
source install/setup.bash
```

Ejecuta las pruebas principales:

```bash
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

Resultado validado actual: `28 tests, 0 errors, 0 failures`.

## Resumen En Espanol: Flujo Completo De La Demo Gazebo

1. Abre una terminal WSL y prepara el entorno:

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

2. Lanza Gazebo, MoveIt, percepcion, sincronizacion de Planning Scene, action adapter y razonamiento mock:

```bash
ros2 launch irb120pe_cognitive cognitive_demo.launch.py \
  dry_run:=false llm_provider:=mock execution_backend:=moveit_sim \
  spawn_timeout:=120.0 start_legacy_interfaces:=false rviz_file:=True
```

3. En otra terminal WSL, prepara el entorno de nuevo:

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

4. Comprueba que los servicios y acciones principales estan disponibles:

```bash
ros2 service list | grep -E "/ATTACHLINK|/DETACHLINK|/irb120pe/perception/get_detected_objects|/irb120pe/reasoning/execute_instruction"
ros2 action list -t | grep -E "/move_action|gripper_cmd|/irb120_controller/follow_joint_trajectory"
```

5. Genera un cubo azul visible en Gazebo:

```bash
ros2 run irb120pe_cognitive gazebo_cube_helper spawn \
  --cube BlueCube --name BlueCube --x 0.55 --y 0.52 --z 0.88 --replace
```

6. Comprueba percepcion estructurada:

```bash
ros2 service call /irb120pe/perception/get_detected_objects \
  irb120pe_cognitive_interfaces/srv/GetDetectedObjects "{}"
```

7. Ejecuta una instruccion en lenguaje natural:

```bash
ros2 service call /irb120pe/reasoning/execute_instruction \
  irb120pe_cognitive_interfaces/srv/ExecuteInstruction \
  "{instruction: 'Pick the blue cube and place it in the right container'}"
```

Respuesta esperada:

```text
success=True
status=moveit_sim pick-and-place completed for BlueCube.
```

8. Limpia el cubo al terminar:

```bash
ros2 run irb120pe_cognitive gazebo_cube_helper delete --name BlueCube
```

9. Para repetir la validacion automatizada, usa la guia:

```text
docs/gazebo_pick_place_moveit_sim.md
```

Evidencia validada:

```text
docs/validation/2026-05-19-gazebo-pick-place/
```

## Architecture

```text
Natural language instruction
-> /irb120pe/reasoning/execute_instruction
-> LangChain/mock reasoning tools
-> /irb120pe/perception/get_detected_objects
-> slot and workspace validation
-> /irb120pe/action/pick_and_place or /irb120pe/action/move_arm
-> /Robmove, /Move, LinkAttacher, MoveIt 2, Gazebo
-> final status and tool trace
```

The LLM is constrained to strict tools. It cannot execute arbitrary code and cannot directly command robot topics.

## Added Packages

- `irb120pe_cognitive_interfaces`: ROS 2 messages and services for detected objects, natural-language instructions, arm motion, and pick-and-place.
- `irb120pe_cognitive`: Python ROS 2 nodes for perception publishing, Planning Scene synchronization, validated robot action tools, and LangChain reasoning.

Legacy deterministic scripts remain available:

```bash
ros2 run irb120pe_detection main.py
ros2 run irb120pe_detection main_Gz.py
ros2 run irb120pe_detection main_GzSimplified.py
```

## Recommended Environment

Use **Ubuntu 22.04 on WSL 2** for ROS 2 Humble, MoveIt 2, and Gazebo Classic. Keep the ROS workspace inside the WSL Linux filesystem:

```bash
mkdir -p ~/irb120_ws/src
cd ~/irb120_ws/src
git clone --branch humble https://github.com/IFRA-Cranfield/irb120_PoseEstimation.git
cd ~/irb120_ws
```

Detailed WSL setup is in [`docs/setup_wsl_ubuntu22.md`](docs/setup_wsl_ubuntu22.md).

## Dependencies

Install ROS 2 Humble desktop, MoveIt 2, Gazebo ROS packages, `colcon`, `rosdep`, `vcstool`, OpenCV/cv_bridge, `vision_msgs`, `tf2_ros`, and the existing IFRA dependencies used by this project (`ros2srrc_*`, `objectpose_msgs`, `linkpose_msgs`, `linkattacher_msgs`, ABB support packages).

Python LLM dependencies depend on provider:

```bash
pip install langchain langchain-openai langchain-ollama langchain-huggingface ultralytics
```

The default reasoning provider is `mock`, so the cognitive stack can be demonstrated without API keys.

## Build

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
# Base Gazebo + MoveIt
ros2 launch irb120pe_moveit2 moveit2.launch.py

# Perception only
ros2 launch irb120pe_cognitive cognitive_perception.launch.py

# Planning Scene sync only
ros2 launch irb120pe_cognitive planning_scene.launch.py

# Reasoning and action adapter
ros2 launch irb120pe_cognitive reasoning.launch.py

# Full cognitive demo
ros2 launch irb120pe_cognitive cognitive_demo.launch.py
```

## Run A Natural-Language Command

```bash
ros2 service call /irb120pe/reasoning/execute_instruction irb120pe_cognitive_interfaces/srv/ExecuteInstruction "{instruction: 'Pick the blue cube and place it in the left container'}"
```

More examples:

- `Classify the black cube into slot B`
- `Pick the white cube and place it in slot A`
- `Sort all visible cubes by color`

## Live Gazebo Cube Stimulus

The proven live camera path uses Gazebo `/spawn_entity` with xacro-expanded cube XML, not raw `spawn_entity.py -file BlueCube.urdf`.

```bash
ros2 run irb120pe_cognitive gazebo_cube_helper spawn \
  --cube BlueCube --name BlueCube \
  --x 0.55 --y 0.52 --z 0.88 --qw 1.0 \
  --replace
```

The full dry-run/mock rerun sequence is documented in [`docs/live_gazebo_cube_demo.md`](docs/live_gazebo_cube_demo.md).

## LLM Provider Configuration

Copy `.env.example` and set the provider-specific variables. Never hardcode API keys.

```bash
# OpenAI
export OPENAI_API_KEY=...
ros2 launch irb120pe_cognitive reasoning.launch.py llm_provider:=openai

# Ollama
ros2 launch irb120pe_cognitive reasoning.launch.py llm_provider:=ollama llm_model:=llama3.1

# OpenRouter (recommended for the demo video)
export OPENROUTER_API_KEY=sk-or-...
pip install langchain-openai
ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py \
  llm_provider:=openrouter llm_model:=anthropic/claude-3.5-sonnet
```

OpenRouter is OpenAI-API-compatible, so the same `langchain-openai` client is reused with a custom `base_url`. Recommended models:

- `anthropic/claude-3.5-sonnet` — strongest structured tool-use; first choice for the live demo.
- `openai/gpt-4o-mini` — cheaper alternative with reliable tool-use.
- `google/gemini-2.0-flash-exp:free` — free tier for development experiments.

The default in `cognitive.yaml` stays `llm_provider: mock` so tests and recorded demos remain deterministic; OpenRouter is opt-in via the launch arg + environment variable.

## Safety

Movement tools validate numeric coordinates, configured workspace bounds, valid slots, and available detected objects. Unsafe, incomplete, ambiguous, or unsupported commands are rejected with explicit errors. Red cube commands are unsupported until the simulation assets and YOLO model include a red class.

## Goal-Oriented Arrangement Demo (RI_26 Cognitive Mission)

This is the headline demo for the RI_26 final project. The LLM is not optional: it receives a natural-language instruction like *"Arrange the cubes in a line by color from white to black to blue along Y at x=0.55, z=1.00, spacing=0.06"*, queries perception, and emits per-cube target poses (coordinates the deterministic code cannot derive on its own). Each pose is validated against the safe workspace before MoveIt executes it.

New ROS interfaces:

- Service `/irb120pe/reasoning/arrange_objects` (`irb120pe_cognitive_interfaces/srv/ArrangeObjects`) — top-level entry point. Returns `plan_json` with the ordered (object, target_pose) plan.
- Topic `/irb120pe/reasoning/trace` (`std_msgs/String`, JSON) — one message per LLM tool call. Watch live during the demo with `ros2 topic echo /irb120pe/reasoning/trace`.
- Extended service `/irb120pe/action/pick_and_place` now honors an optional `target_pose` field, which the LangChain `move_object_to_pose_tool` populates.

### Launch the full stack

```bash
ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py \
  dry_run:=false llm_provider:=mock execution_backend:=moveit_sim
```

### Trigger the mission

The deterministic demo script spawns the three cubes and calls the new service:

```bash
ros2 run irb120pe_cognitive arrangement_demo
# or with a custom instruction:
ros2 run irb120pe_cognitive arrangement_demo \
  --instruction "Build a tower at x=0.55, y=0.52, z=1.00"
```

### Live end-to-end validation (runtime evidence)

The validator below runs against a live launched stack, spawns cubes, calls the reasoning service, records the end-effector and cube trajectories, then verifies the cubes ended within tolerance of the LLM-generated targets. Every artifact is written to `./evidence/arrangement/`.

```bash
ros2 run irb120pe_cognitive arrangement_e2e_validator \
  --output-dir ./evidence/arrangement
cat ./evidence/arrangement/summary.txt   # must be PASS
```

Produced evidence files:

- `spawn_<Cube>.txt` — Gazebo spawn responses
- `detections_initial.txt` — perception output with measured xyz and delta vs spawn pose
- `planning_scene_initial.txt` — collision object IDs in MoveIt at start
- `plan_json.txt` — the LLM's ordered arrangement plan
- `reasoning_response.txt` — service success + status
- `tool0_trajectory.csv` — end-effector pose over time (from TF)
- `cube_trajectory.csv` — every cube's position over time (from `/gazebo/model_states`)
- `final_state.txt` — final cube poses vs planned targets and per-cube delta
- `planning_scene_final.txt` — collision objects after the arrangement
- `summary.txt` — PASS/FAIL per check (the single artifact the grader needs)

## Tests

```bash
cd ~/irb120_ws
colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
colcon test-result --verbose
```

Three layers of automated coverage:

1. **Unit tests** (pure Python, no ROS): workspace bounds, slot resolution, object selection, and the new arrangement planner (`test_arrangement_planner.py`, `test_target_pose_validation.py`).
2. **ROS dry-run e2e** (`test/test_arrangement_e2e_dry_run.py`): boots a `FakePerceptionNode` + `FakeActionAdapter` + the real `LangChainReasoningNode`, calls `/irb120pe/reasoning/arrange_objects`, asserts the returned plan and the per-step service calls.
3. **Live runtime e2e** — see *Live end-to-end validation* above; this is the deliverable that exercises perception, planning scene, MoveIt, gripper, and the LLM together and writes per-check PASS/FAIL evidence.

## Troubleshooting

- If `Ubuntu-22.04` is missing, install it with `wsl --install -d Ubuntu-22.04`.
- If Gazebo GUI fails in WSL, confirm WSLg is enabled and test with a simple GUI app.
- If YOLO does not load, check `yolo_model_path` in `irb120pe_cognitive/config/cognitive.yaml`.
- If the reasoning node reports no detections, launch perception first and call `/irb120pe/perception/get_detected_objects`.
- If movement is rejected, inspect workspace limits and slot coordinates in `config/cognitive.yaml`.
- If a real LLM provider fails, switch back to `llm_provider:=mock` to validate the ROS tool path.
