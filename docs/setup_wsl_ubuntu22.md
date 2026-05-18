# Ubuntu 22.04 WSL Setup

The target runtime is Ubuntu 22.04 on WSL 2 because ROS 2 Humble, MoveIt 2, and Gazebo Classic are most stable there.

## Windows PowerShell

```powershell
wsl --install -d Ubuntu-22.04
wsl -d Ubuntu-22.04
```

If Windows asks for a reboot, reboot and run the second command again.

## Ubuntu 22.04

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

## Workspace

```bash
mkdir -p ~/irb120_ws/src
cd ~/irb120_ws/src
git clone --branch humble https://github.com/IFRA-Cranfield/irb120_PoseEstimation.git
cd ~/irb120_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Keep the workspace under `~/irb120_ws`, not `/mnt/c` or OneDrive, for better build performance.
