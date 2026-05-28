#!/usr/bin/env bash
# WSL Ubuntu 22.04 setup for the ABB IRB-120 cognitive cell.
# Pulls personal/humble, installs deps, rebuilds the cognitive stack, runs tests.
#
# Usage (inside WSL):
#   cd /mnt/c/Users/Ignacio\ González/OneDrive/Escritorio/IRB
#   chmod +x wsl_setup.sh
#   ./wsl_setup.sh                       # full setup (pull + apt + pip + build + test)
#   ./wsl_setup.sh --no-apt              # skip system package install (already done)
#   ./wsl_setup.sh --no-pull             # skip git pull
#   ./wsl_setup.sh --ws ~/my_ws          # use a different colcon workspace path
#   ./wsl_setup.sh --skip-tests          # build only, no colcon test
#
# What it does:
#   1. Pull personal/humble into the current repo folder.
#   2. Ensure rosdep + apt deps from INSTALLATION.md are installed.
#   3. Install the LangChain Python stack (no --break-system-packages on Ubuntu 22).
#   4. Symlink the repo packages into <workspace>/src and clean build/install/log.
#   5. colcon build interfaces first, then the rest.
#   6. colcon test on the cognitive packages (Layer 1 + Layer 2 from CLAUDE.md).

set -euo pipefail

# ---------- Args ----------
DO_APT=1
DO_PULL=1
DO_TESTS=1
WS="${HOME}/irb120_ws"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-apt)      DO_APT=0;     shift ;;
    --no-pull)     DO_PULL=0;    shift ;;
    --skip-tests)  DO_TESTS=0;   shift ;;
    --ws)          WS="$2";      shift 2 ;;
    -h|--help)
      sed -n '2,20p' "$0"; exit 0 ;;
    *)
      echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
done

ROS_DISTRO="${ROS_DISTRO:-humble}"

log()  { printf "\n\033[1;34m[setup]\033[0m %s\n" "$*"; }
warn() { printf "\n\033[1;33m[warn]\033[0m %s\n" "$*"; }
die()  { printf "\n\033[1;31m[error]\033[0m %s\n" "$*" >&2; exit 1; }

# ---------- 0. Sanity ----------
[[ -f "${REPO_DIR}/CLAUDE.md" ]] || die "Run this from the IRB repo root (CLAUDE.md not found in ${REPO_DIR})."
[[ -d "${REPO_DIR}/irb120pe_cognitive" ]] || die "irb120pe_cognitive missing — wrong folder."

log "Repo:       ${REPO_DIR}"
log "Workspace:  ${WS}"
log "ROS distro: ${ROS_DISTRO}"

# ---------- 1. Pull personal/humble ----------
if [[ ${DO_PULL} -eq 1 ]]; then
  log "Configuring git to treat CRLF noise as non-changes (core.autocrlf=input)…"
  cd "${REPO_DIR}"
  git config core.autocrlf input
  git config core.filemode false

  log "Stashing local CRLF noise so the pull is clean…"
  if [[ -n "$(git status --porcelain)" ]]; then
    git stash push -u -m "wsl_setup auto-stash $(date +%F_%T)" || warn "Stash failed; continuing."
  fi

  log "Fetching personal/humble…"
  git fetch personal humble
  log "Merging personal/humble into local humble (fast-forward only)…"
  git checkout humble
  git pull --ff-only personal humble || die "Pull was not fast-forward. Resolve manually (git status / git log)."
fi

# ---------- 2. apt + rosdep ----------
if [[ ${DO_APT} -eq 1 ]]; then
  log "Updating apt and installing ROS 2 ${ROS_DISTRO} sim/control packages…"
  sudo apt update
  sudo apt install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gripper-controllers \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf2-ros \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    gazebo

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initialising rosdep (first time)…"
    sudo rosdep init || true
  fi
  rosdep update
fi

# ---------- 3. Workspace layout ----------
mkdir -p "${WS}/src"
# Symlink the IRB packages into the workspace so colcon sees them
# without copying the whole tree (works whether repo is on /mnt/c or in ~).
log "Linking IRB packages into ${WS}/src …"
for pkg in irb120pe_bringup irb120pe_cognitive irb120pe_cognitive_interfaces \
           irb120pe_detection irb120pe_gazebo irb120pe_moveit2; do
  link="${WS}/src/${pkg}"
  target="${REPO_DIR}/${pkg}"
  [[ -d "${target}" ]] || { warn "Missing ${pkg}, skipping"; continue; }
  if [[ -L "${link}" || -e "${link}" ]]; then
    rm -rf "${link}"
  fi
  ln -s "${target}" "${link}"
done

# 3rd-party deps required by package.xml: ros2srrc_data, linkattacher_msgs (and the full IFRA stack)
need_clone() { [[ ! -d "${WS}/src/$1" ]]; }
clone_into_ws() {
  local url="$1" branch="${2:-}"
  cd "${WS}/src"
  if [[ -n "${branch}" ]]; then
    git clone -b "${branch}" "${url}"
  else
    git clone "${url}"
  fi
}

if need_clone "IFRA_LinkAttacher";    then log "Cloning IFRA_LinkAttacher…";    clone_into_ws https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git; fi
if need_clone "IFRA_LinkPose";        then log "Cloning IFRA_LinkPose…";        clone_into_ws https://github.com/IFRA-Cranfield/IFRA_LinkPose.git;     fi
if need_clone "IFRA_ObjectPose";      then log "Cloning IFRA_ObjectPose…";      clone_into_ws https://github.com/IFRA-Cranfield/IFRA_ObjectPose.git;   fi
if need_clone "ros2_SimRealRobotControl"; then log "Cloning ros2_SimRealRobotControl…"; clone_into_ws https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.git; fi

# ---------- 4. rosdep install for the workspace ----------
if [[ ${DO_APT} -eq 1 ]]; then
  log "Running rosdep install across ${WS}/src …"
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  cd "${WS}"
  rosdep install --from-paths src --ignore-src -r -y || warn "rosdep had warnings; continuing."
fi

# ---------- 5. Python (LangChain) deps ----------
log "Installing LangChain / LLM Python deps…"
python3 -m pip install --user --upgrade \
  langchain \
  langchain-openai \
  langchain-ollama \
  langchain-huggingface \
  ultralytics \
  pyyaml

# ---------- 6. Clean + build ----------
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${WS}"

log "Cleaning previous build/install/log to avoid stale interface artefacts…"
rm -rf build install log

log "Building interfaces package first (rebuild required when .srv files change)…"
colcon build --symlink-install --packages-select irb120pe_cognitive_interfaces

# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

log "Building the rest of the workspace…"
colcon build --symlink-install

# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

# ---------- 7. Tests ----------
if [[ ${DO_TESTS} -eq 1 ]]; then
  log "Running unit + dry-run e2e tests on the cognitive packages…"
  colcon test --packages-select irb120pe_cognitive irb120pe_cognitive_interfaces
  colcon test-result --verbose || warn "Some tests failed — inspect log/latest_test/."
fi

cat <<EOF

------------------------------------------------------------
 Setup complete.

 Next steps:
   source ${WS}/install/setup.bash
   ros2 launch irb120pe_cognitive cognitive_arrangement_demo.launch.py

 Tip: add the source line to your ~/.bashrc so every new WSL
 shell picks up the workspace automatically.
------------------------------------------------------------
EOF
