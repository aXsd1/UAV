#!/usr/bin/env bash
# WSL2 Ubuntu 24.04 development environment:
#   PX4-Autopilot (+ its Gazebo Harmonic toolchain), ROS 2 Jazzy, ros_gz,
#   Micro XRCE-DDS Agent.
# Idempotent: safe to re-run; finished steps are skipped.
#
#   wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/.../UAV/tools/setup/wsl_setup.sh
#
# Builds live in the Linux filesystem ($WORK), not under /mnt/c, because
# building on the Windows drive is many times slower.
set -euo pipefail

WORK="${WORK:-$HOME/uav}"
PX4_TAG="${PX4_TAG:-}"            # empty = latest stable release tag
XRCE_TAG="${XRCE_TAG:-v2.4.3}"
export DEBIAN_FRONTEND=noninteractive

log() { printf '\n=== %s  %s\n' "$(date +%H:%M:%S)" "$*"; }
mkdir -p "$WORK"

log "base packages"
apt-get update -q
apt-get install -y -q git curl wget ca-certificates gnupg lsb-release software-properties-common \
  build-essential cmake ninja-build python3-pip python3-venv unzip nlohmann-json3-dev

# --- PX4 -----------------------------------------------------------------
if [ ! -d "$WORK/PX4-Autopilot/.git" ]; then
  log "clone PX4-Autopilot"
  git clone https://github.com/PX4/PX4-Autopilot.git "$WORK/PX4-Autopilot"
fi
cd "$WORK/PX4-Autopilot"
git fetch --tags -q
if [ -z "$PX4_TAG" ]; then
  PX4_TAG="$(git tag -l 'v1.*' | grep -Ev 'alpha|beta|rc' | sort -V | tail -1)"
fi
log "PX4 at $PX4_TAG"
git checkout -q "$PX4_TAG"
git submodule update --init --recursive -q

if [ ! -f "$WORK/.px4_deps_done" ]; then
  log "PX4 dependencies (includes Gazebo Harmonic on 24.04)"
  # --no-nuttx: no ARM toolchain yet; Cube Orange+ firmware builds come later.
  bash Tools/setup/ubuntu.sh --no-nuttx
  touch "$WORK/.px4_deps_done"
fi

# --- ROS 2 Jazzy ------------------------------------------------------------
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  log "ROS 2 apt repository"
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update -q
fi
log "ROS 2 Jazzy desktop + ros_gz + dev tools"
apt-get install -y -q ros-jazzy-desktop ros-jazzy-ros-gz ros-dev-tools

# --- Micro XRCE-DDS Agent (PX4 <-> ROS 2) -----------------------------------
if ! command -v MicroXRCEAgent >/dev/null; then
  log "Micro XRCE-DDS Agent $XRCE_TAG"
  rm -rf "$WORK/Micro-XRCE-DDS-Agent"
  git clone -q -b "$XRCE_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$WORK/Micro-XRCE-DDS-Agent"
  cmake -S "$WORK/Micro-XRCE-DDS-Agent" -B "$WORK/Micro-XRCE-DDS-Agent/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$WORK/Micro-XRCE-DDS-Agent/build" -j"$(nproc)"
  cmake --install "$WORK/Micro-XRCE-DDS-Agent/build"
  ldconfig
fi

# --- shell setup ------------------------------------------------------------
grep -q 'ros/jazzy/setup.bash' "$HOME/.bashrc" || echo 'source /opt/ros/jazzy/setup.bash' >> "$HOME/.bashrc"

# --- smoke build: stock PX4 SITL for Gazebo -----------------------------------
log "PX4 SITL build (px4_sitl)"
cd "$WORK/PX4-Autopilot"
make px4_sitl

log "versions"
echo "PX4: $(git describe --tags)"
gz sim --versions || true
set +u; . /opt/ros/jazzy/setup.bash; set -u; echo "ROS 2: $ROS_DISTRO"
MicroXRCEAgent --help 2>&1 | head -1 || true
log "done"
