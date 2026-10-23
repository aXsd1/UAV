#!/usr/bin/env bash
# Start PX4 SITL + Gazebo with the P1 model (inside WSL).
#   px4/run_sitl.sh            # Karabük world, with the Gazebo window (WSLg)
#   HEADLESS=1 px4/run_sitl.sh # server only
#   WORLD=default px4/run_sitl.sh  # PX4's empty world (home in Zurich); any world built by tools/worldgen.py by name
set -eo pipefail
WORK="${WORK:-$HOME/uav}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$WORK/build/vtol_dynamics${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export PX4_GZ_WORLD="${WORLD:-karabuk}"
cd "$WORK/PX4-Autopilot"
exec make px4_sitl gz_vtol_p1
