#!/usr/bin/env bash
# Wire this repo into the PX4 checkout made by tools/setup/wsl_setup.sh (run inside WSL):
#   1. apply px4/patches/*.patch (control allocation: hover surfaces for the vanes)
#   2. generate + register the airframe and the Gazebo model
#   3. build the vtol_dynamics Gazebo plugin and its C++ vs Python test
#   4. build PX4 SITL
# Idempotent. Re-run after changing config/p1.toml (after python -m tools.aerogen).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="${WORK:-$HOME/uav}"
PX4="$WORK/PX4-Autopilot"
BUILD="$WORK/build/vtol_dynamics"
NAME=p1

log() { printf '\n=== %s\n' "$*"; }
cd "$PX4"

log "PX4 patches"
for p in "$REPO"/px4/patches/*.patch; do
  if git apply --reverse --check "$p" 2>/dev/null; then
    echo "already applied: $(basename "$p")"
  else
    git apply "$p" && echo "applied: $(basename "$p")"
  fi
done

log "airframe + Gazebo model"
cd "$REPO"
python3 -m tools.px4gen
AIRFRAME="$(basename "$REPO"/px4/airframes/4050_gz_vtol_$NAME)"
cp "$REPO/px4/airframes/$AIRFRAME" "$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/$AIRFRAME"
LIST="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
grep -q "$AIRFRAME" "$LIST" || sed -i "s/^\(\s*\)4018_gz_quadtailsitter/&\n\1$AIRFRAME/" "$LIST"
python3 -m tools.sdfgen --aero-dir "$REPO/models/aero/$NAME" --out "$PX4/Tools/simulation/gz/models/vtol_$NAME"

log "vtol_dynamics plugin"
cmake -S "$REPO/sim/gazebo/vtol_dynamics" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD"
(cd "$BUILD" && ctest --output-on-failure)

log "PX4 SITL"
cd "$PX4"
make px4_sitl
log "done"
