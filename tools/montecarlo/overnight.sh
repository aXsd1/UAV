#!/usr/bin/env bash
# Long Monte Carlo set (inside WSL), in this order:
#   a) 1000 transitions, current settings, wind 0-8 m/s           (seed 4000)
#   b) 100 transitions, MC_LEGACY=1 (settings before the 2026-10-22 fixes), wind 6-8 m/s,
#      same seeds as the 6-8 m/s measurement with the new settings (seed 3000)
#   c) 50 runs of each other scenario                              (seed 5000)
# 2 parallel instances at 4x speed (3 parallel was not reliable on this PC; the speed factor does
# not change results: PX4 runs in lockstep with Gazebo). Vehicle model without the belly camera
# (sim.py), flat "default" world except urban_landing (Karabük).
# Resumable: completed runs (same scenario, seed and variant) are skipped, so after a stop it
# continues with the first unfinished run; watchdog.ps1 restarts it. A set whose runner exits
# with an error is started again (at most 5 times); if one still fails, the script exits without
# OVERNIGHT_DONE and the watchdog restarts it. OVERNIGHT_DONE only when every set completed.
#   bash tools/montecarlo/overnight.sh
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
DONE=runs/montecarlo/OVERNIGHT_DONE
rm -f "$DONE"
mkdir -p runs/montecarlo
SPEED="${SPEED:-4}"

mc() {  # mc <scenario> <n> <seed>; environment (MC_*) is passed through
  for try in 1 2 3 4 5; do
    python3 -W ignore -m tools.montecarlo.run --scenario "$1" --n "$2" --parallel 2 --speed "$SPEED" \
      --seed "$3" --resume && return 0
    echo "runner exited with an error ($1, try $try), starting again $(date -Is)"
    sleep 30
  done
  FAILED=1
}

FAILED=0
echo "start $(date -Is)"
MC_WIND_RANGE=0,8 mc transition "${N_TRANSITION:-1000}" 4000
MC_LEGACY=1 MC_WIND_RANGE=6,8 mc transition "${N_LEGACY:-100}" 3000
for s in hover_wind actuator_failure link_loss gnss_loss urban_landing; do
  mc "$s" "${N_OTHER:-50}" 5000
done
if [ "$FAILED" = 1 ]; then  # e.g. /mnt/c I/O errors: exit without DONE, the watchdog starts it again
  echo "a set did not complete, exiting for a restart $(date -Is)"
  exit 1
fi
echo "end $(date -Is)"
date -Is > "$DONE"
