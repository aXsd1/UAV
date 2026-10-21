#!/usr/bin/env bash
# Overnight Monte Carlo set (inside WSL): 1000 transitions, then 50 runs of every other
# scenario. 2 parallel instances at 2x speed (3 parallel was not reliable on this PC).
# Writes runs/montecarlo/OVERNIGHT_DONE when finished (the Windows keep-awake helper waits for it).
#   bash tools/montecarlo/overnight.sh
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
DONE=runs/montecarlo/OVERNIGHT_DONE
rm -f "$DONE"
mkdir -p runs/montecarlo
echo "start $(date -Is)"
python3 -W ignore -m tools.montecarlo.run --scenario transition --n "${N_TRANSITION:-1000}" \
  --parallel 2 --speed 2 --seed 1000
for s in hover_wind actuator_failure link_loss gnss_loss urban_landing; do
  python3 -W ignore -m tools.montecarlo.run --scenario "$s" --n "${N_OTHER:-50}" --parallel 2 --speed 2 --seed 2000
done
echo "end $(date -Is)"
date -Is > "$DONE"
