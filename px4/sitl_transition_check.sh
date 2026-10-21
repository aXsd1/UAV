#!/usr/bin/env bash
# Headless transition test: take off, front transition to fixed-wing, loiter, back
# transition, land. Prints a trace and the transition metrics the design doc asks for
# (front transition < 5 s, < 5 m altitude loss).
#   px4/sitl_transition_check.sh [cruise_seconds]
#   WIND=5 px4/sitl_transition_check.sh 40
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${WORK:-$HOME/uav}"
B="$WORK/PX4-Autopilot/build/px4_sitl_default"
CRUISE="${1:-30}"
LOG=/tmp/px4_sitl.log

export GZ_SIM_SYSTEM_PLUGIN_PATH="$WORK/build/vtol_dynamics${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export HEADLESS=1 PX4_SYS_AUTOSTART=4050 PX4_SIM_MODEL=gz_vtol_p1

cleanup() { pkill -x px4 2>/dev/null; pkill -f "gz sim" 2>/dev/null; sleep 1; }
cleanup
rm -rf "$B/rootfs/eeprom" "$B/rootfs/parameters"*.bson 2>/dev/null
cd "$B/rootfs"
"$B/bin/px4" -d "$B/etc" -s etc/init.d-posix/rcS > "$LOG" 2>&1 &
PX4_PID=$!
trap cleanup EXIT
cmd() { "$B/bin/px4-$1" "${@:2}" 2>&1; }
field() { awk -v k="$1" '$1 == k":" {print $2; exit}'; }

for i in $(seq 1 90); do
  sleep 1; grep -q "Ready for takeoff" "$LOG" && break
  kill -0 $PX4_PID 2>/dev/null || { echo "PX4 exited"; tail -30 "$LOG"; exit 1; }
done
grep -q "Ready for takeoff" "$LOG" || { echo "not ready"; tail -40 "$LOG"; exit 1; }

if [ -n "${WIND:-}" ]; then
  gz topic -t /model/vtol_p1_0/wind -m gz.msgs.Vector3d -p "x: -${WIND}"
fi

T0=$(date +%s.%N)
sample() {
  local lp st as att now
  lp=$(cmd listener vehicle_local_position -n 1); st=$(cmd listener vehicle_status -n 1)
  as=$(cmd listener airspeed_validated -n 1); att=$(cmd listener vehicle_attitude -n 1)
  now=$(awk -v a="$(date +%s.%N)" -v b="$T0" 'BEGIN{printf "%.1f", a-b}')
  printf '%6s %-8s alt=%-7s vxy=%-6s tas=%-6s type=%s trans=%s q=%s\n' "$now" "$1" \
    "$(awk '$1=="z:"{printf "%.2f", -$2; exit}' <<<"$lp")" \
    "$(awk '$1=="vx:"{vx=$2} $1=="vy:"{vy=$2} END{printf "%.1f", sqrt(vx*vx+vy*vy)}' <<<"$lp")" \
    "$(field true_airspeed_m_s <<<"$as")" "$(field vehicle_type <<<"$st")" \
    "$(field in_transition_mode <<<"$st")" "$(awk '/q:/{print $2,$3,$4,$5; exit}' <<<"$att")"
}

cmd param set MIS_TAKEOFF_ALT 30 >/dev/null
cmd commander arm -f | tail -1
cmd commander takeoff | tail -1
for k in $(seq 1 12); do sleep 2; sample takeoff; done

echo "--- front transition"
cmd commander transition | tail -1
for k in $(seq 1 $((CRUISE * 2))); do sleep 0.5; sample fw; done

echo "--- back transition"
cmd commander transition | tail -1
for k in $(seq 1 20); do sleep 0.5; sample mc; done
cmd commander land | tail -1
for k in $(seq 1 25); do sleep 2; sample land; done

cleanup
echo "--- metrics"
python3 "$HERE/analyze_flight.py"

echo "--- PX4 messages"
grep -E "Transition|transition|WARN|ERROR" "$LOG" | grep -v "LED" | sort | uniq -c | sort -rn | head -20
