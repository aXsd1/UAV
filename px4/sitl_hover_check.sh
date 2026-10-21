#!/usr/bin/env bash
# Headless smoke test: start PX4 SITL + Gazebo with the P1 model, take off to
# MIS_TAKEOFF_ALT, hold, land. Prints a position/attitude trace and a verdict.
#   px4/sitl_hover_check.sh [hold_seconds]
#   WIND=8 px4/sitl_hover_check.sh 30          # wind (m/s, from the east) once at altitude
#   WIND=8 WIND_RAMP=3 px4/sitl_hover_check.sh 30  # ...ramped up over 3 s instead of a step
set -uo pipefail
WORK="${WORK:-$HOME/uav}"
PX4="$WORK/PX4-Autopilot"
B="$PX4/build/px4_sitl_default"
HOLD="${1:-20}"
LOG=/tmp/px4_sitl.log

export GZ_SIM_SYSTEM_PLUGIN_PATH="$WORK/build/vtol_dynamics${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export HEADLESS=1 PX4_SYS_AUTOSTART=4050 PX4_SIM_MODEL=gz_vtol_p1 PX4_GZ_WORLD="${WORLD:-default}"

cleanup() {
  pkill -x px4 2>/dev/null; pkill -f "gz sim" 2>/dev/null; sleep 1
  # a killed PX4 leaves its lock/socket behind and the next start exits with "already running"
  pgrep -x px4 >/dev/null || rm -f /tmp/px4_lock-* /tmp/px4-sock-*
}
cleanup
rm -rf "$B/rootfs/eeprom" "$B/rootfs/parameters"*.bson 2>/dev/null
cd "$B/rootfs"
"$B/bin/px4" -d "$B/etc" -s etc/init.d-posix/rcS > "$LOG" 2>&1 &
PX4_PID=$!
trap cleanup EXIT
cmd() { "$B/bin/px4-$1" "${@:2}" 2>&1; }

echo "waiting for PX4 + EKF..."
for i in $(seq 1 90); do
  sleep 1
  grep -q "Ready for takeoff" "$LOG" && break
  if ! kill -0 $PX4_PID 2>/dev/null; then echo "PX4 exited"; tail -30 "$LOG"; exit 1; fi
done
grep -q "Ready for takeoff" "$LOG" || { echo "not ready after 90 s"; tail -40 "$LOG"; exit 1; }
echo "ready after ${i}s"

sample() {
  local lp att
  lp=$(cmd listener vehicle_local_position -n 1)
  att=$(cmd listener vehicle_attitude -n 1)
  printf '%6s  x=%-8s y=%-8s z=%-8s vz=%-8s q=%s\n' "$1" \
    "$(awk '/^ *x:/{print $2; exit}' <<<"$lp")" "$(awk '/^ *y:/{print $2; exit}' <<<"$lp")" \
    "$(awk '/^ *z:/{print $2; exit}' <<<"$lp")" "$(awk '/^ *vz:/{print $2; exit}' <<<"$lp")" \
    "$(awk '/q:/{print $2,$3,$4,$5; exit}' <<<"$att")"
}

cmd param set MIS_TAKEOFF_ALT 10 >/dev/null
sample "t=0"
cmd commander arm -f | tail -1
cmd commander takeoff | tail -1
for t in $(seq 2 2 $((HOLD + 16))); do
  sleep 2; sample "t=$t"
  if [ "$t" = 16 ] && [ -n "${WIND:-}" ]; then
    RAMP="${WIND_RAMP:-0}"
    ( steps=$(( RAMP > 0 ? RAMP * 4 : 1 ))
      for k in $(seq 1 $steps); do
        gz topic -t /model/vtol_p1_0/wind -m gz.msgs.Vector3d -p "x: -$(awk "BEGIN{print ${WIND}*$k/$steps}")"
        sleep 0.25
      done ) &
    echo "  wind ${WIND} m/s on (ramp ${RAMP} s)"
  fi
done
cmd commander land | tail -1
for t in $(seq 2 2 20); do sleep 2; sample "land+$t"; done

echo "--- result (Gazebo ground truth) ---"
pose=$(gz model -m vtol_p1_0 -p 2>/dev/null | tail -2)
echo "$pose"
# Final tilt from the last vehicle_attitude quaternion: tilt = 2 asin(sqrt(qx^2 + qy^2)).
q=$(cmd listener vehicle_attitude -n 1 | awk '/q:/{gsub(/[\[\],]/,""); print $2,$3,$4,$5; exit}')
tilt=$(awk -v q="$q" 'BEGIN{split(q,a," "); printf "%.1f", 2*atan2(sqrt(a[2]^2+a[3]^2), sqrt(1-a[2]^2-a[3]^2))*57.2958}')
echo "final tilt ${tilt} deg"
landed=$(cmd listener vehicle_land_detected -n 1 | awk '$1=="landed:"{print $2; exit}')
gz_z=$(awk '{gsub(/[\[\]]/,""); print $3}' <<<"$(head -1 <<<"$pose")")
echo "land detector: landed=${landed}, Gazebo height ${gz_z} m (spawn ~0.86)"
if ! awk -v t="$tilt" 'BEGIN{exit !(t < 30)}'; then echo "VERDICT: TIPPED OVER"
elif [ "$landed" != "True" ] || ! awk -v z="$gz_z" 'BEGIN{exit !(z < 1.8)}'; then echo "VERDICT: NOT LANDED"
else echo "VERDICT: landed upright"; fi

echo "--- warnings/errors from PX4 ---"
grep -E "WARN|ERROR|fail" "$LOG" | sort | uniq -c | sort -rn | head -20
