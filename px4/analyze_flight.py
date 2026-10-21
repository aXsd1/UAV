"""Transition and landing metrics from the newest PX4 SITL log (run inside WSL).

python3 px4/analyze_flight.py [log.ulg]

Front transition: from the transition command (in_transition_mode rises) until
the vehicle is fixed-wing; altitude loss = start altitude - lowest altitude
until 5 s after it. Targets from the simulation doc: < 5 s, < 5 m.
"""

import glob
import os
import sys

import numpy as np
from pyulog import ULog

TARGET_TIME_S = 5.0
TARGET_ALT_LOSS_M = 5.0


def main() -> int:
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        logs = glob.glob(os.path.expanduser("~/uav/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg"))
        path = max(logs, key=os.path.getmtime)
    u = ULog(path)
    t0 = u.start_timestamp

    def topic(name):
        return next(x.data for x in u.data_list if x.name == name and x.multi_id == 0)

    def t(x):
        # int64: a sample stamped before the log start would wrap around in uint64
        return (x["timestamp"].astype(np.int64) - np.int64(t0)) / 1e6

    st, lp, att, ld = topic("vehicle_status"), topic("vehicle_local_position"), topic("vehicle_attitude"), \
        topic("vehicle_land_detected")
    alt = -lp["z"]
    trans = st["in_transition_mode"].astype(bool)
    fw = st["vehicle_type"] == 2
    ok = True
    print(f"log: {path}")

    starts = np.where(np.diff(trans.astype(int)) == 1)[0] + 1
    ends = np.where(np.diff(trans.astype(int)) == -1)[0] + 1
    for k, s in enumerate(starts):
        e = ends[ends > s]
        if not len(e):
            print("transition never finished")
            ok = False
            continue
        ts_, te = t(st)[s], t(st)[e[0]]
        front = fw[e[0]]
        a0 = np.interp(ts_, t(lp), alt)
        window = (t(lp) >= ts_) & (t(lp) <= te + 5.0)
        loss = a0 - alt[window].min()
        dur = te - ts_
        name = "front" if front else "back"
        passed = (dur < TARGET_TIME_S and loss < TARGET_ALT_LOSS_M) if front else loss < TARGET_ALT_LOSS_M
        ok &= passed
        print(f"{name:5s} transition: {dur:4.1f} s, altitude loss {loss:+5.2f} m  -> {'PASS' if passed else 'FAIL'}")

    if fw.any():
        fw_t = t(st)[fw]
        win = (t(lp) > fw_t[0] + 5) & (t(lp) < fw_t[-1])
        if win.any():
            print(f"fixed-wing altitude band: {alt[win].min():.1f} .. {alt[win].max():.1f} m")

    tilt = np.degrees(2 * np.arcsin(np.sqrt(att["q[1]"] ** 2 + att["q[2]"] ** 2)))
    landed = bool(ld["landed"][-1])
    upright = tilt[-1] < 30.0
    ok &= landed and upright
    print(f"landing: landed={landed}, final tilt {tilt[-1]:.1f} deg -> {'PASS' if landed and upright else 'FAIL'}")
    print("VERDICT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
