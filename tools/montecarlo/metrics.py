"""Scenario metrics from a PX4 ULog (times are PX4 boot time in seconds, as in the log)."""

from __future__ import annotations

import numpy as np
from pyulog import ULog

NAV_RTL = 5
NAV_LAND = 18
NAV_DESCEND = 12


class Flight:
    def __init__(self, path):
        self.u = ULog(str(path))

    def topic(self, name, instance=0):
        for d in self.u.data_list:
            if d.name == name and d.multi_id == instance:
                return d.data
        return None

    @staticmethod
    def t(data):
        return data["timestamp"].astype(np.int64) / 1e6

    def messages(self):
        return [m.message for m in self.u.logged_messages]


def transitions(f: Flight):
    st, lp = f.topic("vehicle_status"), f.topic("vehicle_local_position")
    trans = st["in_transition_mode"].astype(int)
    fw = st["vehicle_type"] == 2
    alt = -lp["z"]
    out = []
    starts = np.where(np.diff(trans) == 1)[0] + 1
    ends = np.where(np.diff(trans) == -1)[0] + 1
    for s in starts:
        e = ends[ends > s]
        ts = f.t(st)[s]
        if not len(e):
            out.append({"kind": "front" if not fw[s] else "back", "completed": False})
            continue
        te = f.t(st)[e[0]]
        window = (f.t(lp) >= ts) & (f.t(lp) <= te + 5.0)
        a0 = float(np.interp(ts, f.t(lp), alt))
        out.append({"kind": "front" if fw[e[0]] else "back", "completed": True, "duration_s": float(te - ts),
                    "alt_loss_m": float(a0 - alt[window].min()) if window.any() else float("nan")})
    return out


def hold_error(f: Flight, t_from: float, t_to: float):
    """Max horizontal distance from the position setpoint in [t_from, t_to]."""
    lp, sp = f.topic("vehicle_local_position"), f.topic("vehicle_local_position_setpoint")
    if lp is None or sp is None:
        return float("nan")
    m = (f.t(lp) >= t_from) & (f.t(lp) <= t_to)
    if not m.any():
        return float("nan")
    xs = np.interp(f.t(lp)[m], f.t(sp), sp["x"])
    ys = np.interp(f.t(lp)[m], f.t(sp), sp["y"])
    ok = np.isfinite(xs) & np.isfinite(ys)
    d = np.hypot(lp["x"][m][ok] - xs[ok], lp["y"][m][ok] - ys[ok])
    return float(d.max()) if len(d) else float("nan")


def tilt_deg(f: Flight):
    att = f.topic("vehicle_attitude")
    return f.t(att), np.degrees(2 * np.arcsin(np.clip(np.sqrt(att["q[1]"] ** 2 + att["q[2]"] ** 2), 0, 1)))


def landing(f: Flight):
    ld = f.topic("vehicle_land_detected")
    t, tilt = tilt_deg(f)
    lp = f.topic("vehicle_local_position")
    return {"landed": bool(ld["landed"][-1]), "final_tilt_deg": float(tilt[-1]),
            "max_tilt_deg": float(tilt.max()),
            "impact_vz_m_s": float(np.max(lp["vz"][-int(len(lp["vz"]) * 0.2):]))}


def nav_states(f: Flight):
    st = f.topic("vehicle_status")
    nav = st["nav_state"]
    idx = np.r_[0, np.where(np.diff(nav) != 0)[0] + 1]
    return [(float(f.t(st)[i]), int(nav[i])) for i in idx]


def home_distance_end(f: Flight):
    lp = f.topic("vehicle_local_position")
    return float(np.hypot(lp["x"][-1], lp["y"][-1]))
