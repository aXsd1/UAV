"""Scenario set from the simulation doc. Each run samples its conditions from an RNG, flies
them on a fresh SITL instance and returns parameters, metrics and a pass flag.

Pass criteria (simulation doc, "Test senaryoları"):
  hover_wind      : <= 8 m/s mean wind: hold error < 0.5 m (8-12 m/s recorded, no criterion) + upright landing
  transition      : front < 5 s and < 5 m altitude loss, back completes, upright landing ("sıfır kayıp")
  actuator_failure: safe landing (upright) or controlled descent (sink rate after the failure < 4 m/s);
                    the outcome is recorded as "safe_landing" / "controlled_descent" / "uncontrolled"
  link_loss       : datalink loss -> RTL, lands within 10 m of home, upright
  gnss_loss       : recorded only (the 10 min / < 50 m target needs VIO, not built yet)
  urban_landing   : Karabük city spot, gusty wind: lands within 1.5 m of the target (3 x 3 m), upright
"""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np

from . import metrics as M
from .sim import Sim, SimError


def _wind_vec(speed, from_deg):
    # meteorological "from" direction -> ENU velocity of the air
    a = math.radians(from_deg)
    return -speed * math.sin(a), -speed * math.cos(a)


def _px4_time(sim: Sim) -> float:
    return int(sim.listen("vehicle_status").get("timestamp", "0").split()[0]) / 1e6


# --------------------------------------------------------------------------------------
def hover_wind(sim: Sim, rng: np.random.Generator):
    p = {"wind_m_s": float(rng.uniform(0, 12)), "wind_from_deg": float(rng.uniform(0, 360))}
    p["turbulence_m_s"] = 0.1 * p["wind_m_s"]
    sim.start()
    sim.takeoff(10)
    sim.publish("turbulence", "gz.msgs.Double", f"data: {p['turbulence_m_s']:.3f}")
    sim.wind(*_wind_vec(p["wind_m_s"], p["wind_from_deg"]), ramp_s=3)
    sim.sleep_sim(15)                      # weathervane + settle
    t0 = _px4_time(sim)
    sim.sleep_sim(25)
    t1 = _px4_time(sim)
    landed = sim.land_and_wait(90)
    sim.stop()
    f = M.Flight(sim.ulog())
    m = {"hold_error_m": M.hold_error(f, t0, t1), **M.landing(f), "landed_in_time": landed}
    upright = m["landed"] and m["final_tilt_deg"] < 30
    ok = upright and (p["wind_m_s"] > 8 or m["hold_error_m"] < 0.5)
    return p, m, ok


def transition(sim: Sim, rng: np.random.Generator):
    lo, hi = (float(v) for v in os.environ.get("MC_WIND_RANGE", "0,8").split(","))  # e.g. MC_WIND_RANGE=6,8
    p = {"wind_m_s": float(rng.uniform(lo, hi)), "wind_from_deg": float(rng.uniform(0, 360))}
    p["turbulence_m_s"] = 0.1 * p["wind_m_s"]
    # MC_LEGACY=1: settings before the 2026-10-22 fixes (comparison runs on the same seeds)
    legacy = os.environ.get("MC_LEGACY") == "1"
    sim.start()
    if legacy:
        for name, value in (("VT_ARSP_TRANS", 19.1), ("VT_F_TRANS_DUR", 3), ("EKF2_RNG_CTRL", 0)):
            sim.param(name, value)
    sim.publish("turbulence", "gz.msgs.Double", f"data: {p['turbulence_m_s']:.3f}")
    sim.wind(*_wind_vec(p["wind_m_s"], p["wind_from_deg"]))
    sim.takeoff(30)
    sim.sleep_sim(8)                        # weathervane settles, the lean shows the wind side
    # Transition into the wind (Monte Carlo 2026-10-22: tail/crosswind transitions lost the most height).
    up = None if legacy else sim.upwind_heading()
    p["upwind_heading_deg"] = None if up is None else math.degrees(up)
    if up is not None:
        sim.param("WV_EN", 0)               # otherwise the weathervane turns the span back into the wind
        p["faced_wind"] = sim.turn_to(up)
    sim.cmd("commander", "transition")
    sim.sleep_sim(8)
    sim.param("WV_EN", 1)
    sim.sleep_sim(17)
    sim.cmd("commander", "transition")
    sim.sleep_sim(12)
    landed = sim.land_and_wait(90)
    sim.stop()
    f = M.Flight(sim.ulog())
    tr = M.transitions(f)
    front = next((x for x in tr if x["kind"] == "front"), {"completed": False})
    back = next((x for x in tr if x["kind"] == "back"), {"completed": False})
    m = {"front": front, "back": back, **M.landing(f), "landed_in_time": landed}
    ok = (front.get("completed") and front["duration_s"] < 5 and front["alt_loss_m"] < 5
          and back.get("completed") and m["landed"] and m["final_tilt_deg"] < 30)
    return p, m, bool(ok)


def actuator_failure(sim: Sim, rng: np.random.Generator):
    kind = str(rng.choice(["motor 0", "motor 1", "servo 0 stuck", "servo 1 stuck"]))
    p = {"failure": kind, "wind_m_s": float(rng.uniform(0, 5)), "wind_from_deg": float(rng.uniform(0, 360))}
    sim.start()
    sim.wind(*_wind_vec(p["wind_m_s"], p["wind_from_deg"]))
    sim.takeoff(20)
    sim.sleep_sim(5)
    t_fail = _px4_time(sim)
    sim.publish("failure", "gz.msgs.StringMsg", f'data: "{kind}"')
    sim.sleep_sim(5)
    sim.cmd("commander", "land")           # what an operator / failure detector would command
    landed = sim.wait_for(sim.landed, timeout_sim=60, poll_sim=2.0)
    sim.stop()
    f = M.Flight(sim.ulog())
    m = {**M.landing(f, t_fail), "landed_in_time": landed, "nav": M.nav_states(f)[-4:]}
    upright = m["landed"] and m["final_tilt_deg"] < 30
    m["outcome"] = ("safe_landing" if upright else
                    "controlled_descent" if m["max_descent_m_s"] < 4 else "uncontrolled")
    return p, m, m["outcome"] != "uncontrolled"


def link_loss(sim: Sim, rng: np.random.Generator):
    from pymavlink import mavutil
    p = {"wind_m_s": float(rng.uniform(0, 5)), "wind_from_deg": float(rng.uniform(0, 360)),
         "loss_after_fw_s": float(rng.uniform(5, 20))}
    sim.start()
    sim.param("NAV_DLL_ACT", 2)            # datalink loss -> RTL
    sim.param("COM_DL_LOSS_T", 5)
    sim.wind(*_wind_vec(p["wind_m_s"], p["wind_from_deg"]))
    gcs = mavutil.mavlink_connection(f"udpout:127.0.0.1:{18570 + sim.i}", source_system=255)
    alive = threading.Event()
    alive.set()

    def heartbeat():
        while alive.is_set():
            gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.5 / sim.speed)

    th = threading.Thread(target=heartbeat, daemon=True)
    th.start()
    try:
        sim.takeoff(40)
        sim.cmd("commander", "transition")
        sim.sleep_sim(p["loss_after_fw_s"])
        t_loss = _px4_time(sim)
        alive.clear()                      # link lost
        landed = sim.wait_for(sim.landed, timeout_sim=180, poll_sim=3.0)
    finally:
        alive.clear()
        sim.stop()
    f = M.Flight(sim.ulog())
    navs = M.nav_states(f)
    rtl_t = next((t for t, n in navs if n == M.NAV_RTL and t >= t_loss), None)
    m = {**M.landing(f), "landed_in_time": landed, "rtl_after_s": None if rtl_t is None else rtl_t - t_loss,
         "home_distance_m": M.home_distance_end(f)}
    ok = rtl_t is not None and m["landed"] and m["final_tilt_deg"] < 30 and m["home_distance_m"] < 10
    return p, m, bool(ok)


def gnss_loss(sim: Sim, rng: np.random.Generator):
    p = {"outage_s": 60.0, "wind_m_s": float(rng.uniform(0, 5)), "wind_from_deg": float(rng.uniform(0, 360))}
    sim.start()
    sim.wind(*_wind_vec(p["wind_m_s"], p["wind_from_deg"]))
    sim.takeoff(20)
    start = sim.pose()
    sim.param("EKF2_GPS_CTRL", 0)          # stop fusing GNSS: what the estimator sees under jamming
    sim.sleep_sim(p["outage_s"])
    end = sim.pose()
    est = sim.listen("vehicle_local_position")
    sim.stop()
    f = M.Flight(sim.ulog())
    drift = math.hypot(end[0] - start[0], end[1] - start[1]) if start and end else float("nan")
    m = {"true_drift_m": drift, "xy_valid_at_end": est.get("xy_valid"), "nav": M.nav_states(f)[-4:],
         "messages": [x for x in f.messages() if "GPS" in x or "position" in x.lower()][-5:]}
    return p, m, None


def urban_landing(sim: Sim, rng: np.random.Generator):
    p = {"wind_m_s": float(rng.uniform(2, 8)), "wind_from_deg": float(rng.uniform(0, 360))}
    p["turbulence_m_s"] = 0.25 * p["wind_m_s"]   # gusts between buildings
    sim.start()
    sim.publish("turbulence", "gz.msgs.Double", f"data: {p['turbulence_m_s']:.3f}")
    sim.wind(*_wind_vec(p["wind_m_s"], p["wind_from_deg"]))
    start = sim.pose()
    sim.takeoff(25)
    sim.sleep_sim(10)
    landed = sim.land_and_wait(120)
    end = sim.pose()
    sim.stop()
    f = M.Flight(sim.ulog())
    err = math.hypot(end[0] - start[0], end[1] - start[1]) if start and end else float("nan")
    m = {"landing_error_m": err, **M.landing(f), "landed_in_time": landed}
    ok = m["landed"] and m["final_tilt_deg"] < 30 and err < 1.5
    return p, m, bool(ok)


SCENARIOS = {
    "hover_wind": (hover_wind, {"world": "default"}),
    "transition": (transition, {"world": "default"}),
    "actuator_failure": (actuator_failure, {"world": "default"}),
    "link_loss": (link_loss, {"world": "default"}),
    "gnss_loss": (gnss_loss, {"world": "default"}),
    "urban_landing": (urban_landing, {"world": "karabuk", "model_pose": None}),  # pose set by run.py
}
