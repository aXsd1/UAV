"""Write the coefficient tables read by the Gazebo plugin and the Isaac Lab env.

Output is deterministic (no timestamps) so a test can check the committed
tables are up to date with the config. Format is described in docs/conventions.md.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from . import __version__
from .sizing import analyse, report_markdown

FORMAT_VERSION = 1


def _fmt(x: float) -> str:
    if abs(x) < 1e-12:  # trig round-off, e.g. sin(pi)
        return "0"
    s = f"{x:.6g}"
    return "0" if s == "-0" else s


def _write_csv(path: Path, header: list[str], columns: list[np.ndarray]) -> None:
    rows = [",".join(header)]
    for row in zip(*columns):
        rows.append(",".join(_fmt(float(v)) for v in row))
    path.write_text("\n".join(rows) + "\n", encoding="utf-8", newline="\n")


def generate(cfg: dict, out_dir: str | Path) -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    res = analyse(cfg)
    ac, fan, vanes, mp = res["aircraft"], res["fan"], res["vanes"], res["mass"]

    alpha_deg = np.arange(-180.0, 180.0 + 0.5, 1.0)
    cl, cd, cm = ac.longitudinal(np.radians(alpha_deg))
    _write_csv(out / "airframe_alpha.csv", ["alpha_deg", "CL", "CD", "Cm"], [alpha_deg, cl, cd, cm])

    beta_deg = np.arange(-180.0, 180.0 + 0.5, 1.0)
    cy, cl_roll, cn = ac.lateral(np.radians(beta_deg))
    _write_csv(out / "airframe_beta.csv", ["beta_deg", "CY", "Cl", "Cn"], [beta_deg, cy, cl_roll, cn])

    j = np.round(np.linspace(0.0, 1.5 * fan.j0, 151), 6)
    _write_csv(out / "rotor.csv", ["J", "CT", "CQ"], [j, fan.ct(j), fan.cq(j)])

    vmax = cfg["vanes"]["stall_deflection_deg"] + 10.0
    delta_deg = np.arange(-vmax, vmax + 0.5, 1.0)
    d = np.radians(delta_deg)
    _write_csv(out / "vane.csv", ["delta_deg", "side_force_ratio", "axial_loss_ratio"],
               [delta_deg, vanes.side_force_ratio(d), vanes.axial_loss_ratio(d)])

    w, r, v = cfg["wing"], cfg["rotors"], cfg["vanes"]
    meta = {
        "format_version": FORMAT_VERSION,
        "generator": f"tools.aerogen {__version__}",
        "config": cfg["_source"],
        "frame": "body FRD (x nose, y right wing, z down); stations aft from nose",
        "reference": {
            "area_m2": ac.S, "span_m": ac.b, "chord_m": ac.c,
            "moment_reference_station_m": ac.s_ref,
        },
        "mass": {
            "mass_kg": mp.mass, "cg_station_m": mp.cg_station,
            "inertia_kg_m2": {"Ixx": mp.inertia[0], "Iyy": mp.inertia[1], "Izz": mp.inertia[2]},
        },
        "tables": {
            "airframe_alpha.csv": "propeller-off airframe, beta=0; CL/CD wind axes, Cm about reference station",
            "airframe_beta.csv": "propeller-off airframe, alpha=0; CY body y, Cl/Cn about reference station, span-normalised",
            "rotor.csv": "per rotor: T = CT rho n^2 D^4, Q = CQ rho n^2 D^5, J = V_axial/(n D), n in rev/s",
            "vane.csv": "jet side force / unit thrust and axial thrust loss / unit thrust vs vane deflection",
        },
        "derivatives_per_rad": res["derivatives"],
        "derivative_notes": "damping uses hat rates q c/(2V), p b/(2V), r b/(2V); "
                            "elevon_sym: both trailing edges down positive; "
                            "elevon_diff = (left - right)/2, positive rolls right",
        "rotor": {
            "diameter_m": r["diameter_m"], "max_rpm": r["max_rpm"],
            "upper_station_m": r["upper_station_m"], "lower_station_m": r["lower_station_m"],
            "spin": "upper CCW, lower CW seen from the nose (assumption)",
            "rotor_inertia_kg_m2": r["rotor_inertia_kg_m2"],
            "motor_time_constant_s": r["motor_time_constant_s"],
            "disk_area_m2": fan.disk_area, "exit_area_ratio": fan.exit_area_ratio,
        },
        "vanes": {
            "axes": v["axes"], "exit_station_m": cfg["duct"]["exit_station_m"],
            "max_deflection_deg": v["max_deflection_deg"], "servo_rate_deg_s": v["servo_rate_deg_s"],
            "deadband_deg": v["deadband_deg"],
        },
        "elevons": {"max_deflection_deg": cfg["elevons"]["max_deflection_deg"]},
        "hover_allocation": res["hover"],
        "cruise_trim": {
            "alpha_deg": math.degrees(res["cruise"]["alpha_rad"]),
            "CL": res["cruise"]["cl"], "CD": res["cruise"]["cd"], "rpm": res["cruise"]["rpm"],
        },
        "static_margin": res["static_margin"],
    }
    (out / "aero.json").write_text(json.dumps(_round(meta), indent=2, ensure_ascii=False) + "\n",
                                   encoding="utf-8", newline="\n")
    (out / "sizing_report.md").write_text(report_markdown(cfg, res), encoding="utf-8", newline="\n")
    return res


def _round(obj):
    if isinstance(obj, float):
        return float(_fmt(obj))
    if isinstance(obj, dict):
        return {k: _round(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_round(v) for v in obj]
    return obj
