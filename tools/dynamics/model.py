"""Reference 6-DOF force model for the tail-sitter (numpy).

The C++ Gazebo plugin (sim/gazebo/vtol_dynamics) and the Isaac Lab env port
these equations; test vectors written by tools.dynamics.vectors keep them
equal. Everything here is in body FRD (x nose, y right wing, z down); moments
are about the CG. Inputs are the actual (already lagged/rate-limited)
actuator states; actuator dynamics live in ActuatorState.step().
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


def _read_csv(path: Path) -> dict[str, np.ndarray]:
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    header, data = rows[0], np.array(rows[1:], dtype=float)
    return {h: data[:, i] for i, h in enumerate(header)}


def interp(x, xs, ys):
    """Linear interpolation, clamped at the table ends."""
    return np.interp(x, xs, ys)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class Inputs:
    v_air: np.ndarray            # body velocity relative to the air, FRD, m/s
    omega: np.ndarray            # body rates p, q, r, rad/s
    rotor_speed: tuple[float, float]  # upper, lower, rad/s (>= 0)
    vane: tuple[float, float]    # pitch, yaw deflection, rad
    elevon: tuple[float, float]  # left, right deflection, rad (trailing edge down +)
    rotor_accel: tuple[float, float] = (0.0, 0.0)


@dataclass
class Output:
    force: np.ndarray
    moment: np.ndarray
    info: dict = field(default_factory=dict)


class VtolModel:
    def __init__(self, aero_dir: str | Path, cg_station: float | None = None, rho: float | None = None):
        d = Path(aero_dir)
        self.meta = json.loads((d / "aero.json").read_text(encoding="utf-8"))
        self.alpha_t = _read_csv(d / "airframe_alpha.csv")
        self.beta_t = _read_csv(d / "airframe_beta.csv")
        self.rotor_t = _read_csv(d / "rotor.csv")
        self.vane_t = _read_csv(d / "vane.csv")

        ref = self.meta["reference"]
        self.S, self.b, self.c = ref["area_m2"], ref["span_m"], ref["chord_m"]
        self.s_ref = ref["moment_reference_station_m"]
        self.s_cg = self.meta["mass"]["cg_station_m"] if cg_station is None else cg_station
        self.rho = self.meta["air_density_kg_m3"] if rho is None else rho
        self.dv = self.meta["derivatives_per_rad"]
        r, du = self.meta["rotor"], self.meta["duct"]
        self.D = r["diameter_m"]
        self.I_rotor = r["rotor_inertia_kg_m2"]
        self.sigma_a = du["exit_area_ratio"] * du["disk_area_m2"]
        self.s_duct = du["center_station_m"]
        self.s_vane = du["exit_station_m"]

    def arm(self, station: float) -> np.ndarray:
        """Position of a station relative to the CG, body FRD."""
        return np.array([self.s_cg - station, 0.0, 0.0])

    # -- pieces ---------------------------------------------------------------
    def rotor(self, omega_rad_s: float, v_axial: float):
        """Thrust (N, +x) and torque magnitude (N m) of one rotor."""
        n = omega_rad_s / (2.0 * math.pi)
        if n < 1e-3:
            return 0.0, 0.0
        j = max(v_axial, 0.0) / (n * self.D)
        ct = interp(j, self.rotor_t["J"], self.rotor_t["CT"])
        cq = interp(j, self.rotor_t["J"], self.rotor_t["CQ"])
        return float(ct * self.rho * n * n * self.D ** 4), float(cq * self.rho * n * n * self.D ** 5)

    def jet(self, thrust: float, v_axial: float):
        """Duct mass flow (kg/s) and exit velocity (m/s), momentum theory."""
        if thrust <= 0.0:
            return 0.0, max(v_axial, 0.0)
        u = max(v_axial, 0.0)
        ve = 0.5 * (u + math.sqrt(u * u + 4.0 * thrust / (self.rho * self.sigma_a)))
        return self.rho * self.sigma_a * ve, ve

    def airframe(self, v_air: np.ndarray, omega: np.ndarray, elevon):
        """Propeller-off airframe force (FRD) and moment about the reference station."""
        u, v, w = v_air
        V = float(np.linalg.norm(v_air))
        if V < 1e-3:
            return np.zeros(3), np.zeros(3), 0.0, 0.0
        alpha = math.atan2(w, u)
        beta = math.asin(max(-1.0, min(1.0, v / V)))
        a_deg, b_deg = math.degrees(alpha), math.degrees(beta)
        cl = interp(a_deg, self.alpha_t["alpha_deg"], self.alpha_t["CL"])
        cd = interp(a_deg, self.alpha_t["alpha_deg"], self.alpha_t["CD"])
        cm = interp(a_deg, self.alpha_t["alpha_deg"], self.alpha_t["Cm"])
        cy = interp(b_deg, self.beta_t["beta_deg"], self.beta_t["CY"])
        c_roll = interp(b_deg, self.beta_t["beta_deg"], self.beta_t["Cl"])
        cn = interp(b_deg, self.beta_t["beta_deg"], self.beta_t["Cn"])

        de = 0.5 * (elevon[0] + elevon[1])
        da = 0.5 * (elevon[0] - elevon[1])
        cl += self.dv["CL_elevon_sym"] * de
        cm += self.dv["Cm_elevon_sym"] * de
        c_roll += self.dv["Cl_elevon_diff"] * da

        qbar = 0.5 * self.rho * V * V
        # The alpha tables only see the flow in the x-z plane (q cos^2 beta), so they fade
        # out smoothly as beta -> 90 deg, where alpha itself becomes ill-defined.
        qbar_xz = 0.5 * self.rho * (u * u + w * w)
        ca, sa = math.cos(alpha), math.sin(alpha)
        cx = cl * sa - cd * ca
        cz = -cl * ca - cd * sa
        force = self.S * np.array([qbar_xz * cx, qbar * cy, qbar_xz * cz])

        # Damping with hat rates: qbar*S*c*Cmq*(q c / 2V) = 0.25 rho V S c^2 Cmq q (no 1/V singularity).
        p, q, r = omega
        k = 0.25 * self.rho * V * self.S
        damp = np.array([
            k * self.b ** 2 * (self.dv["Clp"] * p + self.dv["Clr"] * r),
            k * self.c ** 2 * self.dv["Cmq"] * q,
            k * self.b ** 2 * (self.dv["Cnr"] * r + self.dv["Cnp"] * p),
        ])
        damp_force_z = -k * self.c * self.dv["CLq"] * q
        force = force + np.array([0.0, k * self.b * self.dv["CYr"] * r, damp_force_z])
        moment = self.S * np.array([qbar * self.b * c_roll, qbar_xz * self.c * cm, qbar * self.b * cn]) + damp
        return force, moment, alpha, beta

    # -- total ------------------------------------------------------------------
    def wrench(self, x: Inputs) -> Output:
        v_air = np.asarray(x.v_air, dtype=float)
        omega = np.asarray(x.omega, dtype=float)
        u = v_air[0]

        t_up, q_up = self.rotor(x.rotor_speed[0], u)
        t_lo, q_lo = self.rotor(x.rotor_speed[1], u)
        thrust = t_up + t_lo
        mdot, ve = self.jet(thrust, u)
        jet_momentum = mdot * ve

        force = np.zeros(3)
        moment = np.zeros(3)

        # Rotor thrust on the axis (no moment about a centreline CG).
        # Upper spins CCW seen from the nose (spin vector +x) -> reaction torque -Q x.
        dp, dy = x.vane
        side_p = float(interp(math.degrees(dp), self.vane_t["delta_deg"], self.vane_t["side_force_ratio"]))
        side_y = float(interp(math.degrees(dy), self.vane_t["delta_deg"], self.vane_t["side_force_ratio"]))
        loss = (float(interp(math.degrees(dp), self.vane_t["delta_deg"], self.vane_t["axial_loss_ratio"]))
                + float(interp(math.degrees(dy), self.vane_t["delta_deg"], self.vane_t["axial_loss_ratio"])))
        force[0] += thrust - jet_momentum * loss
        moment[0] += q_lo - q_up

        # Gyroscopic and spin-up reaction of the rotors.
        h = np.array([self.I_rotor * (x.rotor_speed[0] - x.rotor_speed[1]), 0.0, 0.0])
        moment += -np.cross(omega, h)
        moment[0] += -self.I_rotor * (x.rotor_accel[0] - x.rotor_accel[1])

        # Exit vanes turn the jet: +pitch vane -> nose-up moment, +yaw vane -> nose-right moment.
        r_vane = self.arm(self.s_vane)
        f_vane = np.array([0.0, jet_momentum * side_y, -jet_momentum * side_p])
        force += f_vane
        moment += np.cross(r_vane, f_vane)

        # Duct momentum drag: the ingested air loses its cross-axis velocity.
        f_md = -mdot * np.array([0.0, v_air[1], v_air[2]])
        force += f_md
        moment += np.cross(self.arm(self.s_duct), f_md)

        f_af, m_af_ref, alpha, beta = self.airframe(v_air, omega, x.elevon)
        force += f_af
        moment += m_af_ref + np.cross(self.arm(self.s_ref), f_af)

        info = {"thrust": thrust, "mdot": mdot, "v_exit": ve, "alpha": alpha, "beta": beta,
                "rotor_torque": (q_up, q_lo)}
        return Output(force, moment, info)


@dataclass
class ActuatorState:
    """First-order motor lag; rate-limited servos with deadband. Commands: rotor rad/s, surfaces rad."""
    rotor: np.ndarray = field(default_factory=lambda: np.zeros(2))
    vane: np.ndarray = field(default_factory=lambda: np.zeros(2))
    elevon: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def step(self, meta: dict, cmd_rotor, cmd_vane, cmd_elevon, dt: float):
        tau = meta["rotor"]["motor_time_constant_s"]
        prev = self.rotor.copy()
        self.rotor = self.rotor + (np.asarray(cmd_rotor) - self.rotor) * (1.0 - math.exp(-dt / tau))
        accel = (self.rotor - prev) / dt

        rate = math.radians(meta["vanes"]["servo_rate_deg_s"]) * dt
        dead = math.radians(meta["vanes"]["deadband_deg"])
        vmax = math.radians(meta["vanes"]["max_deflection_deg"])
        emax = math.radians(meta["elevons"]["max_deflection_deg"])

        def servo(cur, cmd, limit):
            target = np.clip(np.asarray(cmd, dtype=float), -limit, limit)
            err = target - cur
            err = np.where(np.abs(err) < dead, 0.0, err)
            return cur + np.clip(err, -rate, rate)

        self.vane = servo(self.vane, cmd_vane, vmax)
        self.elevon = servo(self.elevon, cmd_elevon, emax)
        return accel
