"""Whole-aircraft hand-calc aero (propeller off) and mass properties.

Frames: fixed-wing body FRD (x nose, y right wing, z down). A station s is
measured aft from the nose, so body x = s_ref - s. Forces are reduced to the
config's aero reference station; the dynamic models move them to the actual CG.
Coefficients use wing area S, mean chord c (pitch) and span b (roll/yaw).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .surfaces import (
    LiftingSurface,
    finite_wing_lift_slope,
    flap_cm_qc_per_rad,
    flap_effectiveness,
    stall_blend,
)


@dataclass(frozen=True)
class MassProperties:
    mass: float
    cg_station: float
    inertia: tuple[float, float, float]  # Ixx, Iyy, Izz about CG, body FRD


def mass_properties(cfg: dict) -> MassProperties:
    comps = cfg["mass"]["components"]
    m = sum(c["mass_kg"] for c in comps)
    s_cg = sum(c["mass_kg"] * c["station_m"] for c in comps) / m
    ixx = iyy = izz = 0.0
    for c in comps:
        mi = c["mass_kg"]
        lx, ly, lz = c["size_m"]
        dx = c["station_m"] - s_cg
        # All components sit on the centreline; boxes add their own inertia.
        ixx += mi * (ly ** 2 + lz ** 2) / 12.0
        iyy += mi * (lx ** 2 + lz ** 2) / 12.0 + mi * dx ** 2
        izz += mi * (lx ** 2 + ly ** 2) / 12.0 + mi * dx ** 2
    return MassProperties(m, s_cg, (ixx, iyy, izz))


class Aircraft:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        w, t = cfg["wing"], cfg["tail"]
        self.S = w["area_m2"]
        self.b = w["span_m"]
        self.c = self.S / self.b
        self.s_ref = cfg["aero"]["reference_station_m"]

        ar_w = self.b ** 2 / self.S
        a_w = finite_wing_lift_slope(w["airfoil_lift_slope_per_rad"], ar_w, math.radians(w["sweep_deg"]))
        self.wing = LiftingSurface(
            area=self.S, span=self.b, lift_slope=a_w,
            cl0=-a_w * math.radians(w["airfoil_zero_lift_alpha_deg"]),
            cm0=w["airfoil_cm0"], cd0=w["cd0"], oswald=w["oswald_efficiency"],
            alpha_stall=math.radians(w["stall_alpha_deg"]), blend_rate=w["stall_blend_rate"],
            cd90=w["flat_plate_cd90"],
        )

        # A fin pair (two opposite fins of the + tail) acts as one surface.
        pair_area = 2.0 * t["fin_area_m2"]
        pair_span = 2.0 * t["fin_span_m"]
        a_t = finite_wing_lift_slope(t["lift_slope_2d_per_rad"], pair_span ** 2 / pair_area)
        self.fin_pair = LiftingSurface(
            area=pair_area, span=pair_span, lift_slope=a_t, cl0=0.0, cm0=0.0,
            cd0=t["cd0"], oswald=t["oswald_efficiency"],
            alpha_stall=math.radians(t["stall_alpha_deg"]), blend_rate=w["stall_blend_rate"],
            cd90=t["flat_plate_cd90"],
        )
        self.eta_t = t["dynamic_pressure_ratio"]

    # -- helpers ----------------------------------------------------------
    def _arm(self, station: float) -> float:
        """Distance aft of the reference station."""
        return station - self.s_ref

    @staticmethod
    def _to_body(cl, cd, angle):
        """Lift/drag (lift 'up' = -z) in the plane of `angle` -> body (axial, normal) coefficients."""
        ca, sa = np.cos(angle), np.sin(angle)
        cx = cl * sa - cd * ca
        cz = -cl * ca - cd * sa
        return cx, cz

    @staticmethod
    def _to_wind(cx, cz, angle):
        ca, sa = np.cos(angle), np.sin(angle)
        return cx * sa - cz * ca, -cx * ca - cz * sa

    def downwash(self, alpha):
        cl_w, _, _ = self.wing.coefficients(alpha)
        sigma = stall_blend(alpha, self.wing.alpha_stall, self.wing.blend_rate)
        return (1.0 - sigma) * 2.0 * cl_w / (math.pi * self.wing.aspect_ratio)

    # -- tables -----------------------------------------------------------
    def longitudinal(self, alpha):
        """CL, CD, Cm (about reference station) vs alpha, beta = 0, propeller off."""
        alpha = np.asarray(alpha, dtype=float)
        w, t, body = self.cfg["wing"], self.cfg["tail"], self.cfg["body"]
        cx = np.zeros_like(alpha)
        cz = np.zeros_like(alpha)
        cm = np.zeros_like(alpha)

        cl_w, cd_w, cm_w = self.wing.coefficients(alpha)
        x, z = self._to_body(cl_w, cd_w, alpha)
        cx += x
        cz += z
        cm += cm_w + self._arm(w["quarter_chord_station_m"]) / self.c * z

        k = self.eta_t * self.fin_pair.area / self.S
        i_h = math.radians(t.get("horizontal_incidence_deg", 0.0))
        cl_h, cd_h, _ = self.fin_pair.coefficients(alpha - self.downwash(alpha) + i_h)
        x, z = self._to_body(cl_h, cd_h, alpha)
        cx += k * x
        cz += k * z
        cm += k * self._arm(t["quarter_chord_station_m"]) / self.c * z
        # The vertical fin pair only adds profile drag at beta = 0.
        cx += -k * self.fin_pair.cd0 * np.cos(alpha)

        # Fuselage/duct: axial parasite drag + crossflow normal force.
        ca, sa = np.cos(alpha), np.sin(alpha)
        cx += -body["parasite_cd0"] * ca * np.abs(ca)
        cz_body = -body["crossflow_cd"] * body["side_area_m2"] / self.S * sa * np.abs(sa)
        cz += cz_body
        cm += self._arm(body["side_area_centroid_station_m"]) / self.c * cz_body

        cl, cd = self._to_wind(cx, cz, alpha)
        return cl, cd, cm

    def lateral(self, beta):
        """CY, Cl, Cn (about reference station) vs sideslip, alpha = 0, propeller off."""
        beta = np.asarray(beta, dtype=float)
        t, body = self.cfg["tail"], self.cfg["body"]
        k = self.eta_t * self.fin_pair.area / self.S
        # Sideslip plays the role of alpha for the vertical fins; side force is +y.
        cl_v, cd_v, _ = self.fin_pair.coefficients(beta)
        _, cy_v = self._to_body(cl_v, cd_v, beta)
        cy_v = k * cy_v
        sb = np.sin(beta)
        cy_body = -body["crossflow_cd"] * body["side_area_m2"] / self.S * sb * np.abs(sb)

        cy = cy_v + cy_body
        # N = r_x * F_y with r_x = -(arm): a force aft of the reference yaws the nose into the wind.
        cn = (-self._arm(t["quarter_chord_station_m"]) * cy_v
              - self._arm(body["side_area_centroid_station_m"]) * cy_body) / self.b
        cl_roll = np.zeros_like(beta)  # no dihedral; the + tail is symmetric
        return cy, cl_roll, cn

    # -- linear derivatives ----------------------------------------------
    def derivatives(self, cl_trim: float) -> dict:
        """Damping (non-dimensional, hat-rates) and control derivatives, per rad."""
        w, t, e = self.cfg["wing"], self.cfg["tail"], self.cfg["elevons"]
        a_w, a_t = self.wing.lift_slope, self.fin_pair.lift_slope
        s_ratio = self.fin_pair.area / self.S
        l_t = self._arm(t["quarter_chord_station_m"])
        v_h = s_ratio * l_t / self.c
        v_v = s_ratio * l_t / self.b
        span_ratio = self.fin_pair.span / self.b

        tau = flap_effectiveness(e["chord_fraction"])
        f1, f2 = e["span_start_fraction"], e["span_end_fraction"]
        cl_de = a_w * tau * (f2 - f1)
        dcm_flap = flap_cm_qc_per_rad(e["chord_fraction"]) * (a_w / w["airfoil_lift_slope_per_rad"]) * (f2 - f1)
        cm_de = -cl_de * self._arm(w["quarter_chord_station_m"]) / self.c + dcm_flap

        # Tail elevator on the horizontal fin pair: lift at the tail, arm l_t.
        tau_el = flap_effectiveness(self.cfg["elevator"]["chord_fraction"])
        cl_el = self.eta_t * s_ratio * a_t * tau_el

        tail_roll = -(a_t / 6.0) * s_ratio * span_ratio ** 2  # one fin pair
        return {
            "CLq": 2.0 * self.eta_t * a_t * v_h,
            "Cmq": -2.0 * self.eta_t * a_t * v_h * l_t / self.c,
            "Clp": -a_w / 6.0 + 2.0 * tail_roll,
            "Cnr": -2.0 * self.eta_t * a_t * v_v * l_t / self.b,
            "CYr": 2.0 * self.eta_t * a_t * v_v,
            "Clr": cl_trim / 4.0,
            "Cnp": -cl_trim / 8.0,
            "CL_elevon_sym": cl_de,
            "Cm_elevon_sym": cm_de,
            "Cl_elevon_diff": a_w * tau * (f2 ** 2 - f1 ** 2) / 4.0,
            "elevon_flap_effectiveness": tau,
            "CL_elevator": cl_el,
            "Cm_elevator": -cl_el * l_t / self.c,
        }
