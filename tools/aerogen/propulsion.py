"""Coaxial ducted fan (two counter-rotating rotors, one motor each) and exit vanes.

Hand-calc model, per rotor, coefficients in the usual propeller form:
    T = CT(J) * rho * n^2 * D^4,  Q = CQ(J) * rho * n^2 * D^5,  J = V_axial / (n D)
CT is linear in J (zero at J0). CQ follows from ducted momentum theory with
exit area ratio sigma:
    T_unit = rho * sigma * A * Ve * (Ve - V),  P_ideal = T_unit * (Ve + V) / 2
plus a profile term fixed so that hover power = P_ideal / FM * (1 + coax loss),
the formula used in the design doc. The tables describe each rotor's share
of the whole unit (duct lip thrust included); the two rotors are assumed
pitched so their torques balance at equal rpm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CoaxialDuctedFan:
    rho: float
    diameter: float
    disk_area: float           # annulus between hub and duct wall
    exit_area_ratio: float
    figure_of_merit: float
    coax_loss: float
    j0: float
    ct0: float
    max_rps: float

    @classmethod
    def from_config(cls, cfg: dict) -> "CoaxialDuctedFan":
        r, d = cfg["rotors"], cfg["duct"]
        rho = cfg["environment"]["air_density_kg_m3"]
        weight = cfg["mass"]["mtow_kg"] * cfg["environment"]["gravity_m_s2"]
        D = r["diameter_m"]
        n_max = r["max_rpm"] / 60.0
        # Size CT0 so both rotors at max rpm, static, give the required T/W.
        ct0 = r["max_thrust_to_weight"] * weight / 2.0 / (rho * n_max ** 2 * D ** 4)
        area = math.pi / 4.0 * (d["inner_diameter_m"] ** 2 - d["hub_diameter_m"] ** 2)
        return cls(rho, D, area, d["exit_area_ratio"], r["figure_of_merit"],
                   r["coaxial_power_loss"], r["zero_thrust_advance_ratio"], ct0, n_max)

    # -- coefficients -------------------------------------------------------
    def ct(self, j):
        return self.ct0 * (1.0 - np.asarray(j, dtype=float) / self.j0)

    def _ideal_unit_power(self, thrust, v):
        """Ducted momentum theory, both rotors together. Clamped in the windmill state."""
        sa = self.rho * self.exit_area_ratio * self.disk_area
        ve = 0.5 * (v + np.sqrt(np.maximum(v * v + 4.0 * thrust / sa, 0.0)))
        return thrust * (ve + v) / 2.0

    def cq(self, j):
        """Per-rotor torque coefficient. Evaluated at n = 1 rps (the coefficients are rpm-free)."""
        j = np.asarray(j, dtype=float)
        n, D = 1.0, self.diameter
        v = j * n * D
        t_unit = 2.0 * self.ct(j) * self.rho * n ** 2 * D ** 4
        t_hover = 2.0 * self.ct0 * self.rho * n ** 2 * D ** 4
        p_ideal_hover = self._ideal_unit_power(t_hover, 0.0)
        p_profile = (1.0 / self.figure_of_merit - 1.0) * p_ideal_hover
        p_unit = (1.0 + self.coax_loss) * (self._ideal_unit_power(t_unit, v) + p_profile)
        q_rotor = p_unit / 2.0 / (2.0 * math.pi * n)
        return q_rotor / (self.rho * n ** 2 * D ** 5)

    def efficiency(self, j):
        j = np.asarray(j, dtype=float)
        return j * self.ct(j) / (2.0 * math.pi * self.cq(j))

    # -- dimensional helpers (both rotors at the same rpm) ----------------------
    def unit_thrust(self, rps, v=0.0):
        j = v / (rps * self.diameter)
        return 2.0 * float(self.ct(j)) * self.rho * rps ** 2 * self.diameter ** 4

    def rotor_torque(self, rps, v=0.0):
        j = v / (rps * self.diameter)
        return float(self.cq(j)) * self.rho * rps ** 2 * self.diameter ** 5

    def shaft_power(self, rps, v=0.0):
        return 2.0 * self.rotor_torque(rps, v) * 2.0 * math.pi * rps

    def rps_for_thrust(self, thrust, v=0.0):
        """Bisection on rpm; returns None if out of reach below max rpm."""
        lo, hi = 1e-3, self.max_rps
        if self.unit_thrust(hi, v) < thrust:
            return None
        for _ in range(100):
            mid = 0.5 * (lo + hi)
            if self.unit_thrust(mid, v) < thrust:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def exit_velocity(self, thrust, v=0.0):
        sa = self.rho * self.exit_area_ratio * self.disk_area
        return 0.5 * (v + math.sqrt(v * v + 4.0 * thrust / sa))


@dataclass(frozen=True)
class ExitVanes:
    """Jet deflection by vanes at the duct exit, as a fraction of unit thrust."""
    turning_efficiency: float
    stall: float  # rad

    @classmethod
    def from_config(cls, cfg: dict) -> "ExitVanes":
        v = cfg["vanes"]
        return cls(v["turning_efficiency"], math.radians(v["stall_deflection_deg"]))

    def jet_angle(self, delta):
        # Past vane stall the jet deflection stops growing (plateau).
        return self.turning_efficiency * np.clip(np.asarray(delta, dtype=float), -self.stall, self.stall)

    def side_force_ratio(self, delta):
        return np.sin(self.jet_angle(delta))

    def axial_loss_ratio(self, delta):
        return 1.0 - np.cos(self.jet_angle(delta))
