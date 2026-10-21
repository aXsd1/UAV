"""Full-range (-180..180 deg) lifting-surface model.

Attached flow uses finite-wing lift slope and a parabolic drag polar; separated
flow uses a flat plate. The two are blended with the sigmoid from Beard & McLain,
"Small Unmanned Aircraft", ch. 4, so the coefficients stay continuous through
stall, which the transition (0-90 deg) and gust cases need.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def finite_wing_lift_slope(a0: float, aspect_ratio: float, sweep_rad: float = 0.0) -> float:
    """DATCOM / Helmholtz lift slope per rad, incompressible."""
    kappa = a0 / (2.0 * math.pi)
    a = aspect_ratio
    root = math.sqrt(a * a / (kappa * kappa) * (1.0 + math.tan(sweep_rad) ** 2) + 4.0)
    return 2.0 * math.pi * a / (2.0 + root)


def wrap_angle(alpha):
    """Wrap to [-pi, pi)."""
    return (np.asarray(alpha, dtype=float) + math.pi) % (2.0 * math.pi) - math.pi


def stall_blend(alpha, alpha_stall: float, rate: float):
    """0 in attached flow around alpha=0, 1 once separated (and in reverse flow)."""
    a = wrap_angle(alpha)
    e1 = np.exp(np.clip(-rate * (a - alpha_stall), -500.0, 500.0))
    e2 = np.exp(np.clip(rate * (a + alpha_stall), -500.0, 500.0))
    return (1.0 + e1 + e2) / ((1.0 + e1) * (1.0 + e2))


@dataclass(frozen=True)
class LiftingSurface:
    area: float
    span: float
    lift_slope: float        # 3D, per rad
    cl0: float
    cm0: float               # about the surface's own quarter chord
    cd0: float
    oswald: float
    alpha_stall: float       # rad
    blend_rate: float
    cd90: float

    @property
    def chord(self) -> float:
        return self.area / self.span

    @property
    def aspect_ratio(self) -> float:
        return self.span ** 2 / self.area

    def coefficients(self, alpha):
        """Return (CL, CD, Cm_qc) on the surface's own area and chord."""
        a = wrap_angle(alpha)
        sigma = stall_blend(a, self.alpha_stall, self.blend_rate)

        cl_att = self.cl0 + self.lift_slope * a
        cd_att = self.cd0 + cl_att ** 2 / (math.pi * self.oswald * self.aspect_ratio)

        # Flat plate: normal force CD90*sin(a); centre of pressure moves from
        # 0.25c (a=0) to 0.5c (90 deg) to 0.75c (reverse flow, 180 deg).
        cn = self.cd90 * np.sin(a)
        cl_fp = cn * np.cos(a)
        cd_fp = self.cd0 + self.cd90 * np.sin(a) ** 2
        x_cp = 0.5 - 0.25 * np.cos(a)
        cm_fp = -cn * (x_cp - 0.25)

        cl = (1.0 - sigma) * cl_att + sigma * cl_fp
        cd = (1.0 - sigma) * cd_att + sigma * cd_fp
        cm = (1.0 - sigma) * self.cm0 + sigma * cm_fp
        return cl, cd, cm


def flap_effectiveness(chord_fraction: float) -> float:
    """Thin-airfoil flap effectiveness tau = d(alpha_eff)/d(delta)."""
    theta = math.acos(2.0 * chord_fraction - 1.0)
    return 1.0 - (theta - math.sin(theta)) / math.pi


def flap_cm_qc_per_rad(chord_fraction: float) -> float:
    """Thin-airfoil d(Cm_c/4)/d(delta) for a plain trailing-edge flap (2D)."""
    theta = math.acos(2.0 * chord_fraction - 1.0)
    return -0.5 * math.sin(theta) * (1.0 - math.cos(theta))
