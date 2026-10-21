import math
import unittest
from pathlib import Path

import numpy as np

from tools.dynamics.model import ActuatorState, Inputs, VtolModel

ROOT = Path(__file__).resolve().parents[1]
AERO = ROOT / "models" / "aero" / "p1"


def hover_inputs(m: VtolModel, **kw):
    w = 2 * math.pi * m.meta["hover_allocation"]["rpm"] / 60.0
    args = dict(v_air=np.zeros(3), omega=np.zeros(3), rotor_speed=(w, w), vane=(0.0, 0.0), elevon=(0.0, 0.0))
    args.update(kw)
    return Inputs(**args)


class Hover(unittest.TestCase):
    def setUp(self):
        self.m = VtolModel(AERO)
        self.weight = self.m.meta["mass"]["mass_kg"] * 9.80665

    def test_hover_thrust_equals_weight(self):
        out = self.m.wrench(hover_inputs(self.m))
        self.assertAlmostEqual(out.force[0] / self.weight, 1.0, places=2)
        np.testing.assert_allclose(out.force[1:], 0.0, atol=1e-9)
        np.testing.assert_allclose(out.moment, 0.0, atol=1e-9)

    def test_pitch_vane_gives_nose_up_moment(self):
        out = self.m.wrench(hover_inputs(self.m, vane=(math.radians(10), 0.0)))
        self.assertGreater(out.moment[1], 0)
        self.assertLess(out.force[2], 0)
        self.assertAlmostEqual(out.moment[0], 0.0, places=9)

    def test_vane_moment_matches_allocation_slope(self):
        d = math.radians(1.0)
        out = self.m.wrench(hover_inputs(self.m, vane=(0.0, d)))
        slope = self.m.meta["hover_allocation"]["pitch_yaw_moment_per_vane_rad_nm"]
        self.assertAlmostEqual(out.moment[2] / d / slope, 1.0, delta=0.02)

    def test_rpm_split_gives_thrust_axis_moment(self):
        w = 2 * math.pi * self.m.meta["hover_allocation"]["rpm"] / 60.0
        out = self.m.wrench(hover_inputs(self.m, rotor_speed=(1.05 * w, 0.95 * w)))
        # Upper (CCW from the nose) faster -> body reacts with -x moment.
        self.assertLess(out.moment[0], 0)

    def test_momentum_drag_opposes_cross_flow(self):
        out = self.m.wrench(hover_inputs(self.m, v_air=np.array([0.0, 0.0, 3.0])))
        out0 = self.m.wrench(hover_inputs(self.m))
        self.assertLess(out.force[2] - out0.force[2], 0)


class Sideslip(unittest.TestCase):
    def test_pure_side_wind_is_continuous(self):
        # With v_air almost along y, alpha = atan2(w, u) is arbitrary; the force must not care.
        m = VtolModel(AERO)
        forces = []
        for u, w in ((1e-3, 0.0), (0.0, 1e-3), (-1e-3, 0.0), (0.0, -1e-3)):
            out = m.wrench(hover_inputs(m, v_air=np.array([u, 8.0, w])))
            forces.append(np.concatenate([out.force, out.moment]))
        spread = np.max(np.ptp(np.array(forces), axis=0))
        self.assertLess(spread, 0.05)


class Cruise(unittest.TestCase):
    def setUp(self):
        self.m = VtolModel(AERO)
        trim = self.m.meta["cruise_trim"]
        a = math.radians(trim["alpha_deg"])
        self.v = trim["speed_m_s"] * np.array([math.cos(a), 0.0, math.sin(a)])
        w = 2 * math.pi * trim["rpm"] / 60.0
        self.inp = dict(v_air=self.v, omega=np.zeros(3), rotor_speed=(w, w), vane=(0.0, 0.0), elevon=(0.0, 0.0))

    def test_cruise_force_balance(self):
        out = self.m.wrench(Inputs(**self.inp))
        weight = self.m.meta["mass"]["mass_kg"] * 9.80665
        # Lift ~ weight, net drag ~ 0 at trim (momentum drag adds a little: allow 15 %).
        a = math.radians(self.m.meta["cruise_trim"]["alpha_deg"])
        f = out.force
        lift = f[0] * math.sin(a) - f[2] * math.cos(a)
        drag = -f[0] * math.cos(a) - f[2] * math.sin(a)
        self.assertAlmostEqual(lift / weight, 1.0, delta=0.15)
        self.assertLess(abs(drag), 0.25 * weight / 8.6)

    def test_elevon_signs(self):
        base = self.m.wrench(Inputs(**self.inp))
        d = math.radians(5)
        sym = self.m.wrench(Inputs(**{**self.inp, "elevon": (d, d)}))
        diff = self.m.wrench(Inputs(**{**self.inp, "elevon": (d, -d)}))
        self.assertLess(sym.moment[1] - base.moment[1], 0)   # TE down -> nose down
        self.assertGreater(diff.moment[0] - base.moment[0], 0)  # left down, right up -> roll right

    def test_damping_opposes_rates(self):
        base = self.m.wrench(Inputs(**self.inp))
        for i in range(3):
            om = np.zeros(3)
            om[i] = 0.5
            out = self.m.wrench(Inputs(**{**self.inp, "omega": om}))
            self.assertLess(out.moment[i] - base.moment[i], 0, f"axis {i}")

    def test_cg_shift_only_changes_moment(self):
        m2 = VtolModel(AERO, cg_station=self.m.s_cg + 0.02)
        a, b = self.m.wrench(Inputs(**self.inp)), m2.wrench(Inputs(**self.inp))
        np.testing.assert_allclose(a.force, b.force)
        # M_new = M_old + (r_old_cg - r_new_cg) x F
        expected = a.moment + np.cross(np.array([0.02, 0.0, 0.0]), a.force)
        np.testing.assert_allclose(b.moment, expected, atol=1e-9)


class Actuators(unittest.TestCase):
    def test_motor_lag_and_servo_rate(self):
        meta = VtolModel(AERO).meta
        act = ActuatorState()
        dt = 0.004
        for _ in range(int(meta["rotor"]["motor_time_constant_s"] / dt)):
            act.step(meta, (100.0, 100.0), (1.0, -1.0), (0.0, 0.0), dt)
        self.assertAlmostEqual(act.rotor[0] / 100.0, 1 - math.exp(-1), delta=0.02)
        for _ in range(50):
            act.step(meta, (100.0, 100.0), (1.0, -1.0), (0.0, 0.0), dt)
        vmax = math.radians(meta["vanes"]["max_deflection_deg"])
        self.assertAlmostEqual(act.vane[0], vmax)  # 1 rad command clamps to max deflection
        act2 = ActuatorState()
        act2.step(meta, (0, 0), (vmax, 0), (0, 0), dt)
        self.assertAlmostEqual(act2.vane[0], math.radians(meta["vanes"]["servo_rate_deg_s"]) * dt)


if __name__ == "__main__":
    unittest.main()
