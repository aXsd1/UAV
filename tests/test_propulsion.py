import math
import unittest
from pathlib import Path

import numpy as np

from tools.aerogen import config
from tools.aerogen.propulsion import CoaxialDuctedFan, ExitVanes

CFG = config.load(Path(__file__).resolve().parents[1] / "config" / "p1.toml")


class DuctedFan(unittest.TestCase):
    def setUp(self):
        self.fan = CoaxialDuctedFan.from_config(CFG)
        self.weight = CFG["mass"]["mtow_kg"] * CFG["environment"]["gravity_m_s2"]

    def test_max_static_thrust_meets_thrust_to_weight(self):
        t = self.fan.unit_thrust(self.fan.max_rps)
        self.assertAlmostEqual(t / self.weight, CFG["rotors"]["max_thrust_to_weight"], places=6)

    def test_hover_power_matches_doc_formula(self):
        # P = T^1.5 / sqrt(4 sigma rho A) / FM * (1 + coax loss), computed independently.
        r, d = CFG["rotors"], CFG["duct"]
        rho = CFG["environment"]["air_density_kg_m3"]
        area = math.pi / 4 * (d["inner_diameter_m"] ** 2 - d["hub_diameter_m"] ** 2)
        expected = self.weight ** 1.5 / math.sqrt(4 * d["exit_area_ratio"] * rho * area)
        expected *= (1 + r["coaxial_power_loss"]) / r["figure_of_merit"]
        rps = self.fan.rps_for_thrust(self.weight)
        self.assertAlmostEqual(self.fan.shaft_power(rps) / expected, 1.0, places=6)

    def test_open_rotor_comparison_shows_duct_gain(self):
        # Same thrust, sigma = 1 duct vs open rotor: ideal power ratio is 1/sqrt(2).
        rho, A, T = 1.225, self.fan.disk_area, self.weight
        p_open = T ** 1.5 / math.sqrt(2 * rho * A)
        p_duct = float(self.fan._ideal_unit_power(T, 0.0))
        self.assertAlmostEqual(p_duct / p_open, 1 / math.sqrt(2), places=9)

    def test_thrust_falls_with_advance_ratio(self):
        j = np.linspace(0, self.fan.j0, 50)
        self.assertTrue(np.all(np.diff(self.fan.ct(j)) < 0))
        self.assertAlmostEqual(float(self.fan.ct(self.fan.j0)), 0.0, places=12)

    def test_efficiency_bounded(self):
        j = np.linspace(0.05, 0.95 * self.fan.j0, 40)
        eta = self.fan.efficiency(j)
        self.assertTrue(np.all(eta > 0))
        self.assertTrue(np.all(eta < 1))

    def test_rpm_solver_roundtrip(self):
        for v in (0.0, 10.0, 20.0):
            rps = self.fan.rps_for_thrust(30.0, v)
            self.assertAlmostEqual(self.fan.unit_thrust(rps, v), 30.0, places=4)
        self.assertIsNone(self.fan.rps_for_thrust(10 * self.weight))

    def test_differential_rpm_gives_thrust_axis_torque(self):
        rps = self.fan.rps_for_thrust(self.weight)
        q_up = self.fan.rotor_torque(rps * 1.05)
        q_lo = self.fan.rotor_torque(rps * 0.95)
        self.assertGreater(q_up - q_lo, 0)
        self.assertAlmostEqual(self.fan.rotor_torque(rps) - self.fan.rotor_torque(rps), 0.0)


class Vanes(unittest.TestCase):
    def setUp(self):
        self.v = ExitVanes.from_config(CFG)

    def test_zero_and_odd(self):
        d = np.radians(np.arange(-30, 31))
        np.testing.assert_allclose(self.v.side_force_ratio(d), -self.v.side_force_ratio(-d), atol=1e-12)
        self.assertEqual(float(self.v.side_force_ratio(0.0)), 0.0)

    def test_saturates_past_stall_and_never_exceeds_thrust(self):
        big = self.v.side_force_ratio(math.radians(60))
        at_stall = self.v.side_force_ratio(self.v.stall)
        self.assertAlmostEqual(float(big), float(at_stall), places=12)
        self.assertLess(float(big), 1.0)

    def test_small_angle_slope(self):
        d = math.radians(1)
        self.assertAlmostEqual(float(self.v.side_force_ratio(d)) / d, self.v.turning_efficiency, places=3)


if __name__ == "__main__":
    unittest.main()
