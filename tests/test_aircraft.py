import copy
import math
import unittest
from pathlib import Path

import numpy as np

from tools.aerogen import config
from tools.aerogen.aircraft import Aircraft, mass_properties
from tools.aerogen.sizing import analyse, cm_about_cg

CFG = config.load(Path(__file__).resolve().parents[1] / "config" / "p1.toml")


class Config(unittest.TestCase):
    def test_mass_must_sum_to_mtow(self):
        bad = copy.deepcopy(CFG)
        bad["mass"]["components"][0]["mass_kg"] += 0.5
        with self.assertRaises(config.ConfigError):
            config.validate(bad)

    def test_hash_recorded(self):
        self.assertEqual(len(CFG["_source"]["sha256"]), 64)


class MassProps(unittest.TestCase):
    def test_single_point_mass(self):
        cfg = {"mass": {"components": [{"name": "a", "mass_kg": 2.0, "station_m": 0.3, "size_m": [0, 0, 0]}]}}
        mp = mass_properties(cfg)
        self.assertEqual(mp.cg_station, 0.3)
        self.assertEqual(mp.inertia, (0.0, 0.0, 0.0))

    def test_two_masses_parallel_axis(self):
        cfg = {"mass": {"components": [
            {"name": "a", "mass_kg": 1.0, "station_m": 0.0, "size_m": [0, 0, 0]},
            {"name": "b", "mass_kg": 1.0, "station_m": 1.0, "size_m": [0, 0, 0]},
        ]}}
        mp = mass_properties(cfg)
        self.assertAlmostEqual(mp.cg_station, 0.5)
        self.assertAlmostEqual(mp.inertia[1], 0.5)  # 2 * 1 * 0.5^2
        self.assertAlmostEqual(mp.inertia[2], 0.5)

    def test_p1_inertia_plausible(self):
        ixx, iyy, izz = mass_properties(CFG).inertia
        self.assertTrue(0 < ixx < iyy < izz)
        # Perpendicular-axis sanity: Izz <= Ixx + Iyy for a flat-ish airframe.
        self.assertLessEqual(izz, ixx + iyy + 1e-9)


class Airframe(unittest.TestCase):
    def setUp(self):
        self.ac = Aircraft(CFG)

    def test_reference_move_preserves_forces_and_shifts_moment(self):
        alpha = np.radians(np.arange(-180, 181, 5.0))
        moved = copy.deepcopy(CFG)
        moved["aero"]["reference_station_m"] += 0.1
        ac2 = Aircraft(moved)
        cl1, cd1, _ = self.ac.longitudinal(alpha)
        cl2, cd2, _ = ac2.longitudinal(alpha)
        np.testing.assert_allclose(cl1, cl2)
        np.testing.assert_allclose(cd1, cd2)
        # Both references give the same moment about any third point.
        np.testing.assert_allclose(cm_about_cg(self.ac, alpha, 0.6)[2], cm_about_cg(ac2, alpha, 0.6)[2], atol=1e-12)

    def test_pitch_stable_at_cruise(self):
        a = np.radians([6.0, 10.0])
        _, _, cm = cm_about_cg(self.ac, a, mass_properties(CFG).cg_station)
        self.assertLess(cm[1] - cm[0], 0)

    def test_weathercock_stable(self):
        _, _, cn = self.ac.lateral(np.radians([-2.0, 2.0]))
        self.assertGreater(cn[1] - cn[0], 0)

    def test_side_force_opposes_sideslip(self):
        cy, _, _ = self.ac.lateral(np.radians([10.0]))
        self.assertLess(cy[0], 0)

    def test_derivative_signs(self):
        d = self.ac.derivatives(0.8)
        self.assertLess(d["Cmq"], 0)
        self.assertLess(d["Clp"], 0)
        self.assertLess(d["Cnr"], 0)
        self.assertGreater(d["CL_elevon_sym"], 0)
        self.assertGreater(d["Cl_elevon_diff"], 0)


class Sizing(unittest.TestCase):
    """Doc targets the configuration is tuned to meet. Other checks are reported, not asserted."""

    @classmethod
    def setUpClass(cls):
        cls.r = analyse(CFG)

    def test_cruise_cl_matches_doc(self):
        self.assertAlmostEqual(self.r["cruise"]["cl"], CFG["requirements"]["cruise_cl_doc"], delta=0.02)

    def test_cruise_below_stall(self):
        self.assertLess(self.r["cruise"]["alpha_rad"], math.radians(CFG["wing"]["stall_alpha_deg"] - 3))

    def test_static_margin_in_band(self):
        req = CFG["requirements"]
        self.assertGreaterEqual(self.r["static_margin"], req["static_margin_min"])
        self.assertLessEqual(self.r["static_margin"], req["static_margin_max"])


if __name__ == "__main__":
    unittest.main()
