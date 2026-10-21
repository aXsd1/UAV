import math
import unittest

import numpy as np

from tools.aerogen.surfaces import (
    LiftingSurface,
    finite_wing_lift_slope,
    flap_cm_qc_per_rad,
    flap_effectiveness,
    stall_blend,
)


def symmetric_plate(**kw):
    args = dict(area=0.6, span=1.5, lift_slope=finite_wing_lift_slope(2 * math.pi, 3.75), cl0=0.0,
                cm0=0.0, cd0=0.01, oswald=0.8, alpha_stall=math.radians(15), blend_rate=50.0, cd90=1.2)
    args.update(kw)
    return LiftingSurface(**args)


class LiftSlope(unittest.TestCase):
    def test_tends_to_2d_slope_for_infinite_wing(self):
        self.assertAlmostEqual(finite_wing_lift_slope(2 * math.pi, 1e5), 2 * math.pi, places=3)

    def test_low_aspect_ratio_matches_slender_wing_limit(self):
        # Slender-wing theory: dCL/da -> pi*AR/2 as AR -> 0.
        ar = 0.05
        self.assertAlmostEqual(finite_wing_lift_slope(2 * math.pi, ar) / (math.pi * ar / 2), 1.0, places=2)

    def test_p1_wing_value(self):
        # AR 3.75, a0 = 6.0 -> about 3.68 per rad (hand calc in the design doc notes).
        self.assertAlmostEqual(finite_wing_lift_slope(6.0, 3.75), 3.68, delta=0.02)


class StallBlend(unittest.TestCase):
    def test_attached_near_zero_separated_far(self):
        a_s = math.radians(15)
        self.assertLess(stall_blend(0.0, a_s, 50.0), 1e-3)
        self.assertGreater(stall_blend(math.radians(40), a_s, 50.0), 0.999)
        self.assertGreater(stall_blend(math.radians(-40), a_s, 50.0), 0.999)
        self.assertGreater(stall_blend(math.radians(179), a_s, 50.0), 0.999)
        self.assertAlmostEqual(float(stall_blend(a_s, a_s, 50.0)), 0.5, delta=0.01)


class LiftingSurfaceModel(unittest.TestCase):
    def setUp(self):
        self.s = symmetric_plate()
        self.alpha = np.radians(np.arange(-180, 180.5, 0.5))

    def test_linear_region_slope(self):
        cl, _, _ = self.s.coefficients(np.radians([-2.0, 2.0]))
        self.assertAlmostEqual((cl[1] - cl[0]) / math.radians(4.0), self.s.lift_slope, places=3)

    def test_symmetric_surface_is_odd_in_alpha(self):
        cl, cd, cm = self.s.coefficients(self.alpha)
        cl_m, cd_m, cm_m = self.s.coefficients(-self.alpha)
        np.testing.assert_allclose(cl, -cl_m, atol=1e-9)
        np.testing.assert_allclose(cd, cd_m, atol=1e-9)
        np.testing.assert_allclose(cm, -cm_m, atol=1e-9)

    def test_flat_plate_at_90_deg(self):
        cl, cd, cm = self.s.coefficients(math.radians(90))
        self.assertAlmostEqual(float(cl), 0.0, places=6)
        self.assertAlmostEqual(float(cd), 0.01 + 1.2, places=6)
        # Centre of pressure at mid chord -> nose-down quarter-chord moment of CN * 0.25.
        self.assertAlmostEqual(float(cm), -1.2 * 0.25, places=6)

    def test_periodic_and_continuous(self):
        cl, cd, cm = self.s.coefficients(self.alpha)
        for c in (cl, cd, cm):
            self.assertAlmostEqual(c[0], c[-1], places=6)  # -180 == +180
            # 0.5 deg steps: no jumps bigger than a sharp stall would make.
            self.assertLess(np.max(np.abs(np.diff(c))), 0.1)

    def test_drag_positive_everywhere(self):
        _, cd, _ = self.s.coefficients(self.alpha)
        self.assertTrue(np.all(cd > 0))

    def test_stall_limits_lift(self):
        cl, _, _ = self.s.coefficients(self.alpha)
        cl_max_attached = self.s.lift_slope * self.s.alpha_stall
        self.assertLess(np.max(cl), cl_max_attached * 1.05)


class Flap(unittest.TestCase):
    def test_limits(self):
        self.assertAlmostEqual(flap_effectiveness(1.0), 1.0, places=9)   # whole plate rotates
        self.assertAlmostEqual(flap_effectiveness(0.0), 0.0, places=9)
        self.assertAlmostEqual(flap_cm_qc_per_rad(1.0), 0.0, places=9)

    def test_quarter_chord_flap(self):
        self.assertAlmostEqual(flap_effectiveness(0.25), 0.609, places=3)
        self.assertAlmostEqual(flap_cm_qc_per_rad(0.25), -0.650, places=3)


if __name__ == "__main__":
    unittest.main()
