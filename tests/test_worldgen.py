import math
import struct
import tempfile
import unittest
import zlib
from pathlib import Path

import numpy as np

from tools.worldgen import building_height, ccw, enu, triangulate, write_png16


def poly_area(pts):
    return 0.5 * sum(pts[i - 1][0] * pts[i][1] - pts[i][0] * pts[i - 1][1] for i in range(len(pts)))


class Triangulate(unittest.TestCase):
    def check(self, pts):
        tris = triangulate(pts)
        self.assertEqual(len(tris), len(pts) - 2)
        covered = sum(abs(poly_area([pts[a], pts[b], pts[c]])) for a, b, c in tris)
        self.assertAlmostEqual(covered, abs(poly_area(pts)), places=6)

    def test_clockwise_outline_is_reordered(self):
        # OSM ways come in either direction; a clockwise outline gave buildings no roofs
        cw = [(0, 1), (1, 1), (1, 0), (0, 0)]
        self.assertGreater(poly_area(ccw(cw)), 0)
        self.check(ccw(cw))
        self.assertEqual(ccw(cw[::-1]), cw[::-1])

    def test_square(self):
        self.check([(0, 0), (1, 0), (1, 1), (0, 1)])

    def test_l_shape_is_not_convex(self):
        self.check([(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)])

    def test_u_shape(self):
        self.check([(0, 0), (3, 0), (3, 3), (2, 3), (2, 1), (1, 1), (1, 3), (0, 3)])


class Geodesy(unittest.TestCase):
    def test_one_km_north_and_east(self):
        lat0, lon0 = 41.19, 32.64
        x, y = enu(lat0 + 1000 / 111320.0, lon0, lat0, lon0)
        self.assertAlmostEqual(float(y), 1000.0, delta=2.0)
        dlon = 1000 / (111320.0 * math.cos(math.radians(lat0)))
        x, y = enu(lat0, lon0 + dlon, lat0, lon0)
        self.assertAlmostEqual(float(x), 1000.0, delta=2.0)


class Png(unittest.TestCase):
    def test_roundtrip_16bit(self):
        img = (np.arange(12, dtype=np.uint16).reshape(3, 4) * 5000)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.png"
            write_png16(p, img)
            b = p.read_bytes()
        self.assertEqual(b[:8], b"\x89PNG\r\n\x1a\n")
        w, h, depth, ctype = struct.unpack(">IIBB", b[16:26])
        self.assertEqual((w, h, depth, ctype), (4, 3, 16, 0))
        idat_len = struct.unpack(">I", b[33:37])[0]
        raw = zlib.decompress(b[41:41 + idat_len])
        rows = [np.frombuffer(raw[r * 9 + 1:(r + 1) * 9], dtype=">u2") for r in range(3)]
        np.testing.assert_array_equal(np.vstack(rows), img)


class BuildingHeight(unittest.TestCase):
    def test_tags(self):
        self.assertEqual(building_height({"height": "12 m"}), 12.0)
        self.assertEqual(building_height({"building:levels": "5"}), 15.0)
        self.assertEqual(building_height({"building": "yes"}), 9.0)


if __name__ == "__main__":
    unittest.main()
