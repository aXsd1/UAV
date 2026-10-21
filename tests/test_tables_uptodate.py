"""The committed models/aero/<name>/ tables must match what the generator produces now."""

import tempfile
import unittest
from pathlib import Path

from tools.aerogen import config
from tools.aerogen.export import generate

ROOT = Path(__file__).resolve().parents[1]


class TablesUpToDate(unittest.TestCase):
    def test_p1(self):
        committed = ROOT / "models" / "aero" / "p1"
        cfg = config.load(ROOT / "config" / "p1.toml")
        with tempfile.TemporaryDirectory() as tmp:
            generate(cfg, tmp)
            fresh = sorted(p.name for p in Path(tmp).iterdir())
            self.assertEqual(fresh, sorted(p.name for p in committed.iterdir()))
            for name in fresh:
                a = (Path(tmp) / name).read_text(encoding="utf-8").replace("\r\n", "\n")
                b = (committed / name).read_text(encoding="utf-8").replace("\r\n", "\n")
                self.assertEqual(a, b, f"{name} is stale: run python -m tools.aerogen")


if __name__ == "__main__":
    unittest.main()
