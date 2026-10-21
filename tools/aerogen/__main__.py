"""python -m tools.aerogen [config/p1.toml] [--out models/aero/p1]"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import config
from .export import generate

ROOT = Path(__file__).resolve().parents[2]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m tools.aerogen", description=__doc__)
    p.add_argument("config", nargs="?", default=str(ROOT / "config" / "p1.toml"))
    p.add_argument("--out", help="output directory (default models/aero/<meta.name>)")
    args = p.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")  # Windows consoles default to a legacy code page

    cfg = config.load(args.config)
    out = Path(args.out) if args.out else ROOT / "models" / "aero" / cfg["meta"]["name"]
    res = generate(cfg, out)

    print(f"tables written to {out}")
    failed = [c for c in res["checks"] if c.ok is False]
    for c in res["checks"]:
        mark = " " if c.ok is None else ("+" if c.ok else "!")
        print(f" [{mark}] {c.name}: {c.value}  (hedef {c.target})")
    if failed:
        print(f"{len(failed)} design check(s) failed; see sizing_report.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
