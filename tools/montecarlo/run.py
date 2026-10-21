"""Monte Carlo scenario runner (inside WSL).

python3 -m tools.montecarlo.run --scenario transition --n 20 --parallel 2 --speed 2 --seed 1
python3 -m tools.montecarlo.run --scenario all --n 2            # smoke run of every scenario

Writes runs/montecarlo/<UTC time>_<scenario>/results.jsonl (one line per run, appended as
runs finish) and summary.md. Each run uses its own RNG seed (seed + index), so any run can
be repeated with --only <index>.
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .scenarios import SCENARIOS
from .sim import Sim, SimError

ROOT = Path(__file__).resolve().parents[2]


def urban_spawn_pose() -> str:
    """An open, flat spot in Karabük between buildings: nearest building 15-40 m away."""
    from tools import worldgen
    from osgeo import gdal
    osm = json.loads((worldgen.DATA / "osm.json").read_text())
    lat0, lon0 = 41.1639, 32.6443
    cents = []
    for el in osm["elements"]:
        g = el.get("geometry") or []
        if "building" in el.get("tags", {}) and g:
            x, y = worldgen.enu(np.mean([q["lat"] for q in g]), np.mean([q["lon"] for q in g]), lat0, lon0)
            cents.append((float(x), float(y)))
    c = np.array(cents)
    dense = c[np.argsort(np.hypot(c[:, 0], c[:, 1]))]  # buildings by distance from the takeoff point
    ds = gdal.Open(str(worldgen.DATA / "dem.tif"))
    z = worldgen.read_band(ds)
    gt = ds.GetGeoTransform()
    r = 6378137.0

    def terrain(x, y):
        lat = lat0 + math.degrees(y / r)
        lon = lon0 + math.degrees(x / (r * math.cos(math.radians(lat0))))
        return z[int((lat - gt[3]) / gt[5]), int((lon - gt[0]) / gt[1])]

    z0 = terrain(0.0, 0.0)
    for bx, by in dense[200:2000:7]:      # inside the town, not its edge
        for ang in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            x, y = bx + 25 * np.cos(ang), by + 25 * np.sin(ang)
            d = np.hypot(c[:, 0] - x, c[:, 1] - y).min()
            if 15 <= d <= 40:
                h = [terrain(x + dx, y + dy) for dx in (-30, 0, 30) for dy in (-30, 0, 30)]
                if max(h) - min(h) < 3.0:
                    return f"{x:.1f},{y:.1f},{terrain(x, y) - z0 + 0.9:.2f},0,0,0"
    raise RuntimeError("no open urban spot found")


def worker(args):
    name, index, seed, speed, slot, out_dir, pose = args
    fn, opts = SCENARIOS[name]
    rng = np.random.default_rng(seed + index)
    sim = Sim(instance=slot, world=opts["world"], speed=speed, seed=seed + index,
              model_pose=pose if name == "urban_landing" else None,
              run_dir=Path(f"/tmp/mc/{name}_{index}"))
    t0 = time.time()
    rec = {"scenario": name, "index": index, "seed": seed + index}
    for attempt in (1, 2):  # a harness failure (not a vehicle failure) is retried once with the same seed
        rng = np.random.default_rng(seed + index)
        try:
            params, met, ok = fn(sim, rng)
            rec.update(params=params, metrics=met, passed=ok, attempts=attempt)
            rec.pop("error", None)
            rec.pop("trace", None)
            break
        except Exception as e:  # recorded as a harness error, never as a vehicle loss
            rec.update(error=f"{type(e).__name__}: {e}", trace=traceback.format_exc()[-800:], passed=False,
                       attempts=attempt)
            sim.stop()
    rec["wall_s"] = round(time.time() - t0, 1)
    with open(out_dir / "results.jsonl", "a", encoding="utf-8") as fh:  # O_APPEND: one line per run
        fh.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
    print(f"{name} #{index}: {'PASS' if rec.get('passed') else 'n/a' if rec.get('passed') is None else 'FAIL'} "
          f"({rec['wall_s']} s){' ' + rec['error'] if 'error' in rec else ''}", flush=True)
    log = sim.ulog()
    if log:
        dst = out_dir / "logs" / f"{name}_{index}.ulg"
        dst.parent.mkdir(exist_ok=True)
        dst.write_bytes(log.read_bytes())
    return rec


def summarize(out_dir: Path, records):
    lines = ["# Monte Carlo özeti", "", f"Klasör: `{out_dir}`", "",
             "| Senaryo | Koşu | Geçti | Kaldı | Kriter yok | Düzenek hatası | Ort. süre (s) |", "|---|---|---|---|---|---|---|"]
    for name in sorted({r["scenario"] for r in records}):
        rs = [r for r in records if r["scenario"] == name]
        err = sum("error" in r for r in rs)
        ok = sum(r.get("passed") is True for r in rs)
        none = sum(r.get("passed") is None and "error" not in r for r in rs)
        lines.append(f"| {name} | {len(rs)} | {ok} | {len(rs) - ok - none - err} | {none} | {err} | "
                     f"{np.mean([r['wall_s'] for r in rs]):.0f} |")
    lines += ["", "Kalan / hatalı koşular:", ""]
    for r in records:
        if r.get("passed") is False:
            lines.append(f"- {r['scenario']} #{r['index']} (seed {r['seed']}): "
                         f"{r.get('error') or json.dumps(r.get('params'), ensure_ascii=False)}")
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(prog="python3 -m tools.montecarlo.run")
    ap.add_argument("--scenario", default="all", choices=["all", *SCENARIOS])
    ap.add_argument("--n", type=int, default=2)
    ap.add_argument("--parallel", type=int, default=1)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--only", type=int, help="repeat a single run index")
    a = ap.parse_args()

    names = list(SCENARIOS) if a.scenario == "all" else [a.scenario]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = ROOT / "runs" / "montecarlo" / f"{stamp}_{a.scenario}"
    out_dir.mkdir(parents=True)
    pose = urban_spawn_pose() if "urban_landing" in names else None
    if pose:
        print("urban spawn pose:", pose)
    indices = [a.only] if a.only is not None else range(a.n)
    jobs = [(name, i, a.seed, a.speed, k % a.parallel, out_dir, pose)
            for k, (name, i) in enumerate((n, i) for n in names for i in indices)]
    records = []
    # one worker per slot so two runs never share a PX4 instance id
    with mp.get_context("spawn").Pool(a.parallel, maxtasksperchild=1) as pool:
        slots = [[j for j in jobs if j[4] == s] for s in range(a.parallel)]
        for recs in pool.map(_run_slot, slots):
            records += recs
    summarize(out_dir, records)
    print((out_dir / "summary.md").read_text(encoding="utf-8"))


def _run_slot(jobs):
    return [worker(j) for j in jobs]


if __name__ == "__main__":
    main()
