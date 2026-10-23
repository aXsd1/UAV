"""Keep what the analysis needs from Monte Carlo ULogs, then free the disk space they take.

python3 -m tools.montecarlo.compact_logs                 # summarise only (dry run for deletion)
python3 -m tools.montecarlo.compact_logs --delete        # also delete the .ulg of passed runs

For every set in runs/montecarlo, each logs/*.ulg gets one line in <set>/ulog_summary.jsonl:
file, scenario, index, passed, log duration and every PX4 message at WARNING or above (time,
level, text) -- what tools/montecarlo/ulog_messages.py reads from the full logs. With --delete the
.ulg of runs that passed is removed afterwards; logs of failed runs, harness errors and runs
missing from results.jsonl are kept for a closer look. Sets written to in the last --active-min
minutes are skipped, so a running set is not touched. results.jsonl (what --resume reads) and
summary.md are never changed.
"""

from __future__ import annotations

import argparse
import json
import re
import time
from multiprocessing import Pool
from pathlib import Path

from tools.montecarlo.ulog_messages import messages

ROOT = Path(__file__).resolve().parents[2]
RUNS = ROOT / "runs" / "montecarlo"


def summarise(path: Path) -> dict:
    from pyulog import ULog
    rec = {"file": path.name, "size_mb": round(path.stat().st_size / 1e6, 1)}
    try:
        u = ULog(str(path), message_name_filter_list=["vehicle_status"])
        rec["duration_s"] = round((u.last_timestamp - u.start_timestamp) / 1e6, 1)
    except Exception as ex:  # truncated or empty log
        rec["error"] = f"{type(ex).__name__}: {ex}"[:200]
    rec["messages"] = [[round(t, 1), lvl, text] for t, lvl, text in messages(path)]
    return rec


def results_by_index(set_dir: Path) -> dict:
    out = {}
    f = set_dir / "results.jsonl"
    if f.exists():
        for line in f.read_text(encoding="utf-8").splitlines():
            if line.strip():
                r = json.loads(line)
                out[(r.get("scenario"), r.get("index"))] = r
    return out


def main():
    ap = argparse.ArgumentParser(prog="python3 -m tools.montecarlo.compact_logs")
    ap.add_argument("--delete", action="store_true", help="delete the .ulg of passed runs after summarising")
    ap.add_argument("--active-min", type=float, default=30.0, help="skip sets written to this recently")
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--include-failed", action="store_true", help="with --delete, also delete logs of failed runs")
    ap.add_argument("--keep", nargs="*", default=[], help="logs never deleted, as <set>/<file>.ulg")
    ap.add_argument("--before", default=None, help="only sets whose UTC stamp sorts before this (e.g. 20261022T2336)")
    ap.add_argument("--skip", nargs="*", default=[], help="set directory names left alone")
    ap.add_argument("sets", nargs="*", help="set directory names (default: all)")
    a = ap.parse_args()

    now = time.time()
    total_freed = kept = 0
    for set_dir in sorted(p for p in RUNS.iterdir() if p.is_dir() and (not a.sets or p.name in a.sets)
                          and (a.before is None or p.name < a.before) and p.name not in a.skip):
        logs = sorted((set_dir / "logs").glob("*.ulg"))
        if not logs:
            continue
        newest = max([p.stat().st_mtime for p in set_dir.rglob("*")] + [set_dir.stat().st_mtime])
        if now - newest < a.active_min * 60:
            print(f"{set_dir.name}: written {int((now - newest) / 60)} min ago, skipped")
            continue
        out = set_dir / "ulog_summary.jsonl"
        done = set()
        if out.exists():
            done = {json.loads(line)["file"] for line in out.read_text(encoding="utf-8").splitlines() if line.strip()}
        results = results_by_index(set_dir)
        todo = [p for p in logs if p.name not in done]
        with Pool(a.jobs) as pool, open(out, "a", encoding="utf-8") as f:
            for rec in pool.imap(summarise, todo):
                m = re.match(r"(.+)_(\d+)\.ulg$", rec["file"])
                r = results.get((m.group(1), int(m.group(2)))) if m else None
                rec["scenario"], rec["index"] = (m.group(1), int(m.group(2))) if m else (None, None)
                rec["passed"] = r.get("passed") if r and "error" not in r else None
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                f.flush()
        summaries = {json.loads(line)["file"]: json.loads(line)
                     for line in out.read_text(encoding="utf-8").splitlines() if line.strip()}
        freed = 0
        for p in logs:
            s = summaries.get(p.name)
            if s is None:
                continue
            protected = f"{set_dir.name}/{p.name}" in a.keep
            if a.delete and not protected and (s.get("passed") is True or a.include_failed):
                freed += p.stat().st_size
                p.unlink()
            else:
                kept += 1
        total_freed += freed
        print(f"{set_dir.name}: {len(logs)} logs, {len(todo)} summarised"
              + (f", {freed / 1e9:.1f} GB freed" if a.delete else ""), flush=True)
    print(f"total: {total_freed / 1e9:.1f} GB freed, {kept} logs kept")


if __name__ == "__main__":
    main()
