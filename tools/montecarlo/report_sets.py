"""Figures for the Monte Carlo report: every set in runs/montecarlo, grouped by scenario, seed and
variant (latest record per run index wins, so retried runs count once).

python -m tools.montecarlo.report_sets [--json out.json]

A "takeoff to N m not reached" record whose PX4 warnings show an attitude failure is the vehicle
tipping over at lift-off: counted as a vehicle loss, not as a harness error.
"""

from __future__ import annotations

import argparse
import glob
import json
import statistics as st
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def load():
    sets = defaultdict(dict)
    for f in sorted(glob.glob(str(ROOT / "runs" / "montecarlo" / "*" / "results.jsonl"))):
        if "eskidunya" in f:
            continue
        for line in Path(f).read_text(encoding="utf-8", errors="ignore").splitlines():
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            key = (r["scenario"], r["seed"] - r["index"], r.get("variant"))
            old = sets[key].get(r["index"])
            if old is None or "error" in old or "error" not in r:   # a real result beats an error
                sets[key][r["index"]] = r
    return sets


def tipped_at_takeoff(r) -> bool:
    return ("error" in r and "takeoff to" in r["error"]
            and any("Attitude failure" in w for w in r.get("px4_warnings", [])))


def q(a, p):
    a = sorted(a)
    return a[min(len(a) - 1, int(p * len(a)))] if a else float("nan")


def warn_counts(rs):
    c = Counter()
    for r in rs:
        for key, pat in (("compass_land_now", "Compass needs calibration"), ("failsafe", "Failsafe activated"),
                         ("quad_chute", "Quad-chute"), ("attitude_failure", "Attitude failure")):
            if any(pat in w for w in r.get("px4_warnings", [])):
                c[key] += 1
    c["with_warning_record"] = sum("px4_warnings" in r for r in rs)
    return dict(c)


def transition_stats(runs: dict, wind_range=(0, 8)):
    rs = list(runs.values())
    tip = [r for r in rs if tipped_at_takeoff(r)]
    harness = [r for r in rs if "error" in r and r not in tip]
    valid = [r for r in rs if "error" not in r]
    out = {"runs": len(rs), "valid": len(valid), "harness_errors": len(harness),
           "harness_error_kinds": Counter(r["error"][:60] for r in harness).most_common(5),
           "takeoff_tipover": [(r["index"], round(_wind(r, wind_range), 1)) for r in tip]}
    reasons = Counter()
    losses = []
    for r in valid:
        m = r["metrics"]
        fr, bk = m["front"], m["back"]
        if r["passed"]:
            reasons["pass"] += 1
        elif not fr.get("completed") or fr["duration_s"] >= 5:
            reasons["front_incomplete_or_slow"] += 1
        elif fr["alt_loss_m"] >= 5 and bk.get("completed") and m["landed"] and m["final_tilt_deg"] < 30:
            reasons["front_alt_loss_ge5"] += 1
        else:
            reasons["back_or_landing"] += 1
        if not m["landed"] or m["final_tilt_deg"] >= 30:
            losses.append({"index": r["index"], "wind": round(r["params"]["wind_m_s"], 1),
                           "front_loss": round(fr.get("alt_loss_m", float("nan")), 1),
                           "tilt": round(m["final_tilt_deg"], 0), "max_descent": round(m["max_descent_m_s"], 1),
                           "warnings": [w for w in r.get("px4_warnings", []) if "Preflight Fail: ESC" not in w
                                        and "Airspeed selector" not in w][-4:]})
    out["reasons"] = dict(reasons)
    out["passed"] = reasons["pass"]
    out["pass_rate"] = round(reasons["pass"] / max(1, len(valid)), 3)
    out["alt_loss_ge5"] = sum(r["metrics"]["front"].get("alt_loss_m", 0) >= 5 for r in valid)
    loss = [r["metrics"]["front"]["alt_loss_m"] for r in valid if r["metrics"]["front"].get("completed")]
    dur = [r["metrics"]["front"]["duration_s"] for r in valid if r["metrics"]["front"].get("completed")]
    out["front_loss"] = {"median": round(st.median(loss), 2), "p95": round(q(loss, .95), 2), "max": round(max(loss), 2)}
    out["front_time"] = {"median": round(st.median(dur), 2), "p95": round(q(dur, .95), 2), "max": round(max(dur), 2),
                         "ge5s": sum(d >= 5 for d in dur)}
    out["vehicle_losses_flight"] = losses
    out["vehicle_losses_total"] = len(losses) + len(tip)
    bins = {}
    for lo in range(0, 8, 2):
        b = [r for r in valid if lo <= r["params"]["wind_m_s"] < lo + 2]
        if b:
            bl = [r["metrics"]["front"].get("alt_loss_m", 0) for r in b]
            bins[f"{lo}-{lo + 2}"] = {"runs": len(b), "failed": sum(not r["passed"] for r in b),
                                      "median_loss": round(st.median(bl), 2),
                                      "tipover_takeoff": sum(lo <= _wind(r, wind_range) < lo + 2 for r in tip)}
    out["by_wind"] = bins
    fw = Counter(str(r["params"].get("faced_wind")) for r in valid)
    out["headwind_turn"] = dict(fw)
    by_fw = {}
    for k in ("True", "False", "None"):
        b = [r["metrics"]["front"]["alt_loss_m"] for r in valid if str(r["params"].get("faced_wind")) == k
             and r["metrics"]["front"].get("completed")]
        if b:
            by_fw[k] = {"runs": len(b), "median_loss": round(st.median(b), 2), "max": round(max(b), 2),
                        "ge5": sum(x >= 5 for x in b)}
    out["loss_by_turn"] = by_fw
    out["warnings"] = warn_counts(rs)
    return out


def _wind(r, wind_range):
    """Wind of a run that left no params (error record): first draw of its RNG, as in scenarios.py."""
    import numpy as np
    return float(np.random.default_rng(r["seed"]).uniform(*wind_range))


def other_stats(name, runs: dict):
    rs = list(runs.values())
    tip = [r for r in rs if tipped_at_takeoff(r)]
    harness = [r for r in rs if "error" in r and r not in tip]
    valid = [r for r in rs if "error" not in r]
    out = {"runs": len(rs), "valid": len(valid), "harness_errors": len(harness),
           "harness_error_kinds": Counter(r["error"][:60] for r in harness).most_common(3),
           "takeoff_tipover": len(tip),
           "passed": sum(r.get("passed") is True for r in valid),
           "failed": sum(r.get("passed") is False for r in valid),
           "no_criterion": sum(r.get("passed") is None for r in valid),
           "warnings": warn_counts(rs)}
    m = [r["metrics"] for r in valid]
    if name == "hover_wind":
        e = [x["hold_error_m"] for x in m]
        out["hold_error_m"] = {"median": round(st.median(e), 2), "p95": round(q(e, .95), 2), "max": round(max(e), 2)}
        out["fails"] = [(r["index"], round(r["params"]["wind_m_s"], 1), round(r["metrics"]["hold_error_m"], 2),
                         round(r["metrics"]["final_tilt_deg"])) for r in valid if not r["passed"]]
    if name == "actuator_failure":
        out["by_failure"] = {}
        for kind in sorted({r["params"]["failure"] for r in valid}):
            b = [r for r in valid if r["params"]["failure"] == kind]
            out["by_failure"][kind] = dict(Counter(r["metrics"]["outcome"] for r in b))
    if name == "link_loss":
        out["home_distance_m"] = {"median": round(st.median([x.get("home_distance_m", float("nan")) for x in m]), 2)}
        out["fails"] = [(r["index"], r["metrics"].get("rtl_after_s"), r["metrics"].get("home_distance_m"),
                         r["metrics"].get("landed"), round(r["metrics"]["final_tilt_deg"])) for r in valid if not r["passed"]]
    if name == "gnss_loss":
        d = [x["true_drift_m"] for x in m if x.get("true_drift_m") is not None]
        out["drift_m"] = {"median": round(st.median(d), 1), "p95": round(q(d, .95), 1), "max": round(max(d), 1)}
        out["by_outage"] = {}
        for o in sorted({r["params"]["outage_s"] for r in valid}):
            b = [r["metrics"]["true_drift_m"] for r in valid if r["params"]["outage_s"] == o]
            out["by_outage"][o] = {"runs": len(b), "median_drift_m": round(st.median(b), 1), "max": round(max(b), 1)}
    if name == "urban_landing":
        e = [x["landing_error_m"] for x in m]
        if e:
            out["landing_error_m"] = {"median": round(st.median(e), 2), "max": round(max(e), 2)}
        out["pass_winds"] = sorted(round(r["params"]["wind_m_s"], 1) for r in valid if r["passed"])
        out["tipover_winds"] = sorted(round(float(__import__("numpy").random.default_rng(r["seed"]).uniform(2, 8)), 1)
                                      for r in tip)
    return out


def main():
    ap = argparse.ArgumentParser(prog="python -m tools.montecarlo.report_sets")
    ap.add_argument("--json")
    a = ap.parse_args()
    sets = load()
    res = {}
    for (scn, seed, var), runs in sorted(sets.items(), key=lambda kv: str(kv[0])):
        if seed in (9000, 1) or len(runs) < 20:
            continue
        key = f"{scn} seed {seed}{' ' + var if var else ''}"
        res[key] = (transition_stats(runs, (6, 8) if seed == 3000 else (0, 8)) if scn == "transition"
                    else other_stats(scn, runs))
    text = json.dumps(res, indent=1, ensure_ascii=False, default=str)
    if a.json:
        Path(a.json).write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
