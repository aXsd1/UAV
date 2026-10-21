"""Wing / airfoil trade study:  python -m tools.studies.wing_study

Each variant is trimmed (tail incidence for zero cruise elevon) and balanced
(battery station for 10 % static margin), then flown at max(20 m/s, 1.25 V_stall).
Wing mass changes are NOT modelled.
"""

import copy
from pathlib import Path

from tools.aerogen import config
from tools.aerogen.sizing import analyse

BASE = config.load(Path(__file__).resolve().parents[2] / "config" / "p1.toml")


def check(r, name):
    return next(c for c in r["checks"] if c.name.startswith(name))


def solve_incidence(cfg):
    lo, hi = -12.0, 4.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        cfg["tail"]["horizontal_incidence_deg"] = mid
        de = float(check(analyse(cfg), "Seyir trim").value.replace("°", ""))
        # de has the sign of the nose-down moment; more negative incidence -> more nose-up
        if de > 0:
            lo = mid
        else:
            hi = mid
    return mid


def balance(cfg, target=0.10):
    bi = [c["name"] for c in cfg["mass"]["components"]].index("battery")
    lo, hi = 0.30, 0.70
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        cfg["mass"]["components"][bi]["station_m"] = mid
        if analyse(cfg)["static_margin"] > target:
            lo = mid
        else:
            hi = mid
    return mid


def run(label, cfg, speed=None):
    v_stall = analyse(cfg)["stall"]["v_stall_m_s"]
    v = speed or max(20.0, 1.25 * v_stall)
    cfg["requirements"]["cruise_speed_m_s"] = v
    for _ in range(4):
        inc = solve_incidence(cfg)
        bat = balance(cfg)
    r = analyse(cfg)
    ld = r["cruise"]["cl"] / r["cruise"]["cd"]
    print(f"{label:36s} V={v:4.1f}  Vs={v_stall:4.1f}  i_t={inc:+5.1f}°  bat={bat:.3f}  "
          f"SM={r['static_margin'] * 100:4.1f}%  L/D={ld:4.1f}  P={r['cruise']['shaft_power_w'] / 0.85:4.0f} W  "
          f"{check(r, 'Ulaşılabilir').value:>6s}")


def main():
    run("A kambur profil, 20 m/s (doküman)", copy.deepcopy(BASE), 20.0)
    run("A kambur profil, 1.25 Vs", copy.deepcopy(BASE))
    reflex = copy.deepcopy(BASE)
    reflex["wing"]["airfoil_cm0"] = -0.02
    reflex["wing"]["airfoil_zero_lift_alpha_deg"] = -2.0
    run("B reflex profil, 1.25 Vs", copy.deepcopy(reflex))
    for area, span in ((0.75, 1.5), (0.75, 1.7), (0.85, 1.7)):
        c = copy.deepcopy(reflex)
        c["wing"]["area_m2"] = area
        c["wing"]["span_m"] = span
        run(f"C reflex, S={area} m², b={span} m, 1.25 Vs", c)


if __name__ == "__main__":
    main()
