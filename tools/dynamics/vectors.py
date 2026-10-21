"""Write reference cases for the C++ (and later PyTorch) ports of the force model.

python -m tools.dynamics.vectors  ->  sim/gazebo/vtol_dynamics/test/vectors.json
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .model import Inputs, VtolModel

ROOT = Path(__file__).resolve().parents[2]
AERO = ROOT / "models" / "aero" / "p1"
OUT = ROOT / "sim" / "gazebo" / "vtol_dynamics" / "test" / "vectors.json"


def cases(m: VtolModel):
    w_h = 2 * math.pi * m.meta["hover_allocation"]["rpm"] / 60.0
    yield "hover", dict(v_air=[0, 0, 0], omega=[0, 0, 0], rotor_speed=[w_h, w_h], vane=[0, 0], elevon=[0, 0])
    yield "hover_vanes", dict(v_air=[0, 0, 0], omega=[0, 0, 0], rotor_speed=[w_h, w_h],
                              vane=[math.radians(12), math.radians(-7)], elevon=[0, 0])
    yield "hover_split_spinup", dict(v_air=[0, 0, 0], omega=[0.1, -0.2, 0.3], rotor_speed=[1.05 * w_h, 0.95 * w_h],
                                     vane=[0, 0], elevon=[0, 0], rotor_accel=[300.0, -250.0])
    yield "hover_side_gust", dict(v_air=[0.5, -3.0, 6.0], omega=[0, 0, 0], rotor_speed=[w_h, w_h],
                                  vane=[math.radians(5), 0], elevon=[0, 0])
    trim = m.meta["cruise_trim"]
    a = math.radians(trim["alpha_deg"])
    w_c = 2 * math.pi * trim["rpm"] / 60.0
    yield "cruise", dict(v_air=[20 * math.cos(a), 0, 20 * math.sin(a)], omega=[0, 0, 0], rotor_speed=[w_c, w_c],
                         vane=[0, 0], elevon=[0, 0])
    yield "cruise_controls_rates", dict(v_air=[19.5, 1.2, 2.9], omega=[0.4, -0.3, 0.2], rotor_speed=[w_c, 0.9 * w_c],
                                       vane=[math.radians(3), math.radians(4)],
                                       elevon=[math.radians(8), math.radians(-4)])
    yield "transition_45deg", dict(v_air=[9.0, 0.5, 9.0], omega=[0, 0.5, 0], rotor_speed=[0.9 * w_h, 0.9 * w_h],
                                   vane=[math.radians(-10), 0], elevon=[math.radians(10), math.radians(10)])
    yield "reverse_flow", dict(v_air=[-6.0, 2.0, -1.0], omega=[0, 0, 0], rotor_speed=[w_h, w_h],
                               vane=[0, 0], elevon=[0, 0])
    yield "windmill", dict(v_air=[30.0, 0, 1.0], omega=[0, 0, 0], rotor_speed=[200.0, 200.0],
                           vane=[0, 0], elevon=[0, 0])
    yield "vane_past_stall", dict(v_air=[0, 0, 0], omega=[0, 0, 0], rotor_speed=[w_h, w_h],
                                  vane=[math.radians(40), math.radians(-40)], elevon=[0, 0])
    yield "motors_off_fall", dict(v_air=[0.0, 0.0, -10.0], omega=[0, 0, 0], rotor_speed=[0, 0],
                                  vane=[0, 0], elevon=[0, 0])


def main() -> None:
    out = []
    for cg_shift in (0.0, 0.015):
        m = VtolModel(AERO, cg_station=VtolModel(AERO).s_cg + cg_shift)
        for name, c in cases(m):
            c.setdefault("rotor_accel", [0.0, 0.0])
            r = m.wrench(Inputs(**{k: (np.array(v) if k in ("v_air", "omega") else tuple(v)) for k, v in c.items()}))
            out.append({"name": f"{name}{'_cg' if cg_shift else ''}", "cg_station": m.s_cg, "inputs": c,
                        "force": r.force.tolist(), "moment": r.moment.tolist()})
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(f"{len(out)} cases -> {OUT}")


if __name__ == "__main__":
    main()
