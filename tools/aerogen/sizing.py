"""Design checks: compares the hand-calc models with the design-doc targets."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .aircraft import Aircraft, mass_properties
from .propulsion import CoaxialDuctedFan, ExitVanes


@dataclass
class Check:
    name: str
    value: str
    target: str
    ok: bool | None  # None = informational


def cm_about_cg(ac: Aircraft, alpha, cg_station):
    """Move the reference-station pitching moment to the CG."""
    cl, cd, cm = ac.longitudinal(alpha)
    cz = -cl * np.cos(alpha) - cd * np.sin(alpha)
    # M_cg = sum (s_i - s_cg) Fz_i = M_ref - (s_cg - s_ref) Fz
    return cl, cd, cm - (cg_station - ac.s_ref) / ac.c * cz


def cruise_trim(ac: Aircraft, cl_req: float):
    """Alpha where the airframe gives the required CL (attached branch, bisection)."""
    lo, hi = math.radians(-5.0), ac.wing.alpha_stall
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if float(ac.longitudinal(mid)[0]) < cl_req:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def analyse(cfg: dict) -> dict:
    env, req = cfg["environment"], cfg["requirements"]
    rho, g = env["air_density_kg_m3"], env["gravity_m_s2"]
    ac = Aircraft(cfg)
    fan = CoaxialDuctedFan.from_config(cfg)
    vanes = ExitVanes.from_config(cfg)
    mp = mass_properties(cfg)
    weight = mp.mass * g
    eta_motor = cfg["motors"]["efficiency"]

    # Cruise
    v = req["cruise_speed_m_s"]
    q = 0.5 * rho * v * v
    cl_req = weight / (q * ac.S)
    a_trim = cruise_trim(ac, cl_req)
    cl_t, cd_t, _ = ac.longitudinal(a_trim)
    drag = float(cd_t) * q * ac.S
    rps_cruise = fan.rps_for_thrust(drag, v)
    p_cruise_shaft = fan.shaft_power(rps_cruise, v) if rps_cruise else float("nan")

    # Stall: CLmax of the whole airframe (attached branch) and the 1 g stall speed.
    a_scan = np.radians(np.arange(0.0, 30.0, 0.1))
    cl_max = float(np.max(ac.longitudinal(a_scan)[0]))
    v_stall = math.sqrt(2.0 * weight / (rho * ac.S * cl_max))

    # Cruise pitch trim: airframe Cm about the CG plus the duct momentum drag (the duct sits
    # ahead of the CG and sees the cross-flow V sin(alpha), a nose-up moment).
    _, _, cm_trim0 = cm_about_cg(ac, a_trim, mp.cg_station)
    t_cruise = drag
    sa_ = fan.rho * fan.exit_area_ratio * fan.disk_area
    mdot_cruise = sa_ * fan.exit_velocity(t_cruise, v * math.cos(a_trim))
    m_md = mdot_cruise * v * math.sin(a_trim) * (mp.cg_station - cfg["duct"]["center_station_m"])
    cm_trim = float(cm_trim0) + m_md / (q * ac.S * ac.c)

    # Static margin from d(Cm)/d(CL) about the CG at the trim point.
    da = math.radians(0.5)
    cl1, _, cm1 = cm_about_cg(ac, a_trim - da, mp.cg_station)
    cl2, _, cm2 = cm_about_cg(ac, a_trim + da, mp.cg_station)
    static_margin = -float((cm2 - cm1) / (cl2 - cl1))

    # Hover
    rps_hover = fan.rps_for_thrust(weight)
    p_hover_shaft = fan.shaft_power(rps_hover)
    t_max = fan.unit_thrust(fan.max_rps)
    p_max_shaft = fan.shaft_power(fan.max_rps)
    p_motor_peak = cfg["motors"]["count"] * cfg["motors"]["peak_power_w"]

    # Energy: hover segments + cruise endurance against usable battery energy.
    bat = cfg["battery"]
    e_hover = p_hover_shaft / eta_motor * req["hover_time_per_sortie_s"] / 3600.0
    e_cruise = p_cruise_shaft / eta_motor * req["endurance_min"] / 60.0
    e_usable = bat["energy_wh"] * bat["usable_fraction"]
    cruise_minutes = (e_usable - e_hover) / (p_cruise_shaft / eta_motor) * 60.0
    v_nominal = 3.7 * bat["cells_series"]
    i_max = p_max_shaft / eta_motor / v_nominal
    i_limit = bat["max_continuous_c"] * bat["capacity_ah"]

    # Hover control authority vs a steady side wind (propeller-off airframe load only;
    # duct momentum drag is NOT included, so this is optimistic).
    v_w = req["hover_wind_m_s"]
    q_w = 0.5 * rho * v_w * v_w
    _, _, cm90 = cm_about_cg(ac, math.radians(90.0), mp.cg_station)
    m_wind_pitch = abs(float(cm90)) * q_w * ac.S * ac.c
    cy90_l, _, cn90_l = ac.lateral(math.radians(90.0))
    cn90_cg = float(cn90_l) + (mp.cg_station - ac.s_ref) * float(cy90_l) / ac.b
    m_wind_yaw = abs(cn90_cg) * q_w * ac.S * ac.b
    vane_arm = mp.cg_station - cfg["duct"]["exit_station_m"]
    delta_max = math.radians(cfg["vanes"]["max_deflection_deg"])
    m_vane_max = weight * float(vanes.side_force_ratio(delta_max)) * vane_arm

    # Tip-over. Hovering in wind the vehicle leans into it (duct momentum drag + airframe drag)
    # and must touch down inside its tip-over angle. The weathervane (WV_AXIS=1 patch) keeps
    # the wind on the span axis: the airframe is edge-on (beta = 90 deg) and the wind is on a
    # leg axis of the + gear, so the tipping lever is the full leg radius.
    lg = cfg["landing_gear"]
    s_tail = lg["tail_end_station_m"]
    h_cg = s_tail - mp.cg_station
    mdot_hover = rho * fan.exit_area_ratio * fan.disk_area * fan.exit_velocity(weight)
    cy90, _, cn90_ref = (float(x) for x in ac.lateral(math.radians(90.0)))
    f_wind = mdot_hover * v_w + q_w * ac.S * abs(cy90)
    lean = math.degrees(math.atan2(f_wind, weight))
    tip_angle = math.degrees(math.atan2(lg["radius_m"], h_cg))
    # On the ground, motors off: airframe wind moment about the leg contact vs weight * radius.
    m_ground = abs(cn90_ref * ac.b + (s_tail - ac.s_ref) * cy90) * q_w * ac.S
    ground_safety = weight * lg["radius_m"] / m_ground

    # Rotation about the thrust axis: +/-10 % rpm split between the rotors.
    q_hover = fan.rotor_torque(rps_hover)
    m_roll_split = q_hover * (1.1 ** 2 - 0.9 ** 2)

    derivs = ac.derivatives(float(cl_t))
    de_trim = math.degrees(-cm_trim / derivs["Cm_elevon_sym"])

    sm_lo, sm_hi = req["static_margin_min"], req["static_margin_max"]
    checks = [
        Check("Kütle toplamı", f"{mp.mass:.2f} kg", f"{cfg['mass']['mtow_kg']:.2f} kg", abs(mp.mass - cfg["mass"]["mtow_kg"]) < 1e-6),
        Check("Ağırlık merkezi istasyonu", f"{mp.cg_station:.3f} m", "bilgi", None),
        Check("Statik marj (seyir)", f"{static_margin * 100:.1f} %", f"{sm_lo * 100:.0f}–{sm_hi * 100:.0f} %", sm_lo <= static_margin <= sm_hi),
        Check("Tutunma hızı (CLmax)", f"{v_stall:.1f} m/s (CLmax {cl_max:.2f})",
              f"seyir {v:.0f} m/s ≥ {req['min_cruise_stall_margin']:.2f} × tutunma", v >= req["min_cruise_stall_margin"] * v_stall),
        Check("Seyirde gereken CL", f"{cl_req:.3f}", f"≈ {req['cruise_cl_doc']:.2f} (doküman)", abs(cl_req - req["cruise_cl_doc"]) < 0.03),
        Check("Seyir hücum açısı", f"{math.degrees(a_trim):.1f}°", f"< stall {math.degrees(ac.wing.alpha_stall):.0f}°", a_trim < ac.wing.alpha_stall - math.radians(3)),
        Check("Seyir trim elevonu (momentum sürüklemesi dahil)", f"{de_trim:+.1f}°",
              f"|δe| ≤ {req['max_cruise_trim_elevon_deg']:.0f}° (azami {cfg['elevons']['max_deflection_deg']:.0f}°)",
              abs(de_trim) <= req["max_cruise_trim_elevon_deg"]),
        Check("Seyir L/D", f"{float(cl_t / cd_t):.1f}", f"≈ {req['cruise_lift_to_drag_doc']:.0f} (doküman)", float(cl_t / cd_t) >= 0.85 * req["cruise_lift_to_drag_doc"]),
        Check("Seyir elektrik gücü", f"{p_cruise_shaft / eta_motor:.0f} W", f"≈ {req['cruise_power_doc_w']:.0f} W (doküman)", None),
        Check("Askı devri", f"{rps_hover * 60:.0f} rpm", f"< {fan.max_rps * 60:.0f} rpm", rps_hover < fan.max_rps),
        Check("Askı mil gücü", f"{p_hover_shaft:.0f} W", f"≈ {req['hover_power_doc_w']:.0f} W (doküman)", abs(p_hover_shaft - req["hover_power_doc_w"]) / req["hover_power_doc_w"] < 0.15),
        Check("Askı elektrik gücü", f"{p_hover_shaft / eta_motor:.0f} W", "bilgi", None),
        Check(f"T/W={cfg['rotors']['max_thrust_to_weight']} için mil gücü", f"{p_max_shaft:.0f} W", f"≤ motor tepe {p_motor_peak:.0f} W", p_max_shaft <= p_motor_peak),
        Check("Kanal çıkış hızı (askı)", f"{fan.exit_velocity(weight):.1f} m/s", "bilgi", None),
        Check("Görev enerjisi", f"{e_hover + e_cruise:.0f} Wh (askı {e_hover:.0f} + seyir {e_cruise:.0f})", f"≤ kullanılabilir {e_usable:.0f} Wh", e_hover + e_cruise <= e_usable),
        Check("Ulaşılabilir seyir süresi", f"{cruise_minutes:.0f} dk", f"≥ {req['endurance_min']:.0f} dk", cruise_minutes >= req["endurance_min"]),
        Check("Batarya akımı (T/W tepe, nominal gerilim)", f"{i_max:.0f} A @ {v_nominal:.1f} V", f"≤ {i_limit:.0f} A ({bat['max_continuous_c']:.0f}C)", i_max <= i_limit),
        Check(f"Vane yunuslama momenti vs {v_w:.0f} m/s yan rüzgâr", f"{m_vane_max:.1f} N·m (kol {vane_arm:.2f} m)", f"> 1,5 × {m_wind_pitch:.1f} N·m rüzgâr momenti", m_vane_max > 1.5 * m_wind_pitch),
        Check(f"Vane sapma momenti vs {v_w:.0f} m/s yan rüzgâr", f"{m_vane_max:.1f} N·m", f"> 1,5 × {m_wind_yaw:.1f} N·m rüzgâr momenti", m_vane_max > 1.5 * m_wind_yaw),
        Check(f"Askıda {v_w:.0f} m/s yan rüzgâr (kanat açıklığı rüzgârda) yatışı vs devrilme açısı", f"{lean:.1f}° / {tip_angle:.1f}° (ayak {lg['radius_m']:.2f} m, AM yüksekliği {h_cg:.2f} m)",
              f"devrilme ≥ yatış + {req['tip_over_margin_deg']:.0f}°", tip_angle >= lean + req["tip_over_margin_deg"]),
        Check(f"Yerde {v_w:.0f} m/s rüzgâr, motorlar kapalı", f"{weight * lg['radius_m']:.0f} / {m_ground:.0f} N·m = {ground_safety:.2f}",
              f"≥ {req['ground_tip_safety']:.1f}", ground_safety >= req["ground_tip_safety"]),
        Check("İtki ekseni etrafında moment (±%10 devir farkı)", f"{m_roll_split:.2f} N·m", "bilgi", None),
    ]

    return {
        "aircraft": ac, "fan": fan, "vanes": vanes, "mass": mp, "checks": checks,
        "derivatives": derivs,
        "cruise": {"cl": float(cl_t), "cd": float(cd_t), "alpha_rad": a_trim, "rpm": rps_cruise * 60 if rps_cruise else None,
                   "shaft_power_w": p_cruise_shaft},
        "hover": {"rpm": rps_hover * 60, "shaft_power_w": p_hover_shaft, "rotor_torque_nm": q_hover,
                  "vane_arm_m": vane_arm,
                  # Small-angle effectiveness at hover thrust, for the PX4 control allocation.
                  "pitch_yaw_moment_per_vane_rad_nm": weight * vanes.turning_efficiency * vane_arm,
                  # +d rpm on one rotor, -d rpm on the other; Q ~ n^2.
                  "thrust_axis_moment_per_rpm_split_nm": 2.0 * 2.0 * q_hover / (rps_hover * 60)},
        "static_margin": static_margin,
        "stall": {"cl_max": cl_max, "v_stall_m_s": v_stall},
        "max_unit_thrust_n": t_max,
    }


def report_markdown(cfg: dict, result: dict) -> str:
    lines = [
        f"# {cfg['meta']['name'].upper()} boyutlandırma raporu",
        "",
        f"`tools/aerogen` tarafından `{cfg['_source']['file']}` dosyasından üretildi; elle düzenlemeyin.",
        "Değerler el hesabıdır (pervane kapalı gövde aerodinamiği, kanallı momentum teorisi).",
        "",
        "| Kontrol | Değer | Hedef | Durum |",
        "|---|---|---|---|",
    ]
    for c in result["checks"]:
        status = "bilgi" if c.ok is None else ("GEÇTİ" if c.ok else "KALDI")
        lines.append(f"| {c.name} | {c.value} | {c.target} | {status} |")
    lines += [
        "",
        "Notlar:",
        "- Rüzgâr momenti kontrolü kanal momentum sürüklemesini içermez; gerçek yük daha büyüktür.",
        "- Askı gücü = ideal kanallı güç / FM × (1 + koaksiyel kayıp); FM ve kayıp config'ten.",
        "",
    ]
    return "\n".join(lines)
