"""Generate the Gazebo model (SDF) of the vehicle from the config and aero tables.

python -m tools.sdfgen --aero-dir /abs/path/models/aero/p1 [--out sim/gazebo/models/vtol_p1]

The link follows PX4's tailsitter convention: base_link is the multicopter
frame (FLU, +z towards the nose, -x towards the fixed-wing lift direction)
with its origin at the CG. Fixed-wing FRD -> link FLU: (x, y, z) -> (z, -y, x).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tools.aerogen import config

ROOT = Path(__file__).resolve().parents[1]
LIDAR_ABOVE_TAIL = 0.15  # m, lidar height above the leg tips (ground) when landed

SENSORS = """
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure><pressure><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></pressure></air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <magnetometer>
          <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
          <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
          <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
        </magnetometer>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>"""


def _box_geo(size):
    return f"<geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>"


def _box(name, pose, size, rgba):
    return (f'      <visual name="{name}_visual"><pose>{pose}</pose>{_box_geo(size)}'
            f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>\n")


def _box_collision(name, pose, size):
    return f'      <collision name="{name}"><pose>{pose}</pose>{_box_geo(size)}</collision>\n'


def _cyl(name, pose, radius, length, rgba):
    return (f'      <visual name="{name}_visual"><pose>{pose}</pose><geometry><cylinder><radius>{radius:.4f}</radius>'
            f"<length>{length:.4f}</length></cylinder></geometry>"
            f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>\n")


def _sphere_collision(name, xyz, r):
    return (f'      <collision name="{name}"><pose>{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f} 0 0 0</pose>'
            f"<geometry><sphere><radius>{r}</radius></sphere></geometry>"
            "<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>"
            "<contact><ode><kp>2e5</kp><kd>2e3</kd><max_vel>2</max_vel><min_depth>0.001</min_depth></ode></contact>"
            "</surface></collision>\n")


def build(cfg: dict, aero: dict, aero_dir: str) -> str:
    name = f"vtol_{cfg['meta']['name']}"
    s_cg = aero["mass"]["cg_station_m"]
    I = aero["mass"]["inertia_kg_m2"]
    # FW FRD inertia -> link FLU: x_flu = z_fw, y_flu = -y_fw, z_flu = x_fw.
    ixx, iyy, izz = I["Izz"], I["Iyy"], I["Ixx"]

    def z(station):  # station -> link z (towards the nose)
        return s_cg - station

    L = next(c for c in cfg["mass"]["components"] if c["name"] == "fuselage")["size_m"][0]
    t = cfg["tail"]
    fus_r = 0.07
    tip = cfg["landing_gear"]["radius_m"]
    z_tail = z(cfg["landing_gear"]["tail_end_station_m"])
    spawn_z = -z_tail + 0.03

    w = cfg["wing"]
    chord = w["area_m2"] / w["span_m"]
    fin_chord = t["fin_area_m2"] / t["fin_span_m"]
    d = cfg["duct"]
    grey, dark, blue = "0.6 0.6 0.62 1", "0.2 0.2 0.22 1", "0.15 0.3 0.6 0.6"

    vis = ""
    vis += _cyl("fuselage", f"0 0 {z(L / 2):.4f} 0 0 0", fus_r, L, grey)
    vis += _cyl("duct", f"0 0 {z(d['center_station_m']):.4f} 0 0 0", d["inner_diameter_m"] / 2 + 0.02, 0.25, blue)
    # Wing: span along y, chord along the nose axis (z), thin along x (lift direction).
    vis += _box("wing", f"0 0 {z(w['quarter_chord_station_m'] + chord / 4):.4f} 0 0 0", (0.04, w["span_m"], chord), dark)
    zf = z(t["quarter_chord_station_m"] + fin_chord / 4)
    for k, (dx, dy) in enumerate(((0, 1), (0, -1), (1, 0), (-1, 0))):
        cx, cy = dx * (fus_r + t["fin_span_m"] / 2), dy * (fus_r + t["fin_span_m"] / 2)
        size = (t["fin_span_m"], 0.02, fin_chord) if dx else (0.02, t["fin_span_m"], fin_chord)
        vis += _box(f"fin_{k}", f"{cx:.4f} {cy:.4f} {zf:.4f} 0 0 0", size, dark)

    # Landing-gear struts from the fin roots out to the leg tips.
    strut_len = tip - fus_r
    for k, (dx, dy) in enumerate(((0, 1), (0, -1), (1, 0), (-1, 0))):
        c = fus_r + strut_len / 2
        size = (strut_len, 0.015, 0.015) if dx else (0.015, strut_len, 0.015)
        vis += _box(f"strut_{k}", f"{dx * c:.4f} {dy * c:.4f} {z_tail + 0.01:.4f} 0 0 0", size, grey)

    col = ""
    for k, (x, y) in enumerate(((0, tip), (0, -tip), (tip, 0), (-tip, 0))):
        col += _sphere_collision(f"leg_{k}", (x, y, z_tail), 0.02)
    # Fuselage hull, kept clear of the ground so only the leg tips touch it when upright.
    col += _box_collision("fuselage_collision", f"0 0 {z(L / 2 - 0.05):.4f} 0 0 0", (0.12, 0.12, L - 0.2))

    return f"""<?xml version="1.0"?>
<!-- Generated by tools/sdfgen.py from config/{cfg['_source']['file']}; do not edit. -->
<sdf version="1.10">
  <model name="{name}">
    <pose>0 0 {spawn_z:.4f} 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>{aero['mass']['mass_kg']:.4f}</mass>
        <inertia><ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy:.6f}</iyy><iyz>0</iyz><izz>{izz:.6f}</izz></inertia>
      </inertial>
{vis}{col}{SENSORS}
    </link>

    <!-- Landing lidar at the tail end, looking along -z (down in hover). PX4's gz_bridge reads
         link "lidar_sensor_link" / sensor "lidar"; this pose makes it DOWNWARD_FACING. -->
    <link name="lidar_sensor_link">
      <pose relative_to="base_link">0 0 {z_tail + LIDAR_ABOVE_TAIL:.4f} 0 1.57 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <sensor name="lidar" type="gpu_lidar">
        <gz_frame_id>lidar_sensor_link</gz_frame_id>
        <pose>0 0 0 3.14 0 0</pose>
        <update_rate>50</update_rate>
        <ray>
          <scan><horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
                <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>
          <range><min>0.05</min><max>40.0</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.02</stddev></noise>
        </ray>
        <always_on>1</always_on>
      </sensor>
    </link>
    <joint name="lidar_sensor_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_sensor_link</child>
    </joint>

    <!-- No Gazebo air_speed sensor: VtolDynamics publishes the airspeed (it knows the wind). -->

    <plugin filename="VtolDynamics" name="vtol::VtolDynamics">
      <aero_dir>{aero_dir}</aero_dir>
      <link_name>base_link</link_name>
      <wind>0 0 0</wind>
      <motor_upper>0</motor_upper>
      <motor_lower>1</motor_lower>
      <servo_vane_pitch>0</servo_vane_pitch>
      <servo_vane_yaw>1</servo_vane_yaw>
      <servo_elevon_left>2</servo_elevon_left>
      <servo_elevon_right>3</servo_elevon_right>
      <servo_elevator>4</servo_elevator>
    </plugin>
  </model>
</sdf>
"""


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m tools.sdfgen")
    p.add_argument("--config", default=str(ROOT / "config" / "p1.toml"))
    p.add_argument("--aero-dir", required=True, help="absolute path the plugin will read at runtime")
    p.add_argument("--out", help="model directory (default sim/gazebo/models/vtol_<name>)")
    a = p.parse_args(argv)

    cfg = config.load(a.config)
    tables = ROOT / "models" / "aero" / cfg["meta"]["name"]
    aero = json.loads((tables / "aero.json").read_text(encoding="utf-8"))
    name = f"vtol_{cfg['meta']['name']}"
    out = Path(a.out) if a.out else ROOT / "sim" / "gazebo" / "models" / name
    out.mkdir(parents=True, exist_ok=True)
    (out / "model.sdf").write_text(build(cfg, aero, a.aero_dir), encoding="utf-8", newline="\n")
    (out / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>Ducted coaxial tail-sitter VTOL ({cfg['meta']['name'].upper()}), generated from config.</description>
</model>
""", encoding="utf-8", newline="\n")
    print(f"model written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
