"""Build a Gazebo world from Copernicus DEM GLO-30 + OpenStreetMap (run inside WSL).

python3 tools/worldgen.py fetch  --bbox 41.145 32.580 41.235 32.700   # downloads into world/data/
python3 tools/worldgen.py build  --bbox ... --origin LAT LON --name karabuk

fetch: the DEM window is read straight from the cloud-optimised GeoTIFF on AWS (only the
bytes of the window are transferred), OSM buildings/roads come from the Overpass API.
build: terrain heightmap (16-bit PNG), one OBJ mesh for all buildings, one for roads, and
a world SDF whose spherical_coordinates put the Gazebo origin at --origin, so PX4's GNSS
sees the real location. Heights are relative to the terrain at the origin (origin z = 0).
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import urllib.parse
import urllib.request
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "world" / "data"
DEM_URL = ("https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_N{lat:02d}_00_E{lon:03d}_00_DEM/"
           "Copernicus_DSM_COG_10_N{lat:02d}_00_E{lon:03d}_00_DEM.tif")
OVERPASS = "https://overpass-api.de/api/interpreter"
HEIGHTMAP_PX = 513          # Gazebo wants 2^n + 1
TERRAIN_MESH_N = 321        # terrain mesh grid (~31 m over 10 km, the DEM's own resolution)
DEFAULT_BUILDING_H = 9.0    # m, ~3 floors when OSM has no height
FLOOR_H = 3.0


# --- geodesy: local tangent plane (east, north) around the origin -------------------------
def enu(lat, lon, lat0, lon0):
    """Equirectangular approximation; < 1 cm error over 10 km at this latitude scale."""
    r = 6378137.0
    x = np.radians(np.asarray(lon) - lon0) * r * math.cos(math.radians(lat0))
    y = np.radians(np.asarray(lat) - lat0) * r
    return x, y


# --- fetch ---------------------------------------------------------------------------------
def fetch(bbox):
    from osgeo import gdal
    s, w, n, e = bbox
    DATA.mkdir(parents=True, exist_ok=True)
    if math.floor(s) != math.floor(n) or math.floor(w) != math.floor(e):
        raise SystemExit("bbox spans several 1-degree DEM tiles; not supported yet")
    url = "/vsicurl/" + DEM_URL.format(lat=math.floor(s), lon=math.floor(w))
    out = DATA / "dem.tif"
    gdal.Translate(str(out), url, projWin=[w, n, e, s])  # WGS84 window, native 1 arc-second
    print(f"DEM window -> {out} ({out.stat().st_size / 1e6:.1f} MB)")

    q = (f"[out:json][timeout:180];(way[\"building\"]({s},{w},{n},{e});"
         f"way[\"highway\"]({s},{w},{n},{e}););out geom;")
    req = urllib.request.Request(OVERPASS, data=urllib.parse.urlencode({"data": q}).encode(),
                                 headers={"User-Agent": "uav-sim-worldgen/0.1"})
    with urllib.request.urlopen(req, timeout=300) as r:
        raw = r.read()
    (DATA / "osm.json").write_bytes(raw)
    print(f"OSM -> {DATA / 'osm.json'} ({len(raw) / 1e6:.1f} MB)")


# --- helpers -------------------------------------------------------------------------------
def read_band(ds) -> np.ndarray:
    """Band 1 as float64 via raw bytes: Ubuntu's GDAL numpy bridge breaks with pip numpy 2."""
    from osgeo import gdal
    band = ds.GetRasterBand(1)
    raw = band.ReadRaster(0, 0, ds.RasterXSize, ds.RasterYSize, buf_type=gdal.GDT_Float32)
    return np.frombuffer(raw, dtype=np.float32).reshape(ds.RasterYSize, ds.RasterXSize).astype(float)


def write_png16(path: Path, img: np.ndarray):
    """Grayscale 16-bit PNG without extra dependencies."""
    h, w = img.shape
    raw = b"".join(b"\x00" + img[y].astype(">u2").tobytes() for y in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 16, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    path.write_bytes(png)


def triangulate(poly):
    """Ear clipping for a simple polygon (list of (x, y), counter-clockwise). Returns index triples."""
    idx = list(range(len(poly)))
    tris = []

    def area2(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def inside(p, a, b, c):
        return area2(a, b, p) >= 0 and area2(b, c, p) >= 0 and area2(c, a, p) >= 0

    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if area2(a, b, c) <= 1e-9:
                continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            break  # degenerate polygon: keep what we have
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


def building_height(tags):
    for key in ("height", "building:height"):
        if key in tags:
            try:
                return float(tags[key].split()[0].replace(",", "."))
            except ValueError:
                pass
    if "building:levels" in tags:
        try:
            return float(tags["building:levels"]) * FLOOR_H
        except ValueError:
            pass
    return DEFAULT_BUILDING_H


ROAD_WIDTH = {"motorway": 14, "trunk": 12, "primary": 10, "secondary": 8, "tertiary": 7,
              "residential": 6, "unclassified": 5, "service": 4}


# --- build ---------------------------------------------------------------------------------
def build(bbox, origin, name):
    from osgeo import gdal
    lat0, lon0 = origin
    ds = gdal.Open(str(DATA / "dem.tif"))
    z = read_band(ds)
    gt = ds.GetGeoTransform()
    lons = gt[0] + (np.arange(z.shape[1]) + 0.5) * gt[1]
    lats = gt[3] + (np.arange(z.shape[0]) + 0.5) * gt[5]

    def terrain_at(lat, lon):
        i = np.clip(np.interp(lat, lats[::-1], np.arange(len(lats))[::-1]), 0, len(lats) - 1)
        j = np.clip(np.interp(lon, lons, np.arange(len(lons))), 0, len(lons) - 1)
        i0, j0 = np.floor(i).astype(int), np.floor(j).astype(int)
        i1, j1 = np.minimum(i0 + 1, len(lats) - 1), np.minimum(j0 + 1, len(lons) - 1)
        fi, fj = i - i0, j - j0
        return ((z[i0, j0] * (1 - fj) + z[i0, j1] * fj) * (1 - fi) + (z[i1, j0] * (1 - fj) + z[i1, j1] * fj) * fi)

    z0 = float(terrain_at(lat0, lon0))
    s, w, n, e = bbox
    xw, ys = enu(s, w, lat0, lon0)
    xe, yn = enu(n, e, lat0, lon0)
    size_x, size_y = float(xe - xw), float(yn - ys)
    cx, cy = (xw + xe) / 2, (ys + yn) / 2

    # Heightmap on a square grid covering the bbox (north up), heights relative to z0.
    grid_lat = np.linspace(n, s, HEIGHTMAP_PX)
    grid_lon = np.linspace(w, e, HEIGHTMAP_PX)
    gl, gn = np.meshgrid(grid_lat, grid_lon, indexing="ij")
    hz = terrain_at(gl, gn) - z0
    zmin, zmax = float(hz.min()), float(hz.max())
    out = ROOT / "world" / "build" / name
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    write_png16(out / "terrain.png", np.round((hz - zmin) / (zmax - zmin) * 65535).astype(np.uint16))

    # OSM -> meshes.
    osm_path = DATA / "osm.json"
    if osm_path.exists():
        osm = json.loads(osm_path.read_text(encoding="utf-8"))
    else:
        print("no OSM data (world/data/osm.json): terrain only")
        osm = {"elements": []}
    bv, bf, rv, rf = [], [], [], []
    n_buildings = n_roads = 0
    for el in osm.get("elements", []):
        geom = el.get("geometry")
        tags = el.get("tags", {})
        if not geom or len(geom) < 2:
            continue
        la = np.array([p["lat"] for p in geom])
        lo = np.array([p["lon"] for p in geom])
        x, y = enu(la, lo, lat0, lon0)
        if "building" in tags and len(geom) >= 4:
            pts = list(zip(x[:-1], y[:-1]))  # closed way: drop the repeated last node
            if len(pts) < 3:
                continue
            area = sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1] for i in range(len(pts)))
            if area < 0:  # make counter-clockwise (area computed with reversed order -> sign flipped)
                pts.reverse()
            base = float(np.min(terrain_at(la, lo))) - z0
            top = base + building_height(tags)
            k = len(bv)
            bv += [(px, py, base) for px, py in pts] + [(px, py, top) for px, py in pts]
            m = len(pts)
            for a, b_, c in triangulate(pts):
                bf.append((k + m + a, k + m + b_, k + m + c))
            for i in range(m):
                j = (i + 1) % m
                bf += [(k + i, k + j, k + m + j), (k + i, k + m + j, k + m + i)]
            n_buildings += 1
        elif "highway" in tags:
            wdt = ROAD_WIDTH.get(tags["highway"], 3.0) / 2
            hz_r = terrain_at(la, lo) - z0 + 0.15
            for i in range(len(x) - 1):
                dx, dy = x[i + 1] - x[i], y[i + 1] - y[i]
                L = math.hypot(dx, dy)
                if L < 1e-3:
                    continue
                nx, ny = -dy / L * wdt, dx / L * wdt
                k = len(rv)
                rv += [(x[i] + nx, y[i] + ny, hz_r[i]), (x[i] - nx, y[i] - ny, hz_r[i]),
                       (x[i + 1] - nx, y[i + 1] - ny, hz_r[i + 1]), (x[i + 1] + nx, y[i + 1] + ny, hz_r[i + 1])]
                rf += [(k, k + 1, k + 2), (k, k + 2, k + 3)]
            n_roads += 1

    def write_obj(path, verts, faces):
        # Per-face normals: Gazebo's DART/ODE mesh loader segfaults on an OBJ without them.
        v = np.asarray(verts, dtype=float).reshape(-1, 3)
        f_idx = np.asarray(faces, dtype=int).reshape(-1, 3)
        nrm = np.cross(v[f_idx[:, 1]] - v[f_idx[:, 0]], v[f_idx[:, 2]] - v[f_idx[:, 0]])
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
        with open(path, "w", encoding="ascii") as f:
            f.writelines(f"v {vx:.2f} {vy:.2f} {vz:.2f}\n" for vx, vy, vz in v)
            f.writelines(f"vn {nx:.4f} {ny:.4f} {nz:.4f}\n" for nx, ny, nz in nrm)
            f.writelines(f"f {a + 1}//{k + 1} {b + 1}//{k + 1} {c + 1}//{k + 1}\n"
                         for k, (a, b, c) in enumerate(f_idx))

    # Terrain as a triangle mesh: with Gazebo's heightmap the DART collision surface sat ~9 m above
    # the visual one at the origin in SITL (the vehicle "landed" in mid-air), so collide with a mesh.
    t_lat = np.linspace(n, s, TERRAIN_MESH_N)
    t_lon = np.linspace(w, e, TERRAIN_MESH_N)
    tl, tn = np.meshgrid(t_lat, t_lon, indexing="ij")
    tx, ty = enu(tl, tn, lat0, lon0)
    tz = terrain_at(tl, tn) - z0
    tv = np.stack([tx.ravel(), ty.ravel(), tz.ravel()], axis=1)
    N = TERRAIN_MESH_N
    r, c = np.meshgrid(np.arange(N - 1), np.arange(N - 1), indexing="ij")
    i00 = (r * N + c).ravel(); i01 = i00 + 1; i10 = i00 + N; i11 = i10 + 1
    # rows run north -> south: (i00, i10, i01) is counter-clockwise seen from above
    tf = np.concatenate([np.stack([i00, i10, i01], 1), np.stack([i01, i10, i11], 1)])
    write_obj(out / "meshes" / "terrain.obj", tv, tf)

    write_obj(out / "meshes" / "buildings.obj", bv, bf)
    write_obj(out / "meshes" / "roads.obj", rv, rf)

    # ODE aborts on an empty collision mesh: add the OSM models only when they have geometry.
    static_models = ""
    if bf:
        static_models += f"""    <model name="buildings">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><mesh><uri>file://{out}/meshes/buildings.obj</uri></mesh></geometry></collision>
        <visual name="visual"><geometry><mesh><uri>file://{out}/meshes/buildings.obj</uri></mesh></geometry>
          <material><ambient>0.75 0.72 0.68 1</ambient><diffuse>0.75 0.72 0.68 1</diffuse></material></visual>
      </link>
    </model>
"""
    if rf:
        static_models += f"""    <model name="roads">
      <static>true</static>
      <link name="link">
        <visual name="visual"><geometry><mesh><uri>file://{out}/meshes/roads.obj</uri></mesh></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.25 0.25 0.27 1</diffuse></material></visual>
      </link>
    </model>
"""

    sdf = f"""<?xml version="1.0"?>
<!-- Generated by tools/worldgen.py: Copernicus DEM GLO-30 + OpenStreetMap ({s}, {w}) - ({n}, {e}).
     Origin {lat0:.6f}, {lon0:.6f}, terrain {z0:.1f} m AMSL. Map data (c) OpenStreetMap contributors (ODbL);
     elevation (c) DLR e.V. 2010-2014 and (c) Airbus Defence and Space GmbH 2014-2018, Copernicus programme. -->
<sdf version="1.10">
  <world name="{name}">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene><grid>false</grid><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background><shadows>true</shadows></scene>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 1000 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat0:.7f}</latitude_deg>
      <longitude_deg>{lon0:.7f}</longitude_deg>
      <elevation>{z0:.1f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><mesh><uri>file://{out}/meshes/terrain.obj</uri></mesh></geometry></collision>
        <visual name="visual"><geometry><mesh><uri>file://{out}/meshes/terrain.obj</uri></mesh></geometry>
          <material><ambient>0.45 0.5 0.35 1</ambient><diffuse>0.45 0.5 0.35 1</diffuse></material></visual>
      </link>
    </model>
{static_models}  </world>
</sdf>
"""
    (out / f"{name}.sdf").write_text(sdf, encoding="utf-8")
    # PX4 looks worlds up by name in its worlds directory.
    px4_worlds = Path.home() / "uav" / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "worlds"
    if px4_worlds.is_dir():
        (px4_worlds / f"{name}.sdf").write_text(sdf, encoding="utf-8")
        print(f"installed as PX4 world '{name}' (PX4_GZ_WORLD={name})")
    print(f"world -> {out / (name + '.sdf')}: terrain {size_x / 1000:.1f} x {size_y / 1000:.1f} km, "
          f"relief {zmin:.0f}..{zmax:.0f} m around the origin, {n_buildings} buildings, {n_roads} roads")


def main():
    p = argparse.ArgumentParser(prog="tools/worldgen.py")
    p.add_argument("cmd", choices=["fetch", "build"])
    p.add_argument("--bbox", nargs=4, type=float, metavar=("S", "W", "N", "E"),
                   default=[41.145, 32.580, 41.235, 32.700])
    p.add_argument("--origin", nargs=2, type=float, metavar=("LAT", "LON"))
    p.add_argument("--name", default="karabuk")
    a = p.parse_args()
    if a.cmd == "fetch":
        fetch(a.bbox)
    else:
        s, w, n, e = a.bbox
        build(a.bbox, a.origin or ((s + n) / 2, (w + e) / 2), a.name)


if __name__ == "__main__":
    main()
