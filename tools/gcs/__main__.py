"""Browser ground control station for PX4 SITL (run inside WSL, next to the simulation).

python3 -m tools.gcs                     # http://localhost:8080 (also from Windows)
python3 -m tools.gcs --port 8081 --mavlink udpin:0.0.0.0:14540 --model vtol_p1_0

Telemetry and commands go over MAVLink (PX4's onboard link, UDP 14540, so QGroundControl can keep
14550); the camera image and the wind come from / go to Gazebo transport.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from tools.gcs.vehicle import Vehicle

STATIC = Path(__file__).resolve().parent / "static"


SERVOS = 5  # VtolDynamics: 0 vane pitch, 1 vane yaw, 2 left elevon, 3 right elevon, 4 tail elevator


class Gazebo:
    """Camera, wind and actuator commands over Gazebo transport; absent (None) when the bindings
    are missing."""

    def __init__(self, model: str, camera_topic: str):
        from gz.msgs10.actuators_pb2 import Actuators
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.vector3d_pb2 import Vector3d
        from gz.transport13 import Node

        from tools.gcs.camera import Camera

        self.Vector3d, self.Double = Vector3d, Double
        self.node = Node()
        self.wind_pub = self.node.advertise(f"/model/{model}/wind", Vector3d)
        self.turb_pub = self.node.advertise(f"/model/{model}/turbulence", Double)
        self.camera = Camera(camera_topic)
        self.wind = {"speed": 0.0, "from_deg": 0.0, "turbulence": 0.0}
        # What PX4's gz_bridge commands (the plugin adds servo rate limits and motor lag on top).
        self.servo_rad = [None] * SERVOS
        self.motor_rad_s: list = []
        self.t_act = 0.0
        for i in range(SERVOS):
            self.node.subscribe(Double, f"/model/{model}/servo_{i}", self._servo_cb(i))
        self.node.subscribe(Actuators, f"/{model}/command/motor_speed", self._on_motors)

    def _servo_cb(self, i):
        def cb(msg):
            self.servo_rad[i] = msg.data
            self.t_act = time.time()
        return cb

    def _on_motors(self, msg):
        self.motor_rad_s = list(msg.velocity)
        self.t_act = time.time()

    def actuators(self):
        if time.time() - self.t_act > 2.0:
            return None
        return {"servo_deg": [None if v is None else math.degrees(v) for v in self.servo_rad],
                "motor_rpm": [v * 60 / (2 * math.pi) for v in self.motor_rad_s]}

    def set_wind(self, speed: float, from_deg: float, turbulence: float):
        # "from" direction (meteorological), world ENU: air moves towards from_deg + 180
        to = math.radians(from_deg + 180.0)
        v = self.Vector3d()
        v.x, v.y, v.z = speed * math.sin(to), speed * math.cos(to), 0.0
        self.wind_pub.publish(v)
        t = self.Double()
        t.data = turbulence
        self.turb_pub.publish(t)
        self.wind = {"speed": speed, "from_deg": from_deg, "turbulence": turbulence}


def make_handler(vehicle: Vehicle, gz: Gazebo | None):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # quiet
            pass

        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj).encode())

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                return self._send(200, (STATIC / "index.html").read_bytes(), "text/html; charset=utf-8")
            if path == "/api/stream":
                return self._stream()
            if path == "/camera.mjpg":
                return self._mjpeg()
            self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._json(400, {"ok": False, "msg": "bad json"})
            if self.path == "/api/sticks":
                vehicle.set_sticks(body)
                return self._json(200, {"ok": True})
            if self.path == "/api/wind":
                if gz is None:
                    return self._json(200, {"ok": False, "msg": "Gazebo bağlantısı yok"})
                gz.set_wind(float(body.get("speed", 0)), float(body.get("from_deg", 0)),
                            float(body.get("turbulence", 0)))
                return self._json(200, {"ok": True, "msg": "Rüzgâr ayarlandı"})
            if self.path == "/api/cmd":
                try:
                    return self._json(200, {"ok": True, "msg": vehicle.command(body)})
                except Exception as e:  # report to the page, keep serving
                    return self._json(200, {"ok": False, "msg": str(e)})
            self._json(404, {"ok": False})

        def _stream(self):
            """Server-sent events: vehicle state at 10 Hz."""
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            since = 0
            try:
                while True:
                    s = vehicle.snapshot(since)
                    if s["log"]:
                        since = s["log"][-1]["seq"]
                    s["camera"] = bool(gz and gz.camera.alive())
                    s["wind_set"] = gz.wind if gz else None
                    s["act"] = gz.actuators() if gz else None
                    self.wfile.write(b"data: " + json.dumps(s).encode() + b"\n\n")
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
            self.close_connection = True

        def _mjpeg(self):
            if gz is None:
                return self._send(503, b"no camera", "text/plain")
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            try:
                for jpg in gz.camera.frames():
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            self.close_connection = True

    return Handler


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python3 -m tools.gcs")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--mavlink", default="udpin:0.0.0.0:14540")
    p.add_argument("--model", default="vtol_p1_0", help="Gazebo model name (wind topic)")
    p.add_argument("--camera-topic", default="/vtol/camera")
    a = p.parse_args(argv)

    vehicle = Vehicle(a.mavlink)
    try:
        gz = Gazebo(a.model, a.camera_topic)
    except Exception as e:  # e.g. running outside WSL without the gz Python bindings
        print(f"Gazebo transport unavailable ({e}); no camera or wind control", file=sys.stderr)
        gz = None
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(vehicle, gz))
    srv.daemon_threads = True
    print(f"Yer kontrol arayüzü: http://localhost:{a.port}  (MAVLink {a.mavlink})", flush=True)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        srv.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
