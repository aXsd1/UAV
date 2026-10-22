"""One headless PX4 SITL + Gazebo instance for scenario runs (inside WSL).

Each instance gets its own PX4 instance id (ports, lock files), working directory and
Gazebo partition, so several can run in parallel. Waits are in simulation time, read
from Gazebo's /stats, so they hold when the requested speed factor is not reached.
"""

from __future__ import annotations

import glob
import math
import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path

WORK = Path(os.environ.get("WORK", str(Path.home() / "uav")))
PX4_DIR = WORK / "PX4-Autopilot"
BUILD = PX4_DIR / "build" / "px4_sitl_default"
PLUGIN_DIR = WORK / "build" / "vtol_dynamics"


class SimError(RuntimeError):
    pass


class Sim:
    def __init__(self, instance: int, world: str = "default", speed: float = 1.0, seed: int = 1,
                 model_pose: str | None = None, run_dir: Path | None = None):
        self.i = instance
        self.world = world
        self.speed = speed
        self.model = f"vtol_p1_{instance}"
        self.dir = run_dir or Path(f"/tmp/mc/inst{instance}")
        self.env = dict(os.environ)
        self.env.update({
            "GZ_PARTITION": f"mc{instance}",
            "GZ_SIM_SYSTEM_PLUGIN_PATH": str(PLUGIN_DIR),
            "HEADLESS": "1",
            "PX4_SYS_AUTOSTART": "4050",
            "PX4_SIM_MODEL": "gz_vtol_p1",
            "PX4_GZ_WORLD": world,
            "PX4_SIM_SPEED_FACTOR": f"{speed:g}",
        })
        if model_pose:
            self.env["PX4_GZ_MODEL_POSE"] = model_pose
        self.seed = seed
        self.proc = None

    # -- lifecycle --------------------------------------------------------------------
    def start(self, timeout: float = 240.0):
        self.stop()
        if self.dir.exists():
            shutil.rmtree(self.dir)
        self.dir.mkdir(parents=True)
        # PX4's gz rc script sources ./gz_env.sh (model/world paths) from its working directory.
        shutil.copy(BUILD / "rootfs" / "gz_env.sh", self.dir / "gz_env.sh")
        for f in glob.glob(f"/tmp/px4_lock-{self.i}") + glob.glob(f"/tmp/px4-sock-{self.i}"):
            os.remove(f)
        self.log = open(self.dir / "px4.log", "w")
        self.proc = subprocess.Popen(
            [str(BUILD / "bin" / "px4"), "-i", str(self.i), "-d", str(BUILD / "etc"),
             "-w", str(self.dir), "-s", "etc/init.d-posix/rcS"],
            cwd=self.dir, env=self.env, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(1.0)
            if self.proc.poll() is not None:
                raise SimError(f"PX4 exited: {self.tail()}")
            if "Ready for takeoff" in (self.dir / "px4.log").read_text(errors="ignore"):
                self.publish("turbulence", "gz.msgs.Double", "data: 0")
                return
        raise SimError(f"not ready after {timeout:.0f} s: {self.tail()}")

    def stop(self):
        if self.proc and self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGTERM)
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
        # Gazebo server of this partition (started by PX4's rc script, not our child).
        self._kill_partition_gz()
        self.proc = None

    def _kill_partition_gz(self):
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                env = open(f"/proc/{pid}/environ", "rb").read().split(b"\0")
                cmd = open(f"/proc/{pid}/cmdline", "rb").read()
            except OSError:
                continue
            if b"gz" in cmd and f"GZ_PARTITION=mc{self.i}".encode() in env:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except OSError:
                    pass

    def tail(self, n=15):
        try:
            text = (self.dir / "px4.log").read_text(errors="ignore")
        except OSError:
            return ""
        lines = [re.sub(r"\x1b\[[0-9;]*m", "", ln).strip() for ln in text.splitlines()]
        return " | ".join(ln for ln in lines if ln)[-1500:]

    # -- PX4 and Gazebo access ----------------------------------------------------------
    def cmd(self, *args, timeout=20.0) -> str:
        r = subprocess.run([str(BUILD / "bin" / f"px4-{args[0]}"), "--instance", str(self.i), *args[1:]],
                           env=self.env, capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr

    def param(self, name, value):
        self.cmd("param", "set", name, str(value))

    def listen(self, topic) -> dict:
        out = self.cmd("listener", topic, "-n", "1")
        fields = {}
        for line in out.splitlines():
            m = re.match(r"\s*([a-z_0-9\[\]]+):\s*(.+)$", line)
            if m:
                fields[m.group(1)] = m.group(2).strip()
        return fields

    def publish(self, what, msgtype, payload):
        subprocess.run(["gz", "topic", "-t", f"/model/{self.model}/{what}", "-m", msgtype, "-p", payload],
                       env=self.env, capture_output=True, timeout=20)

    def wind(self, east, north, up=0.0, ramp_s=0.0):
        steps = max(1, int(ramp_s * 4))
        for k in range(1, steps + 1):
            f = k / steps
            self.publish("wind", "gz.msgs.Vector3d", f"x: {east * f:.3f} y: {north * f:.3f} z: {up * f:.3f}")
            if steps > 1:
                self.sleep_sim(ramp_s / steps)

    def sim_time(self) -> float:
        """PX4 time in seconds. PX4 runs in lockstep with Gazebo, so this is simulation time. Read with
        the light px4-listener client: polling Gazebo's /stats with the gz CLI (Ruby + transport
        discovery per call) piled up and stalled runs at 3+ parallel instances."""
        for _ in range(3):
            try:
                ts = self.listen("vehicle_status").get("timestamp", "")
            except subprocess.TimeoutExpired:
                continue
            m = re.match(r"(\d+)", ts)
            if m:
                return int(m.group(1)) / 1e6
            time.sleep(1.0)
        raise SimError("no time from PX4")

    def sleep_sim(self, seconds: float):
        target = self.sim_time() + seconds
        while True:
            left = target - self.sim_time()
            if left <= 0:
                return
            time.sleep(min(2.0, max(0.05, left / self.speed)))

    def pose(self):
        """Gazebo ground truth position of the vehicle (ENU, m)."""
        r = subprocess.run(["gz", "model", "-m", self.model, "-p"], env=self.env, capture_output=True,
                           text=True, timeout=20)
        # first "[x y z]" row; "Model: [4]" (the entity id) must not match
        m = re.search(r"\[\s*(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s*\]", r.stdout)
        return [float(v) for v in m.groups()] if m else None

    def wait_for(self, predicate, timeout_sim: float, poll_sim: float = 1.0) -> bool:
        t_end = self.sim_time() + timeout_sim
        while self.sim_time() < t_end:
            if predicate():
                return True
            self.sleep_sim(poll_sim)
        return False

    def ulog(self) -> Path | None:
        logs = sorted(self.dir.glob("log/*/*.ulg"), key=os.path.getmtime)
        return logs[-1] if logs else None

    # -- common flight steps --------------------------------------------------------------
    def altitude(self) -> float:
        z = self.listen("vehicle_local_position").get("z")
        return -float(z) if z else float("nan")

    def takeoff(self, alt: float):
        self.param("MIS_TAKEOFF_ALT", alt)
        self.cmd("commander", "arm", "-f")
        self.cmd("commander", "takeoff")
        if not self.wait_for(lambda: self.altitude() > alt - 1.0, timeout_sim=alt / 1.5 + 30):
            raise SimError(f"takeoff to {alt} m not reached")
        self.sleep_sim(3)

    def quaternion(self):
        q = self.listen("vehicle_attitude").get("q", "")
        vals = [float(v) for v in re.findall(r"-?[\d.]+(?:e-?\d+)?", q)]
        return vals[:4] if len(vals) >= 4 else None

    def upwind_heading(self, samples: int = 5, min_tilt_deg: float = 1.5):
        """Average of several lean readings (gusts): unit vectors summed, so the direction is robust."""
        sx = sy = 0.0
        for k in range(samples):
            h = self._upwind_once(min_tilt_deg)
            if h is not None:
                sx += math.cos(h)
                sy += math.sin(h)
            if k < samples - 1:
                self.sleep_sim(0.5)
        return math.atan2(sy, sx) if math.hypot(sx, sy) > 0.5 * samples * 0.5 else None

    def _upwind_once(self, min_tilt_deg: float):
        """Heading (rad, NED) that points the nose into the wind, from onboard data only: in hover the
        vehicle leans into the wind, so the horizontal part of the thrust vector (-z body) points upwind.
        None when the lean is too small to tell (calm air)."""
        q = self.quaternion()
        if not q:
            return None
        w, x, y, z = q
        # third column of the body->NED rotation (body z axis in NED)
        zx, zy, zz = 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, zz))))
        if tilt < min_tilt_deg:
            return None
        return math.atan2(-zy, -zx)

    def heading(self) -> float:
        h = self.listen("vehicle_local_position").get("heading")
        return float(h) if h else float("nan")

    def turn_to(self, yaw_rad: float, timeout_sim: float = 20.0) -> bool:
        """Yaw in place with MAV_CMD_DO_REPOSITION (position NaN = keep, param4 = yaw in rad)."""
        from pymavlink import mavutil
        link = mavutil.mavlink_connection(f"udpout:127.0.0.1:{18570 + self.i}", source_system=255)
        nan = float("nan")
        link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0,
                                   -1, 0, 0, yaw_rad, nan, nan, nan)
        err = lambda: abs((self.heading() - yaw_rad + math.pi) % (2 * math.pi) - math.pi)
        return self.wait_for(lambda: err() < math.radians(10), timeout_sim=timeout_sim, poll_sim=1.0)

    def landed(self) -> bool:
        return self.listen("vehicle_land_detected").get("landed") == "True"

    def land_and_wait(self, timeout_sim: float = 120):
        self.cmd("commander", "land")
        return self.wait_for(self.landed, timeout_sim=timeout_sim, poll_sim=2.0)
