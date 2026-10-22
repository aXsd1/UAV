"""MAVLink link to PX4: telemetry state and commands (pymavlink, no other dependencies)."""

from __future__ import annotations

import math
import os
import threading
import time
from collections import deque

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

mav = mavutil.mavlink

# PX4 custom modes: custom_mode = main << 16 | sub << 24
MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED"}
AUTO_SUB = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND", 8: "FOLLOW",
            9: "PRECLAND", 10: "VTOL_TAKEOFF"}
MODES = {  # name -> (main, sub)
    "POSCTL": (3, 0), "ALTCTL": (2, 0), "STABILIZED": (7, 0),
    "LOITER": (4, 3), "MISSION": (4, 4), "RTL": (4, 5), "LAND": (4, 6), "TAKEOFF": (4, 2),
}
MANUAL_MODES = {"POSCTL", "ALTCTL", "STABILIZED"}
VTOL_STATE = {1: "TRANS_FW", 2: "TRANS_MC", 3: "MC", 4: "FW"}
LANDED_STATE = {1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF", 4: "LANDING"}

STREAMS = {  # message id -> rate (Hz)
    mav.MAVLINK_MSG_ID_ATTITUDE_QUATERNION: 20,
    mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10,
    mav.MAVLINK_MSG_ID_VFR_HUD: 10,
    mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10,
    mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 2,
    mav.MAVLINK_MSG_ID_SYS_STATUS: 2,
    mav.MAVLINK_MSG_ID_BATTERY_STATUS: 1,
    mav.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10,
    mav.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
    mav.MAVLINK_MSG_ID_HOME_POSITION: 1,
    mav.MAVLINK_MSG_ID_WIND_COV: 2,
}


def haversine(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def mode_name(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    if main == 4:
        return AUTO_SUB.get(sub, f"AUTO_{sub}")
    return MAIN.get(main, f"MODE_{main}")


class Vehicle:
    def __init__(self, url: str):
        self.url = url
        self.lock = threading.RLock()
        self.state: dict = {"connected": False}
        self.log: deque = deque(maxlen=200)
        self.log_seq = 0
        self.chunks: dict = {}
        self.sticks = {"x": 0.0, "y": 0.0, "z": 0.5, "r": 0.0}
        self.sticks_t = 0.0
        self.manual_enabled = False
        self.last_hb = 0.0
        self.master = None
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._tx, daemon=True).start()

    # -- receive ------------------------------------------------------------------------
    def _rx(self):
        self.master = mavutil.mavlink_connection(self.url, source_system=255, source_component=190)
        streams_requested = 0.0
        while True:
            msg = self.master.recv_match(blocking=True, timeout=1.0)
            now = time.time()
            if self.state.get("connected") and now - self.last_hb > 3:
                with self.lock:
                    self.state["connected"] = False
                self._event("Araçla bağlantı kesildi", 4)
            if msg is None:
                continue
            if msg.get_srcComponent() != 1:  # autopilot only
                continue
            if self.last_hb and now - streams_requested > 10 and self.state.get("connected"):
                streams_requested = now
                self._request_streams()
            self._handle(msg, now)

    def _handle(self, m, now):
        t = m.get_type()
        s = self.state
        with self.lock:
            if t == "HEARTBEAT":
                if m.type == mav.MAV_TYPE_GCS:
                    return
                if not s.get("connected"):
                    self._event("Araca bağlanıldı", 6)
                s["connected"] = True
                self.last_hb = now
                s["armed"] = bool(m.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                s["mode"] = mode_name(m.custom_mode)
            elif t == "ATTITUDE_QUATERNION":
                s["q"] = [m.q1, m.q2, m.q3, m.q4]
            elif t == "GLOBAL_POSITION_INT":
                s["lat"], s["lon"] = m.lat / 1e7, m.lon / 1e7
                s["alt_amsl"] = m.alt / 1000.0
                s["alt_rel"] = m.relative_alt / 1000.0
                s["vn"], s["ve"], s["vd"] = m.vx / 100.0, m.vy / 100.0, m.vz / 100.0
                s["ground_speed"] = math.hypot(s["vn"], s["ve"])
                if m.hdg != 65535:
                    s["heading"] = m.hdg / 100.0
                home = s.get("home")
                if home:
                    d = haversine(home[0], home[1], s["lat"], s["lon"])
                    s["dist_home"] = d
                    s["dist_home_3d"] = math.hypot(d, s["alt_amsl"] - home[2])
            elif t == "VFR_HUD":
                s["airspeed"] = m.airspeed
                s["climb"] = m.climb
                s["throttle"] = m.throttle
            elif t == "LOCAL_POSITION_NED":
                s["local"] = [m.x, m.y, m.z]
            elif t == "EXTENDED_SYS_STATE":
                s["vtol_state"] = VTOL_STATE.get(m.vtol_state, "?")
                s["landed_state"] = LANDED_STATE.get(m.landed_state, "?")
            elif t == "SYS_STATUS":
                s["voltage"] = m.voltage_battery / 1000.0 if m.voltage_battery != 65535 else None
                s["battery"] = m.battery_remaining if m.battery_remaining >= 0 else None
            elif t == "BATTERY_STATUS":
                if m.current_battery >= 0:
                    s["current"] = m.current_battery / 100.0
            elif t == "DISTANCE_SENSOR":
                d = m.current_distance / 100.0
                s["lidar"] = d if m.min_distance <= m.current_distance < m.max_distance else None
            elif t == "GPS_RAW_INT":
                s["gps_fix"], s["sats"] = m.fix_type, m.satellites_visible
            elif t == "HOME_POSITION":
                s["home"] = [m.latitude / 1e7, m.longitude / 1e7, m.altitude / 1000.0]
            elif t == "WIND_COV":
                s["wind_est"] = [m.wind_x, m.wind_y]  # NED, m/s (direction the air moves to)
            elif t == "STATUSTEXT":
                # long texts arrive in 50-char chunks sharing one id; the last chunk is shorter
                text = self.chunks.pop(m.id, "") + m.text if m.id else m.text
                if m.id and len(m.text) >= 50:
                    self.chunks[m.id] = text
                else:
                    self._event(text.strip(), m.severity)
            elif t == "COMMAND_ACK":
                if m.result not in (mav.MAV_RESULT_ACCEPTED, mav.MAV_RESULT_IN_PROGRESS):
                    name = mav.enums["MAV_CMD"].get(m.command)
                    name = name.name if name else str(m.command)
                    res = mav.enums["MAV_RESULT"][m.result].name
                    self._event(f"Komut reddedildi: {name} ({res})", 4)

    def _event(self, text, severity=6):
        with self.lock:
            self.log_seq += 1
            self.log.append({"seq": self.log_seq, "t": time.time(), "text": text, "sev": severity})

    def _request_streams(self):
        for msg_id, hz in STREAMS.items():
            self._cmd_long(mav.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)
        self._cmd_long(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_HOME_POSITION)

    # -- transmit -----------------------------------------------------------------------
    def _tx(self):
        """GCS heartbeat (1 Hz) and, while manual flight is on, virtual sticks (20 Hz)."""
        k = 0
        while True:
            time.sleep(0.05)
            if self.master is None:
                continue
            k += 1
            if k % 20 == 0:
                self.master.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            if self.manual_enabled and self.master.target_system:
                st = dict(self.sticks)
                if time.time() - self.sticks_t > 1.0:  # browser went quiet: centre the sticks
                    st = {"x": 0.0, "y": 0.0, "z": 0.5, "r": 0.0}
                self.master.mav.manual_control_send(
                    self.master.target_system, int(st["x"] * 1000), int(st["y"] * 1000),
                    int(st["z"] * 1000), int(st["r"] * 1000), 0)

    def _cmd_long(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        m = self.master
        if m is None or not m.target_system:
            raise RuntimeError("araç bağlı değil")
        m.mav.command_long_send(m.target_system, 1, cmd, 0, p1, p2, p3, p4, p5, p6, p7)

    def _set_param(self, name: str, value: float):
        m = self.master
        m.mav.param_set_send(m.target_system, 1, name.encode(), float(value), mav.MAV_PARAM_TYPE_REAL32)

    def set_mode(self, name: str):
        main, sub = MODES[name]
        if name in MANUAL_MODES and not self.manual_enabled:
            self.enable_manual(True)
            time.sleep(0.6)  # PX4 needs to see manual control before it accepts the mode
        self._cmd_long(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def enable_manual(self, on: bool):
        self.sticks = {"x": 0.0, "y": 0.0, "z": 0.5, "r": 0.0}
        self.sticks_t = time.time()
        self.manual_enabled = on
        self._event("Sanal kumanda açık" if on else "Sanal kumanda kapalı", 6)

    def command(self, a: dict):
        """Run one action from the browser. Returns a short Turkish confirmation."""
        act = a.get("action")
        nan = float("nan")
        if act == "arm":
            self._cmd_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            return "Motorlar kurma komutu gönderildi"
        if act == "disarm":
            self._cmd_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            return "Motorları durdurma komutu gönderildi"
        if act == "kill":
            self._cmd_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
            return "ACİL DURDURMA gönderildi"
        if act == "takeoff":
            alt = float(a.get("alt", 30))
            self._set_param("MIS_TAKEOFF_ALT", alt)
            time.sleep(0.2)
            if not self.state.get("armed"):
                self._cmd_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                time.sleep(0.5)
            self._cmd_long(mav.MAV_CMD_NAV_TAKEOFF, -1, 0, 0, nan, nan, nan, nan)
            return f"Kalkış: {alt:.0f} m"
        if act == "land":
            self._cmd_long(mav.MAV_CMD_NAV_LAND, 0, 0, 0, nan, nan, nan, nan)
            return "İniş"
        if act == "rtl":
            self._cmd_long(mav.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            return "Eve dönüş"
        if act == "to_fw":
            self._cmd_long(mav.MAV_CMD_DO_VTOL_TRANSITION, mav.MAV_VTOL_STATE_FW)
            return "Yatay uçuşa geçiş"
        if act == "to_mc":
            self._cmd_long(mav.MAV_CMD_DO_VTOL_TRANSITION, mav.MAV_VTOL_STATE_MC)
            return "Askıya (dikey) geçiş"
        if act == "mode":
            name = a["mode"]
            self.set_mode(name)
            return f"Mod: {name}"
        if act == "manual":
            self.enable_manual(bool(a.get("on")))
            return "tamam"
        if act == "goto":
            alt = a.get("alt")
            lat, lon = a.get("lat"), a.get("lon")
            alt_amsl = nan
            if alt is not None and self.state.get("home"):
                alt_amsl = self.state["home"][2] + float(alt)
            self._cmd_long(mav.MAV_CMD_DO_REPOSITION, -1, mav.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0, nan,
                           float(lat) if lat is not None else nan, float(lon) if lon is not None else nan,
                           alt_amsl)
            return "Hedefe gidiliyor" if lat is not None else f"İrtifa: {float(alt):.0f} m"
        if act == "speed":
            v = float(a["speed"])
            kind = 0 if self.state.get("vtol_state") == "FW" else 1  # airspeed in FW, ground speed in MC
            self._cmd_long(mav.MAV_CMD_DO_CHANGE_SPEED, kind, v, -1)
            return f"Hız: {v:.0f} m/s"
        raise ValueError(f"bilinmeyen komut: {act}")

    def set_sticks(self, st: dict):
        for k in ("x", "y", "r"):
            self.sticks[k] = max(-1.0, min(1.0, float(st.get(k, 0.0))))
        self.sticks["z"] = max(0.0, min(1.0, float(st.get("z", 0.5))))
        self.sticks_t = time.time()

    def snapshot(self, since: int = 0) -> dict:
        with self.lock:
            s = dict(self.state)
            s["manual_enabled"] = self.manual_enabled
            s["log"] = [e for e in self.log if e["seq"] > since]
        return s
