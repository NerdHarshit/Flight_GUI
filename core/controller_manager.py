"""
Controller Manager — Dual-controller switching and state management.
"""
from time import time
import math


class ControllerState:
    def __init__(self, name):
        self.name = name
        self.reset()

    def update_from_packet(self, packet, prefix):
        self.alive = packet.get("flags", {}).get(f"{prefix.lower()}_alive", False)
        self.last_state = packet.get(f"{prefix}_state", 0)
        self.apogee_detected = packet.get(f"{prefix}_apogee", False)
        self.voltage = packet.get(f"{prefix}_voltage", 0.0)
        self.temperature = packet.get(f"{prefix}_temperature", 0.0)
        self.pressure = packet.get(f"{prefix}_pressure", 0.0)
        self.baro_alt = packet.get(f"{prefix}_baro_alt", 0.0)
        self.last_update_time = time()
        if prefix == "A":
            self.current = packet.get("A_current", 0.0)

        ax = packet.get(f"{prefix}_ax", 0.0)
        ay = packet.get(f"{prefix}_ay", 0.0)
        az = packet.get(f"{prefix}_az", 0.0)
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
        self.max_accel = max(self.max_accel, accel_mag)
        self.max_alt = max(self.max_alt, self.baro_alt)

        timestamp = packet.get("time_ms", 0)
        if self._last_timestamp is not None:
            dt = (timestamp - self._last_timestamp) / 1000.0
            dt = max(0.001, min(dt, 2.0))
            if 0 < dt <= 2:
                self.vx += 0.5 * (self._last_ax + ax) * dt
                self.vy += 0.5 * (self._last_ay + ay) * dt
                self.vz += 0.5 * (self._last_az + az) * dt
        self._last_ax, self._last_ay, self._last_az = ax, ay, az
        self._last_timestamp = timestamp

    def get_velocity_magnitude(self):
        return math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)

    def get_accel_magnitude(self, packet, prefix):
        ax = packet.get(f"{prefix}_ax", 0.0)
        ay = packet.get(f"{prefix}_ay", 0.0)
        az = packet.get(f"{prefix}_az", 0.0)
        return math.sqrt(ax**2 + ay**2 + az**2)

    def reset(self):
        self.alive = False
        self.last_state = 0
        self.apogee_detected = False
        self.voltage = self.current = self.temperature = 0.0
        self.pressure = self.baro_alt = self.max_alt = self.max_accel = 0.0
        self.vx = self.vy = self.vz = 0.0
        self._last_ax = self._last_ay = self._last_az = 0.0
        self._last_timestamp = None
        self.last_update_time = 0.0


class ControllerManager:
    def __init__(self):
        self.controller_a = ControllerState("Controller A")
        self.controller_b = ControllerState("Controller B")
        self.active = "A"

    @property
    def active_state(self):
        return self.controller_a if self.active == "A" else self.controller_b

    def switch(self, controller):
        if controller in ("A", "B"):
            self.active = controller

    def update(self, packet):
        self.controller_a.update_from_packet(packet, "A")
        self.controller_b.update_from_packet(packet, "B")

    def get_active_telemetry(self, packet):
        p = self.active
        state = self.active_state
        result = {
            "ax": packet.get(f"{p}_ax", 0.0),
            "ay": packet.get(f"{p}_ay", 0.0),
            "az": packet.get(f"{p}_az", 0.0),
            "baro_alt": packet.get(f"{p}_baro_alt", 0.0),
            "pressure": packet.get(f"{p}_pressure", 0.0),
            "temperature": packet.get(f"{p}_temperature", 0.0),
            "voltage": packet.get(f"{p}_voltage", 0.0),
            "state": packet.get(f"{p}_state", 0),
            "apogee": packet.get(f"{p}_apogee", False),
            "velocity": state.get_velocity_magnitude(),
            "accel_magnitude": state.get_accel_magnitude(packet, p),
            "max_altitude": state.max_alt,
            "max_acceleration": state.max_accel,
            "gx": packet.get("A_gx", 0.0),
            "gy": packet.get("A_gy", 0.0),
            "gz": packet.get("A_gz", 0.0),
            "roll": packet.get("A_roll", 0.0),
            "pitch": packet.get("A_pitch", 0.0),
            "yaw": packet.get("A_yaw", 0.0),
            "lat": packet.get("A_lat", 0.0),
            "lon": packet.get("A_lon", 0.0),
            "gps_alt": packet.get("A_gps_alt", 0.0),
            "current": packet.get("A_current", 0.0) if p == "A" else 0.0,
        }
        return result

    def reset(self):
        self.controller_a.reset()
        self.controller_b.reset()
