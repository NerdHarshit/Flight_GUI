"""
Telemetry Manager — Central telemetry state for dual-controller architecture.
Handles packet parsing, dual-controller buffers, and telemetry state tracking.
"""

import struct
from dataclasses import dataclass, field
from typing import Optional, Dict, List
from time import time


# ─── Packet Structure ───────────────────────────────────────────────

PACKET_FORMAT = '<I'       # time_ms (uint32)
PACKET_FORMAT += '6f'      # A_ax, A_ay, A_az, A_gx, A_gy, A_gz
PACKET_FORMAT += '3f'      # A_roll, A_pitch, A_yaw
PACKET_FORMAT += '3f'      # A_lat, A_lon, A_gps_alt
PACKET_FORMAT += 'f'       # A_baro_alt
PACKET_FORMAT += '2f'      # A_pressure, A_temperature
PACKET_FORMAT += '2f'      # A_voltage, A_current
PACKET_FORMAT += 'B?'      # A_state, A_apogee
PACKET_FORMAT += '3f'      # B_ax, B_ay, B_az
PACKET_FORMAT += 'f'       # B_baro_alt
PACKET_FORMAT += '2f'      # B_pressure, B_temperature
PACKET_FORMAT += 'f'       # B_voltage
PACKET_FORMAT += 'B?'      # B_state, B_apogee
PACKET_FORMAT += 'B'       # system_flags
PACKET_FORMAT += 'HH'      # packet_id, crc

PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

# System flag bit definitions
FLAG_A_ALIVE         = 0x01
FLAG_B_ALIVE         = 0x02
FLAG_PARACHUTE       = 0x04
FLAG_GPS_LOCK        = 0x08
FLAG_SD_LOGGING      = 0x10
FLAG_FLASH_LOGGING   = 0x20


def parse_system_flags(flags: int) -> dict:
    return {
        "a_alive": bool(flags & FLAG_A_ALIVE),
        "b_alive": bool(flags & FLAG_B_ALIVE),
        "parachute_deployed": bool(flags & FLAG_PARACHUTE),
        "gps_lock": bool(flags & FLAG_GPS_LOCK),
        "sd_logging": bool(flags & FLAG_SD_LOGGING),
        "flash_logging": bool(flags & FLAG_FLASH_LOGGING),
    }


def parse_binary_packet(data: bytes) -> Optional[dict]:
    """Parse binary telemetry packet from Ground Pico."""
    try:
        if len(data) < PACKET_SIZE:
            return None

        values = struct.unpack(PACKET_FORMAT, data[:PACKET_SIZE])
        idx = 0

        packet = {}
        packet["time_ms"] = values[idx]; idx += 1

        # Controller A
        packet["A_ax"] = values[idx]; idx += 1
        packet["A_ay"] = values[idx]; idx += 1
        packet["A_az"] = values[idx]; idx += 1
        packet["A_gx"] = values[idx]; idx += 1
        packet["A_gy"] = values[idx]; idx += 1
        packet["A_gz"] = values[idx]; idx += 1
        packet["A_roll"] = values[idx]; idx += 1
        packet["A_pitch"] = values[idx]; idx += 1
        packet["A_yaw"] = values[idx]; idx += 1
        packet["A_lat"] = values[idx]; idx += 1
        packet["A_lon"] = values[idx]; idx += 1
        packet["A_gps_alt"] = values[idx]; idx += 1
        packet["A_baro_alt"] = values[idx]; idx += 1
        packet["A_pressure"] = values[idx]; idx += 1
        packet["A_temperature"] = values[idx]; idx += 1
        packet["A_voltage"] = values[idx]; idx += 1
        packet["A_current"] = values[idx]; idx += 1
        packet["A_state"] = values[idx]; idx += 1
        packet["A_apogee"] = values[idx]; idx += 1

        # Controller B
        packet["B_ax"] = values[idx]; idx += 1
        packet["B_ay"] = values[idx]; idx += 1
        packet["B_az"] = values[idx]; idx += 1
        packet["B_baro_alt"] = values[idx]; idx += 1
        packet["B_pressure"] = values[idx]; idx += 1
        packet["B_temperature"] = values[idx]; idx += 1
        packet["B_voltage"] = values[idx]; idx += 1
        packet["B_state"] = values[idx]; idx += 1
        packet["B_apogee"] = values[idx]; idx += 1

        # System
        packet["system_flags"] = values[idx]; idx += 1
        packet["flags"] = parse_system_flags(packet["system_flags"])
        packet["packet_id"] = values[idx]; idx += 1
        packet["crc"] = values[idx]; idx += 1

        packet["_receive_time"] = time()
        return packet

    except Exception:
        return None


def parse_csv_packet(line: str) -> Optional[dict]:
    """
    Parse CSV-formatted telemetry from Ground Pico.
    Supports:
    - AVIOPRO V0 TELEM format: TELEM,Seq,MissionTime,State,...
    - Legacy 14-field format
    - Legacy 31+ field dual-controller format
    """
    try:
        line = line.strip()

        # Ignore ground station info/warning messages
        if line.startswith("GS_"):
            return None

        # STATUS packets are not telemetry — parsed separately
        if line.startswith("STATUS,"):
            return None

        # AVIOPRO V0 telemetry format
        if line.startswith("TELEM,"):
            return _parse_aviopro_csv(line)

        parts = line.split(",")

        # Legacy formats
        if len(parts) == 14:
            return _parse_legacy_csv(parts)
        elif len(parts) >= 31:
            return _parse_new_csv(parts)

        return None

    except Exception:
        return None


def parse_status_packet(line: str) -> Optional[dict]:
    """Parse STATUS packet from Ground Pico AVIOPRO V0.
    Format: STATUS,Marker,BmpOK,BnoOK,SdOK,FlashOK,LoraOK,GpsSearch,CtrlB,BattV,FlightNum,RSSI,SNR
    """
    try:
        if not line.strip().startswith("STATUS,"):
            return None

        parts = line.strip().split(",")
        if len(parts) < 13:
            return None

        return {
            "marker": parts[1],
            "bmp_ok": parts[2].strip() == "1",
            "bno_ok": parts[3].strip() == "1",
            "sd_ok": parts[4].strip() == "1",
            "flash_ok": parts[5].strip() == "1",
            "lora_ok": parts[6].strip() == "1",
            "gps_searching": parts[7].strip() == "1",
            "ctrl_b_alive": parts[8].strip() == "1",
            "batt_v": float(parts[9]),
            "flight_number": int(parts[10]),
            "rssi": int(parts[11]),
            "snr": float(parts[12]),
            "_receive_time": time(),
        }
    except Exception:
        return None


def _parse_aviopro_csv(line: str) -> Optional[dict]:
    """
    Parse AVIOPRO V0 TELEM CSV format (single-controller, mirrored to A+B).
    Format: TELEM,Seq,MissionTime,State,Alt,MaxAlt,AccelMag,Pitch,Roll,Yaw,
            Temp,Press,Batt,Lat,Lon,GpsAlt,Sats,Stale,Launch,Apogee,Sep,Landed,RSSI,SNR
    """
    parts = line.split(",")
    if len(parts) < 24:
        return None

    seq         = int(parts[1])
    mission_time = float(parts[2])
    state       = int(parts[3])
    altitude    = float(parts[4])
    max_alt     = float(parts[5])
    accel_mag   = float(parts[6])
    pitch       = float(parts[7])
    roll        = float(parts[8])
    yaw         = float(parts[9])
    temperature = float(parts[10])
    pressure    = float(parts[11])
    batt_v      = float(parts[12])
    gps_lat     = float(parts[13])
    gps_lon     = float(parts[14])
    gps_alt     = float(parts[15])
    gps_sats    = int(parts[16])
    gps_stale   = parts[17].strip() == "1"
    launched    = parts[18].strip() == "1"
    apogee_flag = parts[19].strip() == "1"
    separated   = parts[20].strip() == "1"
    landed      = parts[21].strip() == "1"
    rssi        = int(parts[22])
    snr         = float(parts[23])

    # Build system flags from booleans
    sys_flags = 0x03  # Both controllers alive (mirrored single-controller)
    if not gps_stale and gps_sats >= 4:
        sys_flags |= FLAG_GPS_LOCK
    if separated:
        sys_flags |= FLAG_PARACHUTE
    sys_flags |= FLAG_SD_LOGGING | FLAG_FLASH_LOGGING  # Default on; STATUS overrides

    packet = {
        "time_ms": mission_time,

        # Controller A — mapped from AVIOPRO single-controller data
        "A_ax": accel_mag,   # Only magnitude available
        "A_ay": 0.0,
        "A_az": 0.0,
        "A_gx": 0.0,         # No individual gyro data from AVIOPRO
        "A_gy": 0.0,
        "A_gz": 0.0,
        "A_roll": roll,
        "A_pitch": pitch,
        "A_yaw": yaw,
        "A_lat": gps_lat,
        "A_lon": gps_lon,
        "A_gps_alt": gps_alt,
        "A_baro_alt": altitude,
        "A_pressure": pressure,
        "A_temperature": temperature,
        "A_voltage": batt_v,
        "A_current": 0.0,    # Not available in AVIOPRO packet
        "A_state": state,
        "A_apogee": apogee_flag,

        # Controller B — mirror Controller A for single-controller test launch
        "B_ax": accel_mag,
        "B_ay": 0.0,
        "B_az": 0.0,
        "B_baro_alt": altitude,
        "B_pressure": pressure,
        "B_temperature": temperature,
        "B_voltage": batt_v,
        "B_state": state,
        "B_apogee": apogee_flag,

        # System
        "system_flags": sys_flags,
        "flags": parse_system_flags(sys_flags),
        "packet_id": seq,
        "crc": 0,
        "signal_strength": rssi,
        "snr": snr,
        "packet_loss_pct": 0.0,
        "_receive_time": time(),

        # AVIOPRO-specific extras
        "max_altitude_avionics": max_alt,
        "gps_sats": gps_sats,
        "gps_stale": gps_stale,
        "launched": launched,
        "separated": separated,
        "landed": landed,
        "accel_mag": accel_mag,

        # Legacy compat keys (animation widget, PDF report)
        "timestamp": mission_time,
        "Ax": accel_mag,
        "Ay": 0.0,
        "Az": 0.0,
        "H_baro": altitude,
        "Latitude": gps_lat,
        "Longitude": gps_lon,
        "H_gps": gps_alt,
        "Gx": roll,    # Map Euler angles → legacy gyro keys for animation
        "Gy": pitch,
        "Gz": yaw,
        "FSM": state,
        "Signal": rssi,
        "Counter": seq,
    }

    return packet


def _parse_legacy_csv(parts: list) -> dict:
    """Parse old 14-field CSV format for backward compatibility."""
    packet = {
        "time_ms": float(parts[0]),
        "A_ax": float(parts[1]),
        "A_ay": float(parts[2]),
        "A_az": float(parts[3]),
        "A_baro_alt": float(parts[4]),
        "A_lat": float(parts[5]),
        "A_lon": float(parts[6]),
        "A_gps_alt": float(parts[7]),
        "A_gx": float(parts[8]),
        "A_gy": float(parts[9]),
        "A_gz": float(parts[10]),
        "A_state": int(parts[11]),
        "A_apogee": False,
        "A_roll": 0.0, "A_pitch": 0.0, "A_yaw": 0.0,
        "A_pressure": 0.0, "A_temperature": 0.0,
        "A_voltage": 0.0, "A_current": 0.0,

        # Controller B — mirror A data for legacy
        "B_ax": float(parts[1]), "B_ay": float(parts[2]), "B_az": float(parts[3]),
        "B_baro_alt": float(parts[4]),
        "B_pressure": 0.0, "B_temperature": 0.0,
        "B_voltage": 0.0,
        "B_state": int(parts[11]),
        "B_apogee": False,

        "system_flags": 0x0F,  # A+B alive, GPS lock
        "flags": parse_system_flags(0x0F),
        "packet_id": int(parts[13]),
        "crc": 0,
        "signal_strength": int(parts[12]),
        "packet_loss_pct": 0.0,
        "_receive_time": time(),

        # Legacy compat keys
        "timestamp": float(parts[0]),
        "Ax": float(parts[1]), "Ay": float(parts[2]), "Az": float(parts[3]),
        "H_baro": float(parts[4]),
        "Latitude": float(parts[5]), "Longitude": float(parts[6]),
        "H_gps": float(parts[7]),
        "Gx": float(parts[8]), "Gy": float(parts[9]), "Gz": float(parts[10]),
        "FSM": int(parts[11]),
        "Signal": int(parts[12]),
        "Counter": int(parts[13]),
    }
    return packet


def _parse_new_csv(parts: list) -> dict:
    """Parse new 31+ field CSV format."""
    idx = 0
    packet = {}
    packet["time_ms"] = float(parts[idx]); idx += 1

    packet["A_ax"] = float(parts[idx]); idx += 1
    packet["A_ay"] = float(parts[idx]); idx += 1
    packet["A_az"] = float(parts[idx]); idx += 1
    packet["A_gx"] = float(parts[idx]); idx += 1
    packet["A_gy"] = float(parts[idx]); idx += 1
    packet["A_gz"] = float(parts[idx]); idx += 1
    packet["A_roll"] = float(parts[idx]); idx += 1
    packet["A_pitch"] = float(parts[idx]); idx += 1
    packet["A_yaw"] = float(parts[idx]); idx += 1
    packet["A_lat"] = float(parts[idx]); idx += 1
    packet["A_lon"] = float(parts[idx]); idx += 1
    packet["A_gps_alt"] = float(parts[idx]); idx += 1
    packet["A_baro_alt"] = float(parts[idx]); idx += 1
    packet["A_pressure"] = float(parts[idx]); idx += 1
    packet["A_temperature"] = float(parts[idx]); idx += 1
    packet["A_voltage"] = float(parts[idx]); idx += 1
    packet["A_current"] = float(parts[idx]); idx += 1
    packet["A_state"] = int(parts[idx]); idx += 1
    packet["A_apogee"] = parts[idx].strip().lower() in ("1", "true"); idx += 1

    packet["B_ax"] = float(parts[idx]); idx += 1
    packet["B_ay"] = float(parts[idx]); idx += 1
    packet["B_az"] = float(parts[idx]); idx += 1
    packet["B_baro_alt"] = float(parts[idx]); idx += 1
    packet["B_pressure"] = float(parts[idx]); idx += 1
    packet["B_temperature"] = float(parts[idx]); idx += 1
    packet["B_voltage"] = float(parts[idx]); idx += 1
    packet["B_state"] = int(parts[idx]); idx += 1
    packet["B_apogee"] = parts[idx].strip().lower() in ("1", "true"); idx += 1

    packet["system_flags"] = int(parts[idx]); idx += 1
    packet["flags"] = parse_system_flags(packet["system_flags"])
    packet["packet_id"] = int(parts[idx]); idx += 1

    # Optional fields appended by Ground Pico
    packet["signal_strength"] = int(parts[idx]) if idx < len(parts) else -1; idx += 1
    packet["packet_loss_pct"] = float(parts[idx]) if idx < len(parts) else 0.0; idx += 1

    packet["crc"] = 0
    packet["_receive_time"] = time()

    # Legacy compat keys for existing systems
    packet["timestamp"] = packet["time_ms"]
    packet["Ax"] = packet["A_ax"]
    packet["Ay"] = packet["A_ay"]
    packet["Az"] = packet["A_az"]
    packet["H_baro"] = packet["A_baro_alt"]
    packet["Latitude"] = packet["A_lat"]
    packet["Longitude"] = packet["A_lon"]
    packet["H_gps"] = packet["A_gps_alt"]
    packet["Gx"] = packet["A_gx"]
    packet["Gy"] = packet["A_gy"]
    packet["Gz"] = packet["A_gz"]
    packet["FSM"] = packet["A_state"]
    packet["Signal"] = packet.get("signal_strength", 0)
    packet["Counter"] = packet["packet_id"]

    return packet


class ControllerBuffer:
    """Independent telemetry buffer for one controller."""

    def __init__(self, controller_id: str):
        self.id = controller_id
        self.data: List[dict] = []
        self.last_counter: Optional[int] = None
        self.lost_packets: int = 0
        self.max_altitude: float = 0.0
        self.max_acceleration: float = 0.0
        self.max_velocity: float = 0.0

    def add_packet(self, packet: dict):
        self.data.append(packet)

        # Track packet loss
        pid = packet.get("packet_id", 0)
        if self.last_counter is not None:
            expected = self.last_counter + 1
            if pid > expected:
                self.lost_packets += (pid - expected)
        self.last_counter = pid

    def get_packet_loss(self) -> int:
        return self.lost_packets

    def reset(self):
        self.data.clear()
        self.last_counter = None
        self.lost_packets = 0
        self.max_altitude = 0.0
        self.max_acceleration = 0.0
        self.max_velocity = 0.0


class TelemetryManager:
    """
    Central telemetry state manager for dual-controller architecture.
    Maintains independent buffers and histories for Controller A and B.
    """

    def __init__(self):
        self.buffer_a = ControllerBuffer("A")
        self.buffer_b = ControllerBuffer("B")
        self.active_controller = "A"  # "A" or "B"

        self.last_packet: Optional[dict] = None
        self.last_receive_time: float = 0
        self.total_packets: int = 0
        self.start_time: float = time()

        # Telemetry rate tracking
        self._rate_window: List[float] = []
        self._rate_window_size = 50

        self.last_status: Optional[dict] = None

    @property
    def active_buffer(self) -> ControllerBuffer:
        return self.buffer_a if self.active_controller == "A" else self.buffer_b

    def switch_controller(self, controller: str):
        """Switch active telemetry source. 'A' or 'B'."""
        if controller in ("A", "B"):
            self.active_controller = controller

    def process_packet(self, packet: dict):
        """Process a parsed telemetry packet into both controller buffers."""
        self.last_packet = packet
        self.last_receive_time = time()
        self.total_packets += 1

        # Track telemetry rate
        self._rate_window.append(self.last_receive_time)
        if len(self._rate_window) > self._rate_window_size:
            self._rate_window.pop(0)

        # Store in both buffers (same packet contains both A and B data)
        self.buffer_a.add_packet(packet)
        self.buffer_b.add_packet(packet)

        # Update max values for A
        import math
        a_acc = math.sqrt(packet.get("A_ax", 0)**2 + packet.get("A_ay", 0)**2 + packet.get("A_az", 0)**2)
        self.buffer_a.max_acceleration = max(self.buffer_a.max_acceleration, a_acc)
        self.buffer_a.max_altitude = max(self.buffer_a.max_altitude, packet.get("A_baro_alt", 0))

        b_acc = math.sqrt(packet.get("B_ax", 0)**2 + packet.get("B_ay", 0)**2 + packet.get("B_az", 0)**2)
        self.buffer_b.max_acceleration = max(self.buffer_b.max_acceleration, b_acc)
        self.buffer_b.max_altitude = max(self.buffer_b.max_altitude, packet.get("B_baro_alt", 0))

    def process_status(self, status: dict):
        """Store the latest STATUS packet from Ground Pico."""
        self.last_status = status

    def get_telemetry_rate(self) -> float:
        """Compute packets per second."""
        if len(self._rate_window) < 2:
            return 0.0
        dt = self._rate_window[-1] - self._rate_window[0]
        if dt <= 0:
            return 0.0
        return len(self._rate_window) / dt

    def get_latency_ms(self) -> float:
        """Time since last packet in milliseconds."""
        if self.last_receive_time == 0:
            return 0
        return (time() - self.last_receive_time) * 1000

    def get_active_data(self, packet: dict, key_prefix: str = None) -> dict:
        """Get telemetry values for the active controller from a packet."""
        prefix = self.active_controller + "_"
        result = {}
        for k, v in packet.items():
            if k.startswith(prefix):
                short_key = k[len(prefix):]
                result[short_key] = v
        return result

    def is_controller_alive(self, controller: str) -> bool:
        """Check if a controller is alive based on system flags."""
        if self.last_packet is None:
            return False
        flags = self.last_packet.get("flags", {})
        if controller == "A":
            return flags.get("a_alive", False)
        return flags.get("b_alive", False)

    def reset(self):
        self.buffer_a.reset()
        self.buffer_b.reset()
        self.last_packet = None
        self.total_packets = 0
        self.start_time = time()
        self._rate_window.clear()
