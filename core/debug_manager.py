"""
Debug Manager — System health evaluation and intelligent status messages.
Evaluates telemetry freshness, controller flags, packet validity, etc.
"""
from time import time


class DebugMessage:
    def __init__(self, text, level="nominal"):
        self.text = text
        self.level = level  # "nominal", "warning", "critical"
        self.timestamp = time()

    @property
    def color(self):
        return {
            "nominal": "#00FF88",
            "warning": "#FFD700",
            "critical": "#FF4444",
        }.get(self.level, "#AAAAAA")


class DebugManager:
    def __init__(self):
        self.messages = []
        self.max_messages = 50
        self.last_packet_time = 0
        self.last_evaluation = 0

    def evaluate(self, packet, telemetry_mgr, controller_mgr, connection_mgr):
        """Run full system health evaluation. Returns current DebugMessage."""
        now = time()
        self.last_evaluation = now
        issues = []

        # 1. Connection check
        if not connection_mgr.is_connected:
            issues.append(("Serial disconnected", "critical"))

        # 2. Telemetry freshness
        latency = telemetry_mgr.get_latency_ms()
        if latency > 5000:
            issues.append(("Telemetry lost", "critical"))
        elif latency > 2000:
            issues.append((f"High latency: {int(latency)}ms", "warning"))

        if packet:
            self.last_packet_time = now
            flags = packet.get("flags", {})

            # 3. Controller alive
            if not flags.get("a_alive", True):
                issues.append(("Controller A offline", "critical"))
            if not flags.get("b_alive", True):
                issues.append(("Controller B offline", "warning"))

            # 4. GPS lock
            if not flags.get("gps_lock", True):
                issues.append(("GPS lock lost", "warning"))

            # 5. SD logging
            if not flags.get("sd_logging", True):
                issues.append(("SD logging disabled", "warning"))

            # 6. Voltage check
            v_a = packet.get("A_voltage", 0)
            v_b = packet.get("B_voltage", 0)
            if 0 < v_a < 3.3:
                issues.append((f"Controller A low voltage: {v_a:.1f}V", "critical"))
            if 0 < v_b < 3.3:
                issues.append((f"Controller B low voltage: {v_b:.1f}V", "warning"))

            # 7. Signal strength
            sig = packet.get("signal_strength", -1)
            if sig != -1 and sig < -90:
                issues.append((f"Weak signal: {sig}dB", "warning"))

            # 8. GPS quality (AVIOPRO fields)
            gps_sats = packet.get("gps_sats", -1)
            if gps_sats >= 0 and gps_sats < 4:
                issues.append((f"GPS: Only {gps_sats} sats", "warning"))
            if packet.get("gps_stale", False):
                issues.append(("GPS: Stale data", "warning"))

        # 9. Status packet sensor health (AVIOPRO)
        status = getattr(telemetry_mgr, 'last_status', None)
        if status:
            if not status.get("bmp_ok", True):
                issues.append(("BMP sensor offline", "critical"))
            if not status.get("bno_ok", True):
                issues.append(("BNO sensor offline", "critical"))
            if not status.get("sd_ok", True):
                issues.append(("SD card failed", "warning"))
            if not status.get("flash_ok", True):
                issues.append(("Flash storage failed", "warning"))

        # Build final message
        if not issues:
            msg = DebugMessage("Everything nominal", "nominal")
        else:
            # Show most critical issue
            issues.sort(key=lambda x: {"critical": 0, "warning": 1, "nominal": 2}[x[1]])
            text = " | ".join([i[0] for i in issues[:3]])
            level = issues[0][1]
            msg = DebugMessage(text, level)

        self._add_message(msg)
        return msg

    def _add_message(self, msg):
        self.messages.append(msg)
        if len(self.messages) > self.max_messages:
            self.messages.pop(0)

    def get_latest(self):
        if self.messages:
            return self.messages[-1]
        return DebugMessage("Waiting for telemetry...", "warning")
