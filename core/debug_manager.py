"""
Debug Manager — System health evaluation and scrollable log.
Maintains an interleaved log of raw packets and GUI-raised warnings/errors.
"""
from time import time
from collections import deque


class LogEntry:
    """A single log entry — either a raw packet or a GUI-raised message."""

    def __init__(self, text, source="gui", level="nominal"):
        self.text = text
        self.source = source     # "packet" | "gui" | "command"
        self.level = level       # "nominal" | "warning" | "critical" | "info"
        self.timestamp = time()

    @property
    def color(self):
        if self.source == "packet":
            return "#888888"     # grey for raw packets
        if self.source == "command":
            return "#64B5F6"     # blue for command events
        return {
            "nominal":  "#00FF88",
            "info":     "#64B5F6",
            "warning":  "#FFD700",
            "critical": "#FF4444",
        }.get(self.level, "#AAAAAA")

    @property
    def prefix(self):
        if self.source == "packet":
            return "[PKT]"
        if self.source == "command":
            return "[CMD]"
        return {
            "nominal":  "[OK]",
            "info":     "[INFO]",
            "warning":  "[WARN]",
            "critical": "[CRIT]",
        }.get(self.level, "[---]")


class DebugManager:
    def __init__(self, max_entries=500):
        self.log = deque(maxlen=max_entries)
        self.last_packet_time = 0
        self.last_evaluation = 0
        self._new_entries_count = 0  # track unread entries for UI polling

    def log_packet(self, line):
        """Log a raw incoming packet line."""
        entry = LogEntry(line, source="packet", level="nominal")
        self.log.append(entry)
        self._new_entries_count += 1

    def log_command(self, text):
        """Log a command event (sent, acked, failed)."""
        entry = LogEntry(text, source="command", level="info")
        self.log.append(entry)
        self._new_entries_count += 1

    def log_warning(self, text):
        """Log a GUI-raised warning."""
        entry = LogEntry(text, source="gui", level="warning")
        self.log.append(entry)
        self._new_entries_count += 1

    def log_error(self, text):
        """Log a GUI-raised critical error."""
        entry = LogEntry(text, source="gui", level="critical")
        self.log.append(entry)
        self._new_entries_count += 1

    def log_info(self, text):
        """Log a GUI info message."""
        entry = LogEntry(text, source="gui", level="nominal")
        self.log.append(entry)
        self._new_entries_count += 1

    def has_new_entries(self):
        """Check if there are unread entries."""
        return self._new_entries_count > 0

    def get_new_entries(self):
        """Get all new entries since last call and reset counter."""
        count = self._new_entries_count
        self._new_entries_count = 0
        if count == 0:
            return []
        return list(self.log)[-count:]

    def get_all_entries(self):
        """Get the full log."""
        return list(self.log)

    def evaluate(self, packet, telemetry_mgr, controller_mgr, connection_mgr):
        """Run full system health evaluation. Logs any issues found."""
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
                issues.append(("Controller B not present", "warning"))

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

        # Log issues to the debug log
        for text, level in issues:
            if level == "critical":
                self.log_error(text)
            else:
                self.log_warning(text)

        # Return a summary for the top-level status label
        if not issues:
            return LogEntry("All systems nominal", source="gui", level="nominal")
        else:
            issues.sort(key=lambda x: {"critical": 0, "warning": 1}[x[1]])
            text = " | ".join([i[0] for i in issues[:3]])
            level = issues[0][1]
            return LogEntry(text, source="gui", level=level)

    def get_latest(self):
        if self.log:
            return list(self.log)[-1]
        return LogEntry("Waiting for telemetry...", source="gui", level="warning")
