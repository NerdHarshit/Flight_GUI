"""
Mission State Manager — FSM tracking, mission timer, state transitions.
"""
from time import time

MISSION_STATES = {
    0: "BOOT",
    1: "TEST_MODE",
    2: "LAUNCH_PAD",
    3: "ASCENT",
    4: "PAYLOAD_SEP",
    5: "DESCENT",
    6: "IMPACT",
    7: "SAFE_MODE",
}

STATE_COLORS = {
    0: "#888888",   # grey
    1: "#FFD700",   # gold
    2: "#00BFFF",   # blue
    3: "#FF6600",   # orange
    4: "#BB86FC",   # purple
    5: "#00FF88",   # green
    6: "#FF4444",   # red
    7: "#FFFFFF",   # white
}


class MissionStateManager:
    def __init__(self):
        self.current_state_a = 0
        self.current_state_b = 0
        self.mission_start_time = None
        self.state_history = []  # [(timestamp, state, controller)]
        self.apogee_time = None
        self.landing_time = None
        self._paused_elapsed = 0
        self._pause_time = None

    @property
    def active_state(self):
        return self.current_state_a

    def update(self, packet):
        state_a = packet.get("A_state", 0)
        state_b = packet.get("B_state", 0)
        t = packet.get("time_ms", 0)

        if state_a != self.current_state_a:
            self.state_history.append((t, state_a, "A"))
            self.current_state_a = state_a

        if state_b != self.current_state_b:
            self.state_history.append((t, state_b, "B"))
            self.current_state_b = state_b

        # Auto-start mission timer on ASCENT
        if self.mission_start_time is None and (state_a >= 3 or state_b >= 3):
            self.mission_start_time = time()

        # Track apogee
        if state_a == 4 and self.apogee_time is None:
            self.apogee_time = time()

        # Track landing
        if state_a >= 6 and self.landing_time is None:
            self.landing_time = time()

    def get_state_for_controller(self, controller):
        if controller == "A":
            return self.current_state_a
        return self.current_state_b

    def get_state_name(self, state_idx=None):
        if state_idx is None:
            state_idx = self.current_state_a
        return MISSION_STATES.get(state_idx, "UNKNOWN")

    def get_state_color(self, state_idx=None):
        if state_idx is None:
            state_idx = self.current_state_a
        return STATE_COLORS.get(state_idx, "#888888")

    def get_elapsed_seconds(self):
        if self.mission_start_time is None:
            return 0.0
        return time() - self.mission_start_time

    def get_elapsed_formatted(self):
        s = self.get_elapsed_seconds()
        mins = int(s) // 60
        secs = int(s) % 60
        ms = int((s % 1) * 100)
        return f"T+{mins:02d}:{secs:02d}.{ms:02d}"

    def is_flight_complete(self):
        return self.current_state_a >= 6 or self.current_state_b >= 6

    def reset(self):
        self.current_state_a = 0
        self.current_state_b = 0
        self.mission_start_time = None
        self.state_history.clear()
        self.apogee_time = None
        self.landing_time = None
