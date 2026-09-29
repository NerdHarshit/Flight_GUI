"""
Command Manager — Bidirectional communication support.
Handles uplink commands to Ground Pico, ACKs, retries, command queue,
and per-command state tracking for UI feedback.
"""
from PyQt6.QtCore import QObject, pyqtSignal, QTimer
from time import time
from collections import deque
from enum import Enum


# ─── Command type constants ─────────────────────────────────────────
CMD_START_TELEMETRY   = 0x01
CMD_STOP_TELEMETRY    = 0x02
CMD_SENSOR_ZERO       = 0x03   # calibrate altitude baseline
CMD_RESET_TIMER       = 0x04
CMD_REQUEST_STATUS    = 0x05
CMD_CALIBRATION_MODE  = 0x06
CMD_RADIO_ENABLE      = 0x07
CMD_RADIO_DISABLE     = 0x08

# New avionics commands
CMD_BUZZER_PLAY       = 0x09
CMD_SERVO_PARACHUTE   = 0x0A   # one-shot auto-cycle (0→90→0)
CMD_SERVO_PAYLOAD     = 0x0B   # one-shot auto-cycle (CanSat release)
CMD_RESET_CONTROLLER  = 0x0C
CMD_SET_QNH           = 0x0D   # payload: float32 pressure in hPa

# Payload / CanSat commands
CMD_ENABLE_SERVOS     = 0x10
CMD_DISABLE_SERVOS    = 0x11
CMD_SERVO_L_POS       = 0x12   # Servo L +ve nudge
CMD_SERVO_L_NEG       = 0x13   # Servo L -ve nudge
CMD_SERVO_R_POS       = 0x14   # Servo R +ve nudge
CMD_SERVO_R_NEG       = 0x15   # Servo R -ve nudge
CMD_SET_TARGET_GPS    = 0x16   # payload: lat(f32) + lon(f32)
CMD_SET_PD_CONSTANTS  = 0x17   # payload: TBD stub
CMD_SPIRAL_DOWN       = 0x18   # emergency spiral descent

# Human-readable names for debug/log display
CMD_NAMES = {
    CMD_START_TELEMETRY:  "start_Tx",
    CMD_STOP_TELEMETRY:   "stop_Tx",
    CMD_SENSOR_ZERO:      "calibrate",
    CMD_RESET_TIMER:      "reset_timer",
    CMD_REQUEST_STATUS:   "Status",
    CMD_CALIBRATION_MODE: "cal_mode",
    CMD_RADIO_ENABLE:     "radio_on",
    CMD_RADIO_DISABLE:    "radio_off",
    CMD_BUZZER_PLAY:      "buzzer_play",
    CMD_SERVO_PARACHUTE:  "Servo_parachute",
    CMD_SERVO_PAYLOAD:    "Servo_payload",
    CMD_RESET_CONTROLLER: "Reset_controller",
    CMD_SET_QNH:          "Set_QNH",
    CMD_ENABLE_SERVOS:    "Enable_Servos",
    CMD_DISABLE_SERVOS:   "Disable_Servos",
    CMD_SERVO_L_POS:      "Servo_L_+ve",
    CMD_SERVO_L_NEG:      "Servo_L_-ve",
    CMD_SERVO_R_POS:      "Servo_R_+ve",
    CMD_SERVO_R_NEG:      "Servo_R_-ve",
    CMD_SET_TARGET_GPS:   "Set_target_GPS",
    CMD_SET_PD_CONSTANTS: "Set_PD_constants",
    CMD_SPIRAL_DOWN:      "Spiral_Down",
}


class CommandState(Enum):
    IDLE    = "idle"
    PENDING = "pending"
    SENT    = "sent"
    ACKED   = "acked"
    FAILED  = "failed"


class Command:
    def __init__(self, cmd_type, payload=b"", retries=3, timeout_ms=2000):
        self.cmd_type = cmd_type
        self.payload = payload
        self.max_retries = retries
        self.timeout_ms = timeout_ms
        self.attempts = 0
        self.sent_time = 0
        self.acknowledged = False
        self.cmd_id = int(time() * 1000) & 0xFFFF

    def to_bytes(self):
        header = bytes([0xAA, 0x55])  # sync bytes
        return header + bytes([self.cmd_type]) + self.cmd_id.to_bytes(2, 'little') + self.payload


class CommandManager(QObject):
    """Manages outbound command packets with ACK tracking and retry logic."""

    command_sent = pyqtSignal(str)            # human-readable status message
    command_acked = pyqtSignal(int)           # cmd_id
    command_failed = pyqtSignal(str)          # error message
    # Emits (cmd_type, CommandState) so buttons can update their visual state
    command_state_changed = pyqtSignal(int, str)

    def __init__(self, serial_write_fn=None):
        super().__init__()
        self.serial_write = serial_write_fn
        self.pending_queue = deque()
        self.active_command = None

        # Per-command-type state tracking for UI
        self._cmd_states = {}

        # Timers for reverting acked/failed flash states
        self._revert_timers = {}

        self.retry_timer = QTimer()
        self.retry_timer.timeout.connect(self._check_timeout)
        self.retry_timer.start(500)

        # Tx session tracking
        self.tx_ever_started = False
        self.tx_active = False

    def set_serial_writer(self, fn):
        self.serial_write = fn

    def get_cmd_state(self, cmd_type):
        """Get the current visual state for a command type."""
        return self._cmd_states.get(cmd_type, CommandState.IDLE)

    def _set_cmd_state(self, cmd_type, state, revert_ms=0):
        """Update command state and emit signal for UI."""
        self._cmd_states[cmd_type] = state
        self.command_state_changed.emit(cmd_type, state.value)

        # Cancel any pending revert timer for this command
        if cmd_type in self._revert_timers:
            self._revert_timers[cmd_type].stop()
            del self._revert_timers[cmd_type]

        # Auto-revert acked/failed states back to idle
        if revert_ms > 0:
            timer = QTimer()
            timer.setSingleShot(True)
            timer.timeout.connect(lambda ct=cmd_type: self._set_cmd_state(ct, CommandState.IDLE))
            timer.start(revert_ms)
            self._revert_timers[cmd_type] = timer

    def send_command(self, cmd_type, payload=b""):
        """Queue and send a command."""
        cmd = Command(cmd_type, payload)
        self._set_cmd_state(cmd_type, CommandState.PENDING)
        self.pending_queue.append(cmd)
        self._process_queue()

        # Track Tx state
        if cmd_type == CMD_START_TELEMETRY:
            self.tx_ever_started = True
            self.tx_active = True
        elif cmd_type == CMD_STOP_TELEMETRY:
            self.tx_active = False

    def _process_queue(self):
        if self.active_command is not None:
            return
        if not self.pending_queue:
            return
        self.active_command = self.pending_queue.popleft()
        self._send_active()

    def _send_active(self):
        cmd = self.active_command
        if cmd is None:
            return
        if self.serial_write is None:
            self._set_cmd_state(cmd.cmd_type, CommandState.FAILED, revert_ms=3000)
            self.command_failed.emit("No serial connection")
            self.active_command = None
            self._process_queue()
            return

        cmd.attempts += 1
        cmd.sent_time = time()
        self._set_cmd_state(cmd.cmd_type, CommandState.SENT)
        try:
            self.serial_write(cmd.to_bytes())
            name = CMD_NAMES.get(cmd.cmd_type, f"0x{cmd.cmd_type:02X}")
            self.command_sent.emit(f"CMD {name} sent (attempt {cmd.attempts})")
        except Exception as e:
            self.command_failed.emit(f"Send failed: {e}")

    def receive_ack(self, cmd_id):
        if self.active_command and self.active_command.cmd_id == cmd_id:
            cmd = self.active_command
            cmd.acknowledged = True
            self._set_cmd_state(cmd.cmd_type, CommandState.ACKED, revert_ms=2000)
            self.command_acked.emit(cmd_id)
            self.active_command = None
            self._process_queue()

    def _check_timeout(self):
        cmd = self.active_command
        if cmd is None:
            return
        elapsed = (time() - cmd.sent_time) * 1000
        if elapsed > cmd.timeout_ms:
            if cmd.attempts < cmd.max_retries:
                self._send_active()
            else:
                name = CMD_NAMES.get(cmd.cmd_type, f"0x{cmd.cmd_type:02X}")
                self._set_cmd_state(cmd.cmd_type, CommandState.FAILED, revert_ms=3000)
                self.command_failed.emit(f"CMD {name} failed after {cmd.attempts} attempts")
                self.active_command = None
                self._process_queue()

    def queue_size(self):
        return len(self.pending_queue) + (1 if self.active_command else 0)
