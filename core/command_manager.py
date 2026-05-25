"""
Command Manager — Bidirectional communication support.
Handles uplink commands to Ground Pico, ACKs, retries, and command queue.
Architecture prepared for future telecommands.
"""
from PyQt6.QtCore import QObject, pyqtSignal, QTimer
from time import time
from collections import deque


# Command types — extensible for future use
CMD_START_TELEMETRY   = 0x01
CMD_STOP_TELEMETRY    = 0x02
CMD_SENSOR_ZERO       = 0x03
CMD_RESET_TIMER       = 0x04
CMD_REQUEST_STATUS    = 0x05
CMD_CALIBRATION_MODE  = 0x06
CMD_RADIO_ENABLE      = 0x07
CMD_RADIO_DISABLE     = 0x08


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

    command_sent = pyqtSignal(str)      # status message
    command_acked = pyqtSignal(int)     # cmd_id
    command_failed = pyqtSignal(str)    # error message

    def __init__(self, serial_write_fn=None):
        super().__init__()
        self.serial_write = serial_write_fn
        self.pending_queue = deque()
        self.active_command = None

        self.retry_timer = QTimer()
        self.retry_timer.timeout.connect(self._check_timeout)
        self.retry_timer.start(500)

    def set_serial_writer(self, fn):
        self.serial_write = fn

    def send_command(self, cmd_type, payload=b""):
        cmd = Command(cmd_type, payload)
        self.pending_queue.append(cmd)
        self._process_queue()

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
            self.command_failed.emit("No serial connection")
            self.active_command = None
            self._process_queue()
            return

        cmd.attempts += 1
        cmd.sent_time = time()
        try:
            self.serial_write(cmd.to_bytes())
            self.command_sent.emit(f"CMD 0x{cmd.cmd_type:02X} sent (attempt {cmd.attempts})")
        except Exception as e:
            self.command_failed.emit(f"Send failed: {e}")

    def receive_ack(self, cmd_id):
        if self.active_command and self.active_command.cmd_id == cmd_id:
            self.active_command.acknowledged = True
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
                self.command_failed.emit(f"CMD 0x{cmd.cmd_type:02X} failed after {cmd.attempts} attempts")
                self.active_command = None
                self._process_queue()

    def queue_size(self):
        return len(self.pending_queue) + (1 if self.active_command else 0)
