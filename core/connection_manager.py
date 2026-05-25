"""
Connection Manager — Serial connection lifecycle with auto-reconnect.
"""
from PyQt6.QtCore import QObject, QThread, pyqtSignal, QTimer
import serial
import serial.tools.list_ports
from time import time


class SerialReaderThread(QThread):
    """Background thread for reading serial data."""
    line_received = pyqtSignal(str)
    binary_received = pyqtSignal(bytes)
    connection_lost = pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True

    def run(self):
        try:
            while self.running:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.line_received.emit(line)
        except Exception as e:
            self.connection_lost.emit(str(e))

    def stop(self):
        self.running = False
        self.wait(2000)


class ConnectionManager(QObject):
    """Manages serial connection to Ground Pico with auto-reconnect."""

    connected = pyqtSignal(str)        # port name
    disconnected = pyqtSignal(str)     # reason
    line_received = pyqtSignal(str)    # telemetry line
    status_changed = pyqtSignal(str)   # status message

    def __init__(self):
        super().__init__()
        self.ser = None
        self.reader_thread = None
        self.is_connected = False
        self.port_name = ""
        self.baudrate = 115200
        self.connect_time = 0

        self.auto_reconnect = True
        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self._try_reconnect)
        self.reconnect_timer.start(3000)

    def find_pico_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            desc = port.description.upper()
            if any(k in desc for k in ["USB", "PICO", "SERIAL", "ACM", "UART"]):
                return port.device
        return None

    def connect(self, port=None):
        if self.is_connected:
            return True
        if port is None:
            port = self.find_pico_port()
        if port is None:
            self.status_changed.emit("No device found")
            return False

        try:
            self.ser = serial.Serial(port, self.baudrate, timeout=1)
            self.port_name = port
            self.is_connected = True
            self.connect_time = time()

            self.reader_thread = SerialReaderThread(self.ser)
            self.reader_thread.line_received.connect(self.line_received.emit)
            self.reader_thread.connection_lost.connect(self._on_connection_lost)
            self.reader_thread.start()

            self.connected.emit(port)
            self.status_changed.emit(f"Connected: {port}")
            return True
        except Exception as e:
            self.status_changed.emit(f"Connect failed: {e}")
            return False

    def disconnect(self):
        reason = "User disconnect"
        self._cleanup(reason)

    def write(self, data: bytes):
        if self.ser and self.is_connected:
            try:
                self.ser.write(data)
            except Exception as e:
                self._on_connection_lost(str(e))

    def _on_connection_lost(self, error):
        self._cleanup(f"Connection lost: {error}")

    def _cleanup(self, reason):
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        was_connected = self.is_connected
        self.is_connected = False
        self.port_name = ""
        if was_connected:
            self.disconnected.emit(reason)
            self.status_changed.emit(reason)

    def _try_reconnect(self):
        if self.is_connected or not self.auto_reconnect:
            return
        self.connect()

    def get_uptime_seconds(self):
        if not self.is_connected:
            return 0
        return time() - self.connect_time
