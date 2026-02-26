from PyQt6.QtCore import QThread, pyqtSignal
import serial


class SerialWorker(QThread):
    line_received = pyqtSignal(str)
    connection_error = pyqtSignal(str)

    def __init__(self, port, baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)

            while self.running:
                line = ser.readline().decode("utf-8").strip()

                if line:
                    self.line_received.emit(line)

            ser.close()

        except Exception as e:
            self.connection_error.emit(str(e))

    def stop(self):
        self.running = False
        self.wait()