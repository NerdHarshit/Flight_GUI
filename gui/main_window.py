from PyQt6.QtWidgets import QMainWindow , QWidget , QGridLayout , QVBoxLayout
from gui.data_card import DataCard
from core.serial_worker import SerialWorker
from core.packet_parser import PacketParser
from core.calculations import CalculationsEngine
from core.flight_buffer import FlightBuffer
from gui.plots import LivePlot

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("DJS Impulse Ground Station")
        self.setGeometry(100,100,500,400)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        self.main_layout = QVBoxLayout()
        central_widget.setLayout(self.main_layout)

        self.build_top_dashboard()
        self.build_plot_section()
        self.calculations = CalculationsEngine()
        self.buffer = FlightBuffer()

        self.start_serial("COM3")

    def start_serial(self, port):
        self.serial_worker = SerialWorker(port, 9600)
        self.serial_worker.line_received.connect(self.process_line)
        self.serial_worker.start()

    def process_line(self, line):

        packet = PacketParser.parse(line)

        if not packet:
            return

        # Store packet
        self.buffer.add_packet(packet)

        # Run calculations
        processed = self.calculations.update(packet)

        # Update UI
        self.update_ui(packet, processed)

    def build_plot_section(self):

        # Create container layout for plots
        self.plot_layout = QGridLayout()
        self.main_layout.addLayout(self.plot_layout)

        # Acceleration Plot
        self.acc_plot = LivePlot(
            title="Acceleration vs Time",
            curve_names=["Ax", "Ay", "Az"],
            y_label="Acceleration (m/s²)"
        )

        # Height Plot (Barometric)
        self.height_plot = LivePlot(
            title="Barometric Height vs Time",
            curve_names=["H_baro"],
            y_label="Height (m)"
        )

        self.plot_layout.addWidget(self.acc_plot, 0, 0)
        self.plot_layout.addWidget(self.height_plot, 0, 1)

    def update_ui(self, packet, processed):

        # Cards
        self.acc_card.update_value("Ax", packet["Ax"])
        self.acc_card.update_value("Ay", packet["Ay"])
        self.acc_card.update_value("Az", packet["Az"])

        self.vel_card.update_value("Vx", round(processed["Vx"], 2))
        self.vel_card.update_value("Vy", round(processed["Vy"], 2))
        self.vel_card.update_value("Vz", round(processed["Vz"], 2))

        self.gps_card.update_value("Latitude", packet["Latitude"])
        self.gps_card.update_value("Longitude", packet["Longitude"])
        self.gps_card.update_value("Altitude", packet["H_gps"])

        # Convert timestamp ms → seconds
        t = packet["timestamp"] / 1000.0

        # Acceleration plot
        self.acc_plot.add_point("Ax", t, packet["Ax"])
        self.acc_plot.add_point("Ay", t, packet["Ay"])
        self.acc_plot.add_point("Az", t, packet["Az"])

        # Height plot (Barometric)
        self.height_plot.add_point("H_baro", t, packet["H_baro"])

        
    def build_top_dashboard(self):
        self.top_grid = QGridLayout()
        self.main_layout.addLayout(self.top_grid)

        self.acc_card = DataCard("Acceleration", ["Ax", "Ay", "Az"])
        self.top_grid.addWidget(self.acc_card, 0, 0)

        self.gyro_card = DataCard("Gyroscope", ["Gx", "Gy", "Gz"])
        self.top_grid.addWidget(self.gyro_card, 0, 1)

        self.gps_card = DataCard("GPS", ["Latitude", "Longitude", "Altitude"])
        self.top_grid.addWidget(self.gps_card, 1, 0)

        self.vel_card = DataCard("Velocity", ["Vx", "Vy", "Vz"])
        self.top_grid.addWidget(self.vel_card, 1, 1)



