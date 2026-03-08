from time import time

from PyQt6.QtWidgets import QMainWindow , QWidget , QGridLayout , QVBoxLayout , QHBoxLayout , QPushButton
from gui.data_card import DataCard
from core.serial_worker import SerialWorker
from core.packet_parser import PacketParser
from core.calculations import CalculationsEngine
from core.flight_buffer import FlightBuffer
from gui.plots import LivePlot
from gui.timeline_widget import TimelineWidget
from core.csv_exporter import CSVExporter
from gui.animation_widget import AnimationWindow

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

        self.anim_window = None

        self.calculations = CalculationsEngine()
        self.buffer = FlightBuffer()

        self.start_serial("COM20")

        self.timeline_widget = TimelineWidget()
        self.main_layout.addWidget(self.timeline_widget)

        self.build_button_row()

        self.packet_count =0
        self.start_time = time()

    def start_serial(self, port):
        self.serial_worker = SerialWorker(port, 115200)
        self.serial_worker.line_received.connect(self.process_line)
        self.serial_worker.start()

    def process_line(self, line):

        packet = PacketParser.parse(line)
        

        if not packet:
            return
        
        self.packet_count +=1

        elapsed = time() - self.start_time
        rate = self.packet_count/elapsed if elapsed >0 else 0

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

        self.gyro_card.update_value("Gx",packet["Gx"])
        self.gyro_card.update_value("Gy",packet["Gy"])
        self.gyro_card.update_value("Gz",packet["Gz"])

        # Height plot (Barometric)
        self.height_plot.add_point("H_baro", t, packet["H_baro"])

        #timeline
        self.timeline_widget.set_state(packet["FSM"])
        
        #telemetry
        self.telemetry_card.update_value("Signal", packet["Signal"])
        self.telemetry_card.update_value("Packets", packet["Counter"])

        lost = self.buffer.get_packet_loss()
        self.telemetry_card.update_value("Lost", lost)

        total = packet["Counter"]
        loss_percent = (lost / total * 100) if total > 0 else 0

        self.telemetry_card.update_value("Loss %", round(loss_percent, 2))

        rate = self.packet_count / (time() - self.start_time)
        self.telemetry_card.update_value("Rate", round(rate, 1))

        if self.anim_window is not None:
            self.anim_window.update_state(packet)
        
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

        self.telemetry_card = DataCard("Telemetry",["Signal","Packets","Lost","Loss %","Rate"])
        self.top_grid.addWidget(self.telemetry_card,0,2)

    
    def build_button_row(self):
        self.buttonRow = QHBoxLayout()
        self.main_layout.addLayout(self.buttonRow)

        self.checkpointButton = QPushButton("Save checkpoint",self)
        self.checkpointButton.clicked.connect(self.Save_CheckPoint)

        self.saveCSVButton = QPushButton("Download CSV",self)
        self.saveCSVButton.clicked.connect(self.Download_CSV)

        self.savePDFButton = QPushButton("Download PDF Report",self)
        self.savePDFButton.clicked.connect(self.Download_PDF_report)

        self.downloadAnimationButton = QPushButton("Download Animation",self)
        self.downloadAnimationButton.clicked.connect(self.Download_animation)

        self.connectButton = QPushButton("Connect Radio",self)
        self.connectButton.clicked.connect(self.connect_to_radio)

        self.viewanimButton = QPushButton("View Animation",self)
        self.viewanimButton.clicked.connect(self.open_animation_window)

        self.buttonRow.addWidget(self.checkpointButton)
        self.buttonRow.addWidget(self.saveCSVButton)
        self.buttonRow.addWidget(self.savePDFButton)
        self.buttonRow.addWidget(self.downloadAnimationButton)
        self.buttonRow.addWidget(self.viewanimButton)
        self.buttonRow.addWidget(self.connectButton)


    def Save_CheckPoint(self):
        filename = CSVExporter.exportCheckPoint(self.buffer)

        if(filename):
            print("Checkpoint saved to:",filename)

    def Download_CSV(self):
        pass

    def Download_PDF_report(self):
        pass

    def Download_animation(self):
        pass

    def open_animation_window(self):
        if self.anim_window is None:
            self.anim_window  = AnimationWindow()

        self.anim_window.show()

    def connect_to_radio(self):
        pass
