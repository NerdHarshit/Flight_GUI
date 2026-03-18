from dbm import error
from time import time
import serial.tools.list_ports

from PyQt6.QtWidgets import QMainWindow , QWidget , QGridLayout , QVBoxLayout , QHBoxLayout , QPushButton,QScrollArea
from PyQt6.QtCore import QTimer

from gui.data_card import DataCard
from core.serial_worker import SerialWorker
from core.packet_parser import PacketParser
from core.calculations import CalculationsEngine
from core.flight_buffer import FlightBuffer
from gui.plots import LivePlot
from gui.timeline_widget import TimelineWidget
from core.csv_exporter import CSVExporter
from gui.animation_widget import AnimationWindow
from core.pdf_generator import PDFReport
from core.video_saver import VideoSaver

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("DJS Impulse Ground Station")
        self.setGeometry(100,100,500,400)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.anim_window = None

        container = QWidget()
        scroll.setWidget(container)

        self.setCentralWidget(scroll)

        self.main_layout = QVBoxLayout()
        container.setLayout(self.main_layout)

        self.build_top_dashboard()
        self.build_plot_section()

        

        self.calculations = CalculationsEngine()
        self.buffer = FlightBuffer()

        #self.start_serial("COM20")

        self.timeline_widget = TimelineWidget()
        self.main_layout.addWidget(self.timeline_widget)

        self.build_button_row()

        self.packet_count =0
        self.start_time = time()

        self.last_packet_time = time()
        self.auto_saved = False

        self.monitor_timer = QTimer()
        self.monitor_timer.timeout.connect(self.check_signal_loss)
        self.monitor_timer.start(1000)

        self.serial_connected = False
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.try_connect_serial)
        self.serial_timer.start(2000)#checks in 2 secs 

    #-------end of init-------

    def find_serial_ports(self):
        ports = serial.tools.list_ports.comports()

        #finding pico as a usb device
        for port in ports:
            print(port.device, port.description)
            if "USB" in port.description or "Pico" in port.description or "Serial" in port.description:
                return port.device
            
        return None
    #------end of find serial ports-------

    def start_serial(self, port):
        self.serial_worker = SerialWorker(port, 115200)
        self.serial_worker.line_received.connect(self.process_line)
        self.serial_worker.connection_error.connect(self.handle_serial_error)

        self.serial_worker.start()

    #------end of start serial-------

    def try_connect_serial(self):

        if self.serial_connected:
            return

        port = self.find_serial_ports()

        if port:
            print("Connecting to:", port)

            try:
                self.start_serial(port)
                self.serial_connected = True
                print("Connected successfully!")
            except Exception as e:
                print("Connection failed:", e)
    #------end of try connect serial-------

    def handle_serial_error(self, error):
        print("Serial error:", error)

        self.serial_connected = False

        # allow reconnect
        if hasattr(self, "serial_worker"):
            self.serial_worker.stop()
    #------end of handle serial error-------

    def process_line(self, line):

        #print("RAW",line)
        packet = PacketParser.parse(line)
        self.last_packet_time = time()

        if not packet:
            print("Failed to parse line..invalid packet:", line)
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
    
    #------end of process line-------

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

        self.acc_plot.setMinimumHeight(250)
        self.height_plot.setMinimumHeight(250)

        self.plot_layout.addWidget(self.acc_plot, 0, 0)
        self.plot_layout.addWidget(self.height_plot, 0, 1)
    #------end of build plot section-------

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

        t = packet["timestamp"]/1000.0

        h_baro = packet["H_baro"]
        h_gps = packet["H_gps"]
        h_avg = (h_baro+h_gps)/2

        self.flight_card.update_value("Time",round(t,2))
        self.flight_card.update_value("H_baro",round(h_baro,2))
        self.flight_card.update_value("H_avg",round(h_avg,2))

        if self.anim_window is not None:
            self.anim_window.update_state(packet)

        if packet["FSM"] == 7 and not self.auto_saved:
            print("Landing detected...autosaving data")
            self.auto_save_all()

    #------end of update ui-------
        
    def build_top_dashboard(self):
        self.top_grid = QGridLayout()
        self.main_layout.addLayout(self.top_grid)
        self.main_layout.setSpacing(12)
        self.main_layout.setContentsMargins(10,10,10,10)
        self.top_grid.setSpacing(10)

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

        self.flight_card = DataCard("Flight Stats", ["Time", "H_baro", "H_avg"])
        self.top_grid.addWidget(self.flight_card,1,2)

    #------end of build top dashboard-------
    
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
        

        self.connectButton = QPushButton("Download Graph",self)
        self.connectButton.clicked.connect(self.download_graph)

        self.viewanimButton = QPushButton("View Animation",self)
        self.viewanimButton.clicked.connect(self.open_animation_window)

        self.buttonRow.addWidget(self.checkpointButton)
        self.buttonRow.addWidget(self.saveCSVButton)
        self.buttonRow.addWidget(self.savePDFButton)
        self.buttonRow.addWidget(self.downloadAnimationButton)
        self.buttonRow.addWidget(self.viewanimButton)
        self.buttonRow.addWidget(self.connectButton)
#------end of build button row-------

    def Save_CheckPoint(self):
        filename = CSVExporter.exportCheckPoint(self.buffer)

        if(filename):
            print("Checkpoint saved to:",filename)
#------end of save checkpoint-------
    def Download_CSV(self):
        filename = CSVExporter.exportFullCSV(self.buffer)
        if filename:
            print("Full flight CSV saved:", filename)
#------end of download csv-------
    def Download_PDF_report(self):
        PDFReport.generate(self.buffer,self.acc_plot,self.height_plot)
#------end of download pdf report-------
    def Download_animation(self):
        if self.anim_window is None:
            self.anim_window = AnimationWindow()
            self.anim_window.show()
        
        #start recording
        if not self.anim_window.recording:
            print("recording started")
            self.anim_window.frames = []
            self.anim_window.recording = True

        else:
            print("saving video")
            self.anim_window.recording = False
            print("Frames captured:", len(self.anim_window.frames))
            self.anim_window.save_video()

        if self.anim_window.recording:
            self.downloadAnimationButton.setText("Stop & Save")
        else:
            self.downloadAnimationButton.setText("Download Animation")
            
#------end of download animation-------
    def open_animation_window(self):
        if self.anim_window is None:
            self.anim_window  = AnimationWindow()

        self.anim_window.show()
#------end of open animation window-------
    def download_graph(self):
        ts = int(time())

        self.acc_plot.save_plot(f"acc{ts}.png")
        self.height_plot.save_plot(f"height{ts}.png")
        print("Graphs saved")
#------end of download graph-------
    def check_signal_loss(self):
        if time() - self.last_packet_time > 20 and not self.auto_saved:
            print("Lost signal ....auto saving progresss")
            self.auto_save_all()
    #------end of check signal loss-------
    def auto_save_all(self):

        if self.auto_saved:
            return
        
        self.auto_saved = True

        print("auto saving all data")

        self.Download_CSV()
        print("Csv saved")

        self.download_graph()
        print("graphs saved")

        self.Download_PDF_report()
        print("pdf report saved")

         # 🚀 VIDEO IN SEPARATE THREAD
        if self.anim_window and self.anim_window.frames:

            print("Starting video save thread...")

            self.video_thread = VideoSaver(self.anim_window.frames.copy())

            self.video_thread.finished.connect(
                lambda: print("Video saved successfully!")
            )

            self.video_thread.error.connect(
                lambda e: print("Video error:", e)
            )

            self.video_thread.start()
        print("video saved")

#------end of auto save all-------
