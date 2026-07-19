from time import time
import os

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QGridLayout, QVBoxLayout, QHBoxLayout, 
    QPushButton, QScrollArea, QLabel, QFrame, QRadioButton, QButtonGroup, QSizePolicy
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap

from gui.data_card import DataCard
from gui.plots import LivePlot
from gui.timeline_widget import TimelineWidget
from gui.gauge_widget import GaugeWidget
from gui.animation_widget import AnimationWindow
from gui.map_window import MapWindow

from core.telemetry_manager import TelemetryManager, parse_csv_packet, parse_status_packet
from core.controller_manager import ControllerManager
from core.connection_manager import ConnectionManager
from core.command_manager import CommandManager
from core.network_manager import NetworkManager
from core.mission_state import MissionStateManager
from core.debug_manager import DebugManager
from core.logging_manager import LoggingManager
from core.pdf_generator import PDFReport
from core.video_saver import VideoSaver


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DJS Impulse Ground Station")
        self.setGeometry(100, 100, 1300, 800)

        # --- Managers ---
        self.telemetry_mgr = TelemetryManager()
        self.controller_mgr = ControllerManager()
        self.connection_mgr = ConnectionManager()
        self.command_mgr = CommandManager(self.connection_mgr.write)
        self.network_mgr = NetworkManager()
        self.mission_state = MissionStateManager()
        self.debug_mgr = DebugManager()
        
        self.anim_window = None
        self.map_window = None
        self.auto_saved = False

        # --- Setup UI ---
        self._build_ui()
        self._connect_signals()
        
        # --- Timers ---
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self._update_ui_timer)
        self.ui_timer.start(100)  # 10 Hz UI refresh
        
        self.debug_timer = QTimer()
        self.debug_timer.timeout.connect(self._update_debug)
        self.debug_timer.start(1000) # 1 Hz debug check

    def _build_ui(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        scroll.setWidget(container)
        self.setCentralWidget(scroll)

        self.main_layout = QVBoxLayout()
        container.setLayout(self.main_layout)

        # 1. Top Strip (Timestamp, Debug, Toggle, Branding)
        self._build_top_strip()

        # 2. Middle Section (Gauges, Timeline, Buttons, Connection, Plots)
        mid_layout = QGridLayout()
        self.main_layout.addLayout(mid_layout)
        
        # Left: Gauges
        gauge_layout = QVBoxLayout()
        self.gauge_acc = GaugeWidget("Acceleration", "m/s²", 100)
        self.gauge_vel = GaugeWidget("Velocity", "m/s", 300)
        self.gauge_alt = GaugeWidget("Altitude", "m", 3000)
        gauge_layout.addWidget(self.gauge_alt)
        gauge_layout.addWidget(self.gauge_acc)
        gauge_layout.addWidget(self.gauge_vel)
        mid_layout.addLayout(gauge_layout, 0, 0, 2, 1)

        # Center Top: Timeline
        self.timeline_widget = TimelineWidget()
        mid_layout.addWidget(self.timeline_widget, 0, 1)

        # Center Bottom: Control Buttons & Connection Panel
        center_bot_layout = QHBoxLayout()
        
        btn_layout = QGridLayout()
        self.btn_save_ckpt = QPushButton("Save Checkpoint")
        self.btn_view_anim = QPushButton("View Animation")
        self.btn_view_map = QPushButton("View Map")
        self.btn_report = QPushButton("Download PDF Report")
        self.btn_csv = QPushButton("Download CSV")
        self.btn_anim_save = QPushButton("Download Animation")
        
        btn_layout.addWidget(self.btn_save_ckpt, 0, 0)
        btn_layout.addWidget(self.btn_view_anim, 0, 1)
        btn_layout.addWidget(self.btn_view_map, 1, 0)
        btn_layout.addWidget(self.btn_report, 1, 1)
        btn_layout.addWidget(self.btn_csv, 2, 0)
        btn_layout.addWidget(self.btn_anim_save, 2, 1)
        center_bot_layout.addLayout(btn_layout)

        # Connection Panel
        conn_frame = QFrame()
        conn_frame.setObjectName("ConnectionPanel")
        conn_layout = QVBoxLayout(conn_frame)
        self.lbl_conn_status = QLabel("Radio: Disconnected")
        self.lbl_conn_sig = QLabel("Signal Strength: -- dB | Loss: 0%")
        self.lbl_conn_server = QLabel("Server: Off | Devices: 0")
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        conn_layout.addWidget(self.lbl_conn_status)
        conn_layout.addWidget(self.lbl_conn_sig)
        conn_layout.addWidget(self.lbl_conn_server)
        
        conn_btns = QHBoxLayout()
        conn_btns.addWidget(self.btn_connect)
        conn_btns.addWidget(self.btn_disconnect)
        conn_layout.addLayout(conn_btns)
        center_bot_layout.addWidget(conn_frame)
        
        mid_layout.addLayout(center_bot_layout, 1, 1)

        # Right: Plots
        plot_layout = QVBoxLayout()
        self.acc_plot = LivePlot(title="Acceleration Magnitude (m/s²)", curve_names=["AccMag"])
        self.height_plot = LivePlot(title="Altitude (m)", curve_names=["Baro", "GPS"])
        self.acc_plot.setMinimumHeight(200)
        self.height_plot.setMinimumHeight(200)
        plot_layout.addWidget(self.height_plot)
        plot_layout.addWidget(self.acc_plot)
        mid_layout.addLayout(plot_layout, 0, 2, 2, 1)

        # 3. Bottom Section (Data Cards)
        bot_layout = QGridLayout()
        self.card_attitude = DataCard("Attitude", ["Roll", "Pitch", "Yaw"])
        self.card_gps = DataCard("GPS", ["Lat", "Lon", "Alt", "Sats"])
        self.card_power = DataCard("Power", ["Voltage", "Current"])
        self.card_telemetry = DataCard("Telemetry", ["Rate", "Lost", "Latency"])
        self.card_radio = DataCard("Radio", ["RSSI", "SNR"])
        
        bot_layout.addWidget(self.card_attitude, 0, 0)
        bot_layout.addWidget(self.card_gps, 0, 1)
        bot_layout.addWidget(self.card_power, 0, 2)
        bot_layout.addWidget(self.card_telemetry, 0, 3)
        bot_layout.addWidget(self.card_radio, 0, 4)
        self.main_layout.addLayout(bot_layout)

    def _build_top_strip(self):
        top_layout = QHBoxLayout()
        
        # Timestamp & Debug
        time_debug_layout = QVBoxLayout()
        self.lbl_timestamp = QLabel("Time: T+00:00.00")
        self.lbl_timestamp.setStyleSheet("font-size: 16px; font-weight: bold; color: white;")
        self.lbl_debug = QLabel("Status: Waiting for telemetry...")
        self.lbl_debug.setStyleSheet("font-size: 14px; font-weight: bold; color: #FFD700;")
        time_debug_layout.addWidget(self.lbl_timestamp)
        time_debug_layout.addWidget(self.lbl_debug)
        top_layout.addLayout(time_debug_layout)
        
        top_layout.addStretch()

        # Controller Switch
        switch_frame = QFrame()
        switch_frame.setObjectName("Card")
        switch_layout = QHBoxLayout(switch_frame)
        switch_layout.setContentsMargins(10, 5, 10, 5)
        self.btn_grp = QButtonGroup(self)
        self.radio_c1 = QRadioButton("C1")
        self.radio_c2 = QRadioButton("C2")
        self.radio_c1.setChecked(True)
        self.radio_c1.setStyleSheet("color: white; font-weight: bold;")
        self.radio_c2.setStyleSheet("color: white; font-weight: bold;")
        self.btn_grp.addButton(self.radio_c1, 1)
        self.btn_grp.addButton(self.radio_c2, 2)
        switch_layout.addWidget(self.radio_c1)
        switch_layout.addWidget(self.radio_c2)
        top_layout.addWidget(switch_frame)
        
        top_layout.addStretch()
        
        # Branding Cards
        brand_frame = QFrame()
        brand_frame.setObjectName("LogoCard")
        brand_layout = QHBoxLayout(brand_frame)
        brand_layout.setContentsMargins(0,0,0,0)
        
        logo_label = QLabel()
        if os.path.exists("Assets/impulse_logo_black.png"):
            pixmap = QPixmap("Assets/impulse_logo_black.png").scaledToHeight(50, Qt.TransformationMode.SmoothTransformation)
            logo_label.setPixmap(pixmap)
        
        text_label = QLabel("<b>Impulse Ground Station</b><br/>Designed by Harshit Pandya")
        text_label.setStyleSheet("color: white; font-size: 12px; text-align: center;")
        
        brand_layout.addWidget(logo_label)
        brand_layout.addWidget(text_label)
        top_layout.addWidget(brand_frame)
        
        self.main_layout.addLayout(top_layout)

    def _connect_signals(self):
        self.connection_mgr.line_received.connect(self._process_line)
        self.connection_mgr.connected.connect(lambda p: self.lbl_conn_status.setText(f"Radio: Connected ({p})"))
        self.connection_mgr.disconnected.connect(lambda r: self.lbl_conn_status.setText(f"Radio: Disconnected"))
        
        self.btn_connect.clicked.connect(self.connection_mgr.connect)
        self.btn_disconnect.clicked.connect(self.connection_mgr.disconnect)
        
        self.btn_grp.buttonClicked.connect(self._switch_controller)
        
        self.btn_save_ckpt.clicked.connect(lambda: LoggingManager.exportCheckPoint(self.telemetry_mgr))
        self.btn_csv.clicked.connect(lambda: LoggingManager.exportFullCSV(self.telemetry_mgr))
        self.btn_report.clicked.connect(lambda: PDFReport.generate(self.telemetry_mgr.buffer_a, self.acc_plot, self.height_plot))
        
        self.btn_view_anim.clicked.connect(self._open_animation)
        self.btn_anim_save.clicked.connect(self._toggle_record_animation)
        self.btn_view_map.clicked.connect(self._open_map)
        
        # Server starts automatically
        self.network_mgr.start_server()
        self.network_mgr.clients_updated.connect(self._update_server_clients)

    def _switch_controller(self, btn):
        c = "A" if btn == self.radio_c1 else "B"
        self.telemetry_mgr.switch_controller(c)
        self.controller_mgr.switch(c)
        self._rebuild_plots()

    def _rebuild_plots(self):
        self.acc_plot.clear()
        self.height_plot.clear()
        
        buffer = self.telemetry_mgr.active_buffer.data
        recent = buffer[-300:] if len(buffer) > 300 else buffer
        
        for packet in recent:
            active_telem = self.controller_mgr.get_active_telemetry(packet)
            t = packet["time_ms"] / 1000.0
            self.acc_plot.add_point("AccMag", t, active_telem.get("accel_magnitude", 0))
            self.height_plot.add_point("Baro", t, active_telem.get("baro_alt", 0))
            if "gps_alt" in active_telem:
                self.height_plot.add_point("GPS", t, active_telem["gps_alt"])

    def _process_line(self, line):
        if line.startswith("STATUS,"):
            status = parse_status_packet(line)
            if status:
                self.telemetry_mgr.process_status(status)
            return

        if line.startswith("GS_"):
            return

        packet = parse_csv_packet(line)
        if not packet:
            return

        self.telemetry_mgr.process_packet(packet)
        self.controller_mgr.update(packet)
        self.mission_state.update(packet)
        self.network_mgr.broadcast(packet)
        
        # Pass to animation 
        if self.anim_window:
            self.anim_window.update_state(packet)
            
        # Check landing
        if self.mission_state.is_flight_complete() and not self.auto_saved:
            self._auto_save()

    def _update_ui_timer(self):
        packet = self.telemetry_mgr.last_packet
        if not packet:
            return
            
        active_telem = self.controller_mgr.get_active_telemetry(packet)
        t = packet["time_ms"] / 1000.0

        # Telemetry Card (always update for Latency)
        self.card_telemetry.update_value("Rate", f"{self.telemetry_mgr.get_telemetry_rate():.1f} Hz")
        self.card_telemetry.update_value("Lost", self.telemetry_mgr.active_buffer.get_packet_loss())
        self.card_telemetry.update_value("Latency", f"{self.telemetry_mgr.get_latency_ms():.0f} ms")
        
        # Connection Panel
        sig = packet.get("signal_strength", -1)
        loss = packet.get("packet_loss_pct", 0)
        self.lbl_conn_sig.setText(f"Signal Strength: {sig} dB | Loss: {loss:.1f}%")

        # Skip adding duplicate points to plots if data has stopped
        pid = packet.get("packet_id")
        if getattr(self, "_last_processed_pid", None) == pid:
            return
        self._last_processed_pid = pid

        # Timeline & Time
        self.lbl_timestamp.setText(f"Time: {self.mission_state.get_elapsed_formatted()}")
        self.timeline_widget.set_state(active_telem["state"])
        
        # Gauges
        self.gauge_acc.set_value(active_telem["accel_magnitude"])
        self.gauge_vel.set_value(active_telem["velocity"])
        self.gauge_alt.set_value(active_telem["baro_alt"])

        # Plots
        self.acc_plot.add_point("AccMag", t, active_telem["accel_magnitude"])
        self.height_plot.add_point("Baro", t, active_telem["baro_alt"])
        if "gps_alt" in active_telem:
            self.height_plot.add_point("GPS", t, active_telem["gps_alt"])

        # Data Cards
        self.card_attitude.update_value("Roll", active_telem.get("roll", 0))
        self.card_attitude.update_value("Pitch", active_telem.get("pitch", 0))
        self.card_attitude.update_value("Yaw", active_telem.get("yaw", 0))
        
        self.card_gps.update_value("Lat", active_telem.get("lat", 0))
        self.card_gps.update_value("Lon", active_telem.get("lon", 0))
        self.card_gps.update_value("Alt", active_telem.get("gps_alt", 0))
        self.card_gps.update_value("Sats", packet.get("gps_sats", 0))
        
        self.card_power.update_value("Voltage", active_telem.get("voltage", 0))
        self.card_power.update_value("Current", active_telem.get("current", 0))
        
        self.card_radio.update_value("RSSI", packet.get("signal_strength", 0))
        self.card_radio.update_value("SNR", packet.get("snr", 0.0))
        
        # (Telemetry and connection panels moved up)
        
        # Map update
        if self.map_window and "lat" in active_telem and active_telem["lat"] != 0:
            self.map_window.update_location(active_telem["lat"], active_telem["lon"], active_telem["gps_alt"])

    def _update_debug(self):
        msg = self.debug_mgr.evaluate(
            self.telemetry_mgr.last_packet,
            self.telemetry_mgr,
            self.controller_mgr,
            self.connection_mgr
        )
        self.lbl_debug.setText(f"Status: {msg.text}")
        self.lbl_debug.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {msg.color};")

    def _update_server_clients(self, clients):
        status = "On" if self.network_mgr.is_running else "Off"
        self.lbl_conn_server.setText(f"Server: {status} | Devices: {len(clients)}")

    def _open_animation(self):
        if self.anim_window is None:
            self.anim_window = AnimationWindow()
        self.anim_window.show()

    def _toggle_record_animation(self):
        if self.anim_window is None:
            self._open_animation()
            
        if not self.anim_window.recording:
            self.anim_window.frames = []
            self.anim_window.recording = True
            self.btn_anim_save.setText("Stop & Save Animation")
        else:
            self.anim_window.recording = False
            self.anim_window.save_video()
            self.btn_anim_save.setText("Download Animation")

    def _open_map(self):
        if self.map_window is None:
            self.map_window = MapWindow()
        self.map_window.show()

    def _auto_save(self):
        if self.auto_saved:
            return
        self.auto_saved = True
        LoggingManager.exportFullCSV(self.telemetry_mgr)
        PDFReport.generate(self.telemetry_mgr.buffer_a, self.acc_plot, self.height_plot)
        if self.anim_window and self.anim_window.recording:
            self._toggle_record_animation()
