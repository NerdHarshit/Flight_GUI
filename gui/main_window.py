"""
Main Window — AvioPro Ground Station

Three-tab layout:
  Tab 0 — Ctrl_A  (AvionicsPanel, controller A active)
  Tab 1 — Ctrl_B  (AvionicsPanel, controller B active)
  Tab 2 — Payload (PayloadPanel)

Top bar: mission timer, tab selector (Ctrl_A | Ctrl_B | Payload).

All manager wiring, telemetry routing, debug feeding, and auto-save
are handled here. The individual panels own their layout/widgets.
"""
import os
import struct
from time import time

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QButtonGroup, QSizePolicy,
    QStackedWidget,
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap, QFont

from gui.avionics_panel import AvionicsPanel
from gui.payload_panel  import PayloadPanel

try:
    from gui.map_window import MapWindow
    _MAP_AVAILABLE = True
except Exception:
    _MAP_AVAILABLE = False
    MapWindow = None

try:
    from gui.animation_widget import AnimationWindow
    _ANIMATION_AVAILABLE = True
except Exception:
    _ANIMATION_AVAILABLE = False
    AnimationWindow = None


from core.telemetry_manager import TelemetryManager, parse_csv_packet, parse_status_packet
from core.controller_manager import ControllerManager
from core.connection_manager  import ConnectionManager
from core.command_manager     import CommandManager
from core.network_manager     import NetworkManager
from core.mission_state       import MissionStateManager
from core.debug_manager       import DebugManager
from core.logging_manager     import LoggingManager
from core.pdf_generator       import PDFReport
from core.video_saver         import VideoSaver


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AvioPro Ground Station")
        self.setGeometry(50, 50, 1400, 860)

        # ── Managers ──────────────────────────────────────────────────
        self.telemetry_mgr  = TelemetryManager()
        self.controller_mgr = ControllerManager()
        self.connection_mgr = ConnectionManager()
        self.command_mgr    = CommandManager(self.connection_mgr.write)
        self.network_mgr    = NetworkManager()
        self.mission_state  = MissionStateManager()
        self.debug_mgr      = DebugManager()

        self.anim_window = None
        self.map_window  = None
        self.auto_saved  = False

        # ── Build UI ──────────────────────────────────────────────────
        self._build_ui()
        self._connect_all_signals()

        # ── Timers ────────────────────────────────────────────────────
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self._tick_ui)
        self.ui_timer.start(100)      # 10 Hz

        self.debug_timer = QTimer()
        self.debug_timer.timeout.connect(self._tick_debug)
        self.debug_timer.start(1000)  # 1 Hz health check

    # ──────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root_widget = QWidget()
        root_layout = QVBoxLayout(root_widget)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        self.setCentralWidget(root_widget)

        # Top bar
        root_layout.addWidget(self._build_top_bar())

        # Stacked pages (one per tab)
        self.stack = QStackedWidget()
        root_layout.addWidget(self.stack, 1)

        # Page 0: Ctrl_A
        self.panel_a = AvionicsPanel(self.command_mgr)
        self.stack.addWidget(self.panel_a)

        # Page 1: Ctrl_B
        self.panel_b = AvionicsPanel(self.command_mgr)
        self.stack.addWidget(self.panel_b)

        # Page 2: Payload
        self.panel_payload = PayloadPanel(self.command_mgr)
        self.stack.addWidget(self.panel_payload)

        # Wire flyout buttons that open sub-windows (same for both avionics panels)
        for panel in (self.panel_a, self.panel_b):
            self._wire_avionics_flyout(panel)
        self._wire_payload_flyout(self.panel_payload)

        self.stack.setCurrentIndex(0)

    def _build_top_bar(self):
        bar = QFrame()
        bar.setObjectName("TopBar")
        bar.setFixedHeight(52)
        bar.setStyleSheet(
            "QFrame#TopBar { background-color: #0d0f14; "
            "border-bottom: 1px solid rgba(187,134,252,0.25); }"
        )

        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 4, 12, 4)
        lay.setSpacing(12)

        # Mission timer
        self.lbl_timer = QLabel("T: 00:00:000")
        self.lbl_timer.setStyleSheet(
            "font-size: 20px; font-weight: bold; color: #BB86FC; "
            "font-family: 'Consolas','Courier New',monospace;"
        )
        lay.addWidget(self.lbl_timer)

        lay.addStretch()

        # Tab selector: Ctrl_A | Ctrl_B | Payload
        self.tab_group = QButtonGroup(self)
        tab_frame = QFrame()
        tab_frame.setStyleSheet(
            "QFrame { background: rgba(20,22,28,0.85); border: 1px solid rgba(187,134,252,0.3); "
            "border-radius: 10px; }"
        )
        tab_lay = QHBoxLayout(tab_frame)
        tab_lay.setContentsMargins(4, 4, 4, 4)
        tab_lay.setSpacing(2)

        _TAB_BTN_BASE = (
            "QPushButton {{ background: transparent; color: #888; border-radius: 8px; "
            "padding: 6px 18px; font-weight: bold; font-size: 13px; border: none; }}"
            "QPushButton:checked {{ background: rgba(139,43,226,0.85); color: white; "
            "border: 1px solid rgba(187,134,252,0.6); }}"
            "QPushButton:hover:!checked {{ color: #BB86FC; }}"
        )
        self.btn_tab_a       = QPushButton("Ctrl_A");  self.btn_tab_a.setCheckable(True)
        self.btn_tab_b       = QPushButton("Ctrl_B");  self.btn_tab_b.setCheckable(True)
        self.btn_tab_payload = QPushButton("Payload"); self.btn_tab_payload.setCheckable(True)
        self.btn_tab_a.setChecked(True)

        for i, btn in enumerate((self.btn_tab_a, self.btn_tab_b, self.btn_tab_payload)):
            btn.setStyleSheet(_TAB_BTN_BASE)
            self.tab_group.addButton(btn, i)
            tab_lay.addWidget(btn)

        lay.addWidget(tab_frame)
        lay.addStretch()

        # Status label (replaces old debug label in top bar)
        self.lbl_status = QLabel("Status: Waiting for telemetry…")
        self.lbl_status.setStyleSheet(
            "font-size: 12px; color: #FFD700; font-weight: bold;"
        )
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        lay.addWidget(self.lbl_status)

        return bar

    # ──────────────────────────────────────────────────────────────────
    # Signal wiring
    # ──────────────────────────────────────────────────────────────────

    def _connect_all_signals(self):
        # Tab switching
        self.btn_tab_a.clicked.connect(lambda: self._switch_tab(0))
        self.btn_tab_b.clicked.connect(lambda: self._switch_tab(1))
        self.btn_tab_payload.clicked.connect(lambda: self._switch_tab(2))

        # Incoming data
        self.connection_mgr.line_received.connect(self._process_line)

        # Command events → debug log
        self.command_mgr.command_sent.connect(
            lambda msg: self.debug_mgr.log_command(msg))
        self.command_mgr.command_failed.connect(
            lambda msg: self.debug_mgr.log_error(msg))
        self.command_mgr.command_acked.connect(
            lambda cid: self.debug_mgr.log_command(f"ACK id={cid}"))

        # Network server
        self.network_mgr.start_server()
        self.network_mgr.clients_updated.connect(self._on_clients_updated)

    def _wire_avionics_flyout(self, panel: AvionicsPanel):
        """Wire the Additional_options flyout buttons for an avionics panel."""
        fb = panel._flyout_btns
        fb["save_csv"].clicked.connect(lambda: LoggingManager.exportFullCSV(self.telemetry_mgr))
        fb["save_ckpt"].clicked.connect(lambda: LoggingManager.exportCheckPoint(self.telemetry_mgr))
        fb["show_map"].clicked.connect(self._open_map)
        fb["show_anim"].clicked.connect(self._open_animation)
        fb["save_data"].clicked.connect(self._save_all_data)
        fb["start_srv"].clicked.connect(lambda: self.network_mgr.start_server())
        fb["stop_srv"].clicked.connect(lambda: self.network_mgr.stop_server())

    def _wire_payload_flyout(self, panel: PayloadPanel):
        panel.btn_save_csv.clicked.connect(lambda: LoggingManager.exportFullCSV(self.telemetry_mgr))
        panel.btn_save_ckpt.clicked.connect(lambda: LoggingManager.exportCheckPoint(self.telemetry_mgr))
        panel.btn_show_map.clicked.connect(self._open_map)
        panel.btn_show_anim.clicked.connect(self._open_animation)
        panel.btn_save_data.clicked.connect(self._save_all_data)
        panel.btn_start_srv.clicked.connect(lambda: self.network_mgr.start_server())
        panel.btn_stop_srv.clicked.connect(lambda: self.network_mgr.stop_server())

    # ──────────────────────────────────────────────────────────────────
    # Tab management
    # ──────────────────────────────────────────────────────────────────

    def _switch_tab(self, index: int):
        self.stack.setCurrentIndex(index)
        if index == 0:
            self.controller_mgr.switch("A")
        elif index == 1:
            self.controller_mgr.switch("B")
        # index 2 = Payload — no controller switch needed

    # ──────────────────────────────────────────────────────────────────
    # Incoming telemetry
    # ──────────────────────────────────────────────────────────────────

    def _process_line(self, line: str):
        # Log raw packet to debug
        self.debug_mgr.log_packet(line)

        # Status packet
        if line.startswith("STATUS,"):
            status = parse_status_packet(line)
            if status:
                self.telemetry_mgr.process_status(status)
            return

        # Ground-station echo lines (ignore)
        if line.startswith("GS_"):
            return

        packet = parse_csv_packet(line)
        if not packet:
            return

        self.telemetry_mgr.process_packet(packet)
        self.controller_mgr.update(packet)
        self.mission_state.update(packet)
        self.network_mgr.broadcast(packet)

        if self.anim_window:
            self.anim_window.update_state(packet)

        if self.mission_state.is_flight_complete() and not self.auto_saved:
            self._auto_save()

    # ──────────────────────────────────────────────────────────────────
    # UI update ticks
    # ──────────────────────────────────────────────────────────────────

    def _tick_ui(self):
        # Always update timer
        self.lbl_timer.setText(f"T: {self.mission_state.get_elapsed_formatted()}")

        current_total = self.telemetry_mgr.total_packets
        if getattr(self, "_last_processed_total", None) == current_total:
            return
        self._last_processed_total = current_total

        idx = self.stack.currentIndex()
        
        if idx in (0, 1):
            packet = self.telemetry_mgr.last_packet
            if not packet:
                return
            active_telem = self.controller_mgr.get_active_telemetry(packet)
            t = packet.get("time_ms", 0) / 1000.0
            if idx == 0:
                self.panel_a.update_telemetry(active_telem, packet, t)
            elif idx == 1:
                self.panel_b.update_telemetry(active_telem, packet, t)
        elif idx == 2:
            packet = self.telemetry_mgr.last_cansat_packet
            if not packet:
                return
            t = packet.get("time_ms", 0) / 1000.0
            # For CanSat, active_telem is just the packet itself
            self.panel_payload.update_telemetry(packet, packet, t)

        # Update timelines for inactive panels so they're ready when switched to
        if idx != 0 and self.telemetry_mgr.last_packet:
            at = self.controller_mgr.get_active_telemetry(self.telemetry_mgr.last_packet)
            self.panel_a.timeline.set_state(at.get("state", 0))
        if idx != 1 and self.telemetry_mgr.last_packet:
            at = self.controller_mgr.get_active_telemetry(self.telemetry_mgr.last_packet)
            self.panel_b.timeline.set_state(at.get("state", 0))
        if idx != 2 and self.telemetry_mgr.last_cansat_packet:
            self.panel_payload.timeline.set_state(self.telemetry_mgr.last_cansat_packet.get("state", 0))

        # Map window update
        if self.map_window:
            map_lat, map_lon, map_alt = 0.0, 0.0, 0.0
            if idx in (0, 1) and self.telemetry_mgr.last_packet:
                at = self.controller_mgr.get_active_telemetry(self.telemetry_mgr.last_packet)
                map_lat = at.get("lat", 0.0)
                map_lon = at.get("lon", 0.0)
                map_alt = at.get("gps_alt", 0.0)
            elif idx == 2 and self.telemetry_mgr.last_cansat_packet:
                cp = self.telemetry_mgr.last_cansat_packet
                map_lat = cp.get("lat", 0.0)
                map_lon = cp.get("lon", 0.0)
                map_alt = cp.get("alt", 0.0)

            if map_lat != 0.0 or map_lon != 0.0:
                self.map_window.update_location(map_lat, map_lon, map_alt)

    def _tick_debug(self):
        """1 Hz: run health check, update status label, feed debug console."""
        packet = self.telemetry_mgr.last_packet
        summary = self.debug_mgr.evaluate(
            packet,
            self.telemetry_mgr,
            self.controller_mgr,
            self.connection_mgr,
        )
        self.lbl_status.setText(f"Status: {summary.text}")
        self.lbl_status.setStyleSheet(
            f"font-size: 12px; color: {summary.color}; font-weight: bold;"
        )

        # Pull new log entries and distribute to all debug consoles
        if self.debug_mgr.has_new_entries():
            entries = self.debug_mgr.get_new_entries()
            self.panel_a.feed_debug(entries)
            self.panel_b.feed_debug(entries)
            self.panel_payload.feed_debug(entries)

    # ──────────────────────────────────────────────────────────────────
    # Sub-window helpers
    # ──────────────────────────────────────────────────────────────────

    def _open_animation(self):
        if not _ANIMATION_AVAILABLE:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "Animation Unavailable",
                "3D animation requires PyOpenGL.\n"
                "Install it with: pip install PyOpenGL")
            return
        if self.anim_window is None:
            self.anim_window = AnimationWindow()
        self.anim_window.show()

    def _open_map(self):
        if not _MAP_AVAILABLE:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "Map Unavailable",
                "Live map tracking requires PyQt6-WebEngine.\n"
                "Install it with: pip install PyQt6-WebEngine")
            return
        if self.map_window is None:
            self.map_window = MapWindow()
        self.map_window.show()

    def _save_all_data(self):
        """Combined save: CSV + animation export + graphs + PDF report."""
        LoggingManager.exportFullCSV(self.telemetry_mgr)
        PDFReport.generate(
            self.telemetry_mgr.buffer_a,
            self.panel_a.plot_accel,
            self.panel_a.plot_alt,
        )
        if self.anim_window and self.anim_window.recording:
            self.anim_window.recording = False
            self.anim_window.save_video()

    def _auto_save(self):
        if self.auto_saved:
            return
        self.auto_saved = True
        self._save_all_data()

    def _on_clients_updated(self, clients):
        # Could show in status label if needed
        pass
