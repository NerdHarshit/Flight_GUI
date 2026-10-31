import struct

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QDoubleSpinBox, QSpinBox,
    QSizePolicy, QSpacerItem, QFormLayout, QGroupBox,
)
from PyQt6.QtCore import Qt, pyqtSignal

import pyqtgraph as pg
import numpy as np

from gui.debug_console import DebugConsole
from gui.timeline_widget import TimelineWidget
from gui.plots import LivePlot
from gui.command_button import CommandButton
from gui.confirmation_dialog import ConfirmationDialog

from core.command_manager import (
    CommandManager,
    CMD_START_TELEMETRY, CMD_STOP_TELEMETRY, CMD_REQUEST_STATUS,
    CMD_BUZZER_PLAY, CMD_SENSOR_ZERO,
    CMD_RESET_CONTROLLER,
    CMD_SET_TARGET_GPS,
    CMD_GYRO_STAB_ON, CMD_GYRO_STAB_OFF, CMD_BURN_NICHROME,
)



def _info_block(title, fields, columns=1):
    """Build a compact labeled-field block QFrame."""
    frame = QFrame()
    frame.setObjectName("Card")
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(8, 6, 8, 6)
    lay.setSpacing(2)
    t = QLabel(title); t.setObjectName("CardTitle")
    lay.addWidget(t)
    
    grid = QGridLayout()
    grid.setSpacing(4)
    lay.addLayout(grid)
    
    labels = {}
    for i, name in enumerate(fields):
        row = i // columns
        col = (i % columns) * 2
        
        fl = QLabel(f"{name}:"); fl.setObjectName("FieldLabel")
        vl = QLabel("--"); vl.setObjectName("ValueLabel")
        
        grid.addWidget(fl, row, col)
        grid.addWidget(vl, row, col + 1)
        labels[name] = vl
        
    return frame, labels


class _GpsMapPlot(QFrame):
    """Live GPS path plot using pyqtgraph (lon=X, lat=Y scatter + trail)."""

    def __init__(self):
        super().__init__()
        self.setObjectName("Card")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)

        title = QLabel("GPS Track (Lon / Lat)")
        title.setObjectName("CardTitle")
        lay.addWidget(title)

        self._plot = pg.PlotWidget(background="#0b0c10")
        self._plot.showGrid(x=True, y=True, alpha=0.15)
        self._plot.setLabel("bottom", "Longitude")
        self._plot.setLabel("left", "Latitude")
        self._plot.setAspectLocked(True)
        lay.addWidget(self._plot)

        # Trail
        self._trail = self._plot.plot(
            [], [], pen=pg.mkPen("#BB86FC", width=2))
        # Current position marker
        self._marker = pg.ScatterPlotItem(
            size=10, brush=pg.mkBrush("#00FF88"), pen=pg.mkPen(None))
        self._plot.addItem(self._marker)

        # Target marker
        self._target = pg.ScatterPlotItem(
            size=12, symbol="x", brush=pg.mkBrush("#FF4444"),
            pen=pg.mkPen("#FF4444", width=2))
        self._plot.addItem(self._target)

        self._lons = []
        self._lats = []

    def update_pos(self, lat: float, lon: float):
        if lat == 0.0 and lon == 0.0:
            return
        self._lons.append(lon)
        self._lats.append(lat)
        # Keep trail manageable
        if len(self._lons) > 2000:
            self._lons = self._lons[-2000:]
            self._lats = self._lats[-2000:]
        self._trail.setData(self._lons, self._lats)
        self._marker.setData([lon], [lat])

    def set_target(self, lat: float, lon: float):
        self._target.setData([lon], [lat])


class PayloadPanel(QWidget):
    """CanSat telemetry, mapping and command screen."""

    def __init__(self, command_mgr: CommandManager, parent=None):
        super().__init__(parent)
        self.command_mgr = command_mgr

        self._build_ui()
        self._connect_cmd_signals()

    # ──────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        root.addLayout(self._build_left_col(), 0)
        root.addLayout(self._build_center_col(), 1)
        root.addLayout(self._build_right_col(), 0)

    # ── Left column ──────────────────────────────────────────────────

    def _build_left_col(self):
        lay = QVBoxLayout()
        lay.setSpacing(5)

        # Flight state arc
        self.timeline = TimelineWidget()
        self.timeline.setMinimumHeight(140)
        self.timeline.setMaximumHeight(180)
        lay.addWidget(self.timeline)

        # Pitch/Yaw/Roll plot
        self.plot_pyr = LivePlot(
            title="Orientation (°)",
            curve_names=["Pitch", "Yaw", "Roll"],
            colors=["#FF4444", "#44FF88", "#4488FF"],
        )
        self.plot_pyr.setMinimumHeight(160)
        lay.addWidget(self.plot_pyr)

        # Ax/Ay/Az plot
        self.plot_acc = LivePlot(
            title="Accel (m/s²)",
            curve_names=["Ax", "Ay", "Az"],
            colors=["#FF4444", "#44FF88", "#4488FF"],
        )
        self.plot_acc.setMinimumHeight(160)
        lay.addWidget(self.plot_acc)

        return lay

    # ── Center column ─────────────────────────────────────────────────

    def _build_center_col(self):
        lay = QVBoxLayout()
        lay.setSpacing(5)

        # Radio + Debug top section
        top_row = QHBoxLayout()
        top_row.setSpacing(5)

        radio_frame, self._radio_lbl = _info_block(
            "Radio", ["Radio_connected", "Rssi", "SNR", "Packet_loss"])
        radio_frame.setFixedWidth(200)
        top_row.addWidget(radio_frame)

        self.debug_console = DebugConsole("Debug")
        self.debug_console.setMinimumHeight(80)
        self.debug_console.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        top_row.addWidget(self.debug_console)

        lay.addLayout(top_row)

        # GPS track plot (pyqtgraph — live Lon/Lat path)
        self.gps_map = _GpsMapPlot()
        self.gps_map.setMinimumHeight(220)
        self.gps_map.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        lay.addWidget(self.gps_map)

        # GPS / position block below map
        gps_frame, self._gps_lbl = _info_block(
            "Position", ["H", "COG", "Lat", "headingErr", "Lon", "distToTarget", "Alt", "", "SatCnt", "", "Heading", ""], columns=2)
        lay.addWidget(gps_frame)

        return lay

    # ── Right column — command stack ───────────────────────────────────

    def _build_right_col(self):
        lay = QVBoxLayout()
        lay.setSpacing(5)

        def _btn(label, cmd_type, warning=False):
            b = CommandButton(label, cmd_type, warning=warning)
            b.setFixedHeight(34)
            b.setMinimumWidth(148)
            return b

        # --- start_Tx / stop_Tx ---
        self.btn_start_tx = _btn("start_Tx", CMD_START_TELEMETRY)
        self.btn_stop_tx  = _btn("stop_Tx",  CMD_STOP_TELEMETRY)
        lay.addWidget(self.btn_start_tx)
        lay.addWidget(self.btn_stop_tx)

        # --- Status / buzzer_play ---
        self.btn_status = _btn("Status",      CMD_REQUEST_STATUS)
        self.btn_buzzer = _btn("buzzer_play", CMD_BUZZER_PLAY)
        lay.addWidget(self.btn_status)
        lay.addWidget(self.btn_buzzer)

        # --- calibrate ---
        self.btn_calibrate = _btn("calibrate", CMD_SENSOR_ZERO)
        lay.addWidget(self.btn_calibrate)

        # --- Reset_controller ---
        self.btn_reset = _btn("Reset_controller", CMD_RESET_CONTROLLER, warning=True)
        lay.addWidget(self.btn_reset)

        # ── CanSat-Specific Commands Section ─────────────────────────
        lay.addSpacerItem(QSpacerItem(0, 8))

        cansat_sep = QFrame()
        cansat_sep.setFrameShape(QFrame.Shape.HLine)
        cansat_sep.setStyleSheet("border: 1px solid rgba(187,134,252,0.3);")
        lay.addWidget(cansat_sep)

        cansat_label = QLabel("🛰  CanSat Commands")
        cansat_label.setStyleSheet(
            "color:#BB86FC; font-size:12px; font-weight:bold; padding:2px 0px;")
        lay.addWidget(cansat_label)

        # Gyro Stabilizer ON
        self.btn_gyro_on = QPushButton("⟳  Gyro Stabilizer ON")
        self.btn_gyro_on.setFixedHeight(36)
        self.btn_gyro_on.setMinimumWidth(148)
        self.btn_gyro_on.setStyleSheet(
            "QPushButton { background-color: qlineargradient("
            "x1:0,y1:0,x2:1,y2:1,stop:0 #0a3d2e,stop:1 #06261c); "
            "color:#00FF88; border:2px solid rgba(0,255,136,0.5); "
            "border-radius:8px; font-weight:bold; font-size:12px; padding:7px;}"
            "QPushButton:hover { background-color: #0d5240; border-color:#00FF88; }"
            "QPushButton:pressed { background-color: #116b52; }"
        )
        lay.addWidget(self.btn_gyro_on)

        # Gyro Stabilizer OFF
        self.btn_gyro_off = QPushButton("⏹  Gyro Stabilizer OFF")
        self.btn_gyro_off.setFixedHeight(36)
        self.btn_gyro_off.setMinimumWidth(148)
        self.btn_gyro_off.setStyleSheet(
            "QPushButton { background-color: qlineargradient("
            "x1:0,y1:0,x2:1,y2:1,stop:0 #3d2e0a,stop:1 #261c06); "
            "color:#FFB347; border:2px solid rgba(255,179,71,0.5); "
            "border-radius:8px; font-weight:bold; font-size:12px; padding:7px;}"
            "QPushButton:hover { background-color: #52400d; border-color:#FFB347; }"
            "QPushButton:pressed { background-color: #6b5211; }"
        )
        lay.addWidget(self.btn_gyro_off)

        # Burn Nichrome Wire — high-consequence, visually distinct
        lay.addSpacing(4)

        burn_sep = QFrame()
        burn_sep.setFrameShape(QFrame.Shape.HLine)
        burn_sep.setStyleSheet("border: 1px solid rgba(255,80,80,0.3);")
        lay.addWidget(burn_sep)

        burn_label = QLabel("⚠  IRREVERSIBLE")
        burn_label.setStyleSheet("color:#FF4444; font-size:11px; font-weight:bold;")
        lay.addWidget(burn_label)

        self.btn_burn_nichrome = QPushButton("🔥  Burn Nichrome Wire")
        self.btn_burn_nichrome.setFixedHeight(38)
        self.btn_burn_nichrome.setMinimumWidth(148)
        self.btn_burn_nichrome.setStyleSheet(
            "QPushButton { background-color: qlineargradient("
            "x1:0,y1:0,x2:1,y2:1,stop:0 #7a0000,stop:1 #330000); "
            "color:#FF6666; border:2px solid rgba(255,80,80,0.6); "
            "border-radius:8px; font-weight:bold; font-size:12px; padding:7px;}"
            "QPushButton:hover { background-color: #880000; color:#FF8888; }"
            "QPushButton:pressed { background-color: #aa0000; }"
        )
        lay.addWidget(self.btn_burn_nichrome)

        # --- Additional_options flyout ---
        lay.addSpacerItem(QSpacerItem(0, 8))
        self.btn_additional = QPushButton("Additional_options ▸")
        self.btn_additional.setFixedHeight(34)
        self.btn_additional.setMinimumWidth(148)
        self.btn_additional.setCheckable(True)
        self.btn_additional.setStyleSheet(
            "QPushButton { background-color: #1e2128; color: #BB86FC; "
            "border: 1px solid rgba(187,134,252,0.5); border-radius:8px; "
            "font-weight:bold; font-size:12px; padding:7px; }"
            "QPushButton:checked { background-color: rgba(139,43,226,0.3); }"
            "QPushButton:hover { background-color: rgba(139,43,226,0.2); }"
        )
        lay.addWidget(self.btn_additional)

        self.flyout = QFrame()
        self.flyout.setObjectName("Card")
        flyout_lay = QVBoxLayout(self.flyout)
        flyout_lay.setContentsMargins(6, 6, 6, 6)
        flyout_lay.setSpacing(4)
        self.flyout.hide()

        for label, key in [
            ("Save_csv",        "save_csv"),
            ("Save_checkpoint", "save_ckpt"),
            ("Show_map",        "show_map"),
            ("Show_animation",  "show_anim"),
            ("save_data",       "save_data"),
            ("Start_server",    "start_srv"),
            ("Stop_server",     "stop_srv"),
        ]:
            b = QPushButton(label)
            b.setFixedHeight(28)
            b.setStyleSheet(
                "QPushButton { background-color: #1a1d24; color: #c5c6c7; "
                "border:1px solid rgba(187,134,252,0.2); border-radius:6px; "
                "font-size:11px; padding:3px 8px; }"
                "QPushButton:hover { background-color: rgba(139,43,226,0.25); color:#BB86FC; }"
            )
            flyout_lay.addWidget(b)
            setattr(self, f"btn_{key}", b)

        lay.addWidget(self.flyout)

        # ── Advanced / rare-use section ──────────────────────────────
        lay.addSpacerItem(QSpacerItem(0, 6))
        self.btn_advanced_toggle = QPushButton("Advanced ▸")
        self.btn_advanced_toggle.setCheckable(True)
        self.btn_advanced_toggle.setFixedHeight(30)
        self.btn_advanced_toggle.setMinimumWidth(148)
        self.btn_advanced_toggle.setStyleSheet(
            "QPushButton { background-color: #12141a; color: #888; "
            "border: 1px solid #333; border-radius:6px; font-size:11px; padding:5px; }"
            "QPushButton:checked { color: #BB86FC; border-color: rgba(187,134,252,0.4); }"
        )
        lay.addWidget(self.btn_advanced_toggle)

        self.advanced_frame = QFrame()
        self.advanced_frame.setObjectName("Card")
        adv_lay = QVBoxLayout(self.advanced_frame)
        adv_lay.setContentsMargins(8, 6, 8, 6)
        adv_lay.setSpacing(6)
        self.advanced_frame.hide()

        # Set target GPS
        gps_group = QFrame(); gps_group.setObjectName("Card")
        gps_lay = QFormLayout(gps_group)
        gps_lay.setContentsMargins(6, 4, 6, 4)
        gps_lay.setSpacing(4)
        lbl = QLabel("Target GPS"); lbl.setObjectName("CardTitle")
        adv_lay.addWidget(lbl)
        self.tgt_lat = QDoubleSpinBox()
        self.tgt_lat.setRange(-90, 90); self.tgt_lat.setDecimals(6); self.tgt_lat.setFixedWidth(110)
        self.tgt_lon = QDoubleSpinBox()
        self.tgt_lon.setRange(-180, 180); self.tgt_lon.setDecimals(6); self.tgt_lon.setFixedWidth(110)
        for w in (self.tgt_lat, self.tgt_lon):
            w.setStyleSheet(
                "QDoubleSpinBox { background:#1a1d24; color:#fff; "
                "border:1px solid rgba(187,134,252,0.3); border-radius:4px; padding:2px 4px;}"
            )
        gps_lay.addRow("Lat:", self.tgt_lat)
        gps_lay.addRow("Lon:", self.tgt_lon)
        adv_lay.addWidget(gps_group)

        self.btn_set_target = QPushButton("Set Target GPS")
        self.btn_set_target.setFixedHeight(30)
        self.btn_set_target.setStyleSheet(
            "QPushButton { background-color:#2a1060; color:#BB86FC; "
            "border:1px solid rgba(187,134,252,0.4); border-radius:6px; font-size:11px;}"
            "QPushButton:hover { background-color:#3a1880; }"
        )
        adv_lay.addWidget(self.btn_set_target)

        lay.addWidget(self.advanced_frame)
        lay.addStretch()

        return lay

    # ──────────────────────────────────────────────────────────────────
    # Signal connections
    # ──────────────────────────────────────────────────────────────────

    def _connect_cmd_signals(self):
        self.btn_start_tx.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_START_TELEMETRY))
        self.btn_stop_tx.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_STOP_TELEMETRY))
        self.btn_status.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_REQUEST_STATUS))
        self.btn_buzzer.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_BUZZER_PLAY))
        self.btn_calibrate.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_SENSOR_ZERO))

        self.btn_reset.clicked.connect(self._confirm_reset)
        self.btn_additional.toggled.connect(self._toggle_flyout)
        self.btn_advanced_toggle.toggled.connect(self._toggle_advanced)

        self.btn_set_target.clicked.connect(self._send_target_gps)

        # CanSat-specific commands
        self.btn_gyro_on.clicked.connect(self._confirm_gyro_on)
        self.btn_gyro_off.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_GYRO_STAB_OFF))
        self.btn_burn_nichrome.clicked.connect(self._confirm_burn_nichrome)

        self.command_mgr.command_state_changed.connect(self._on_cmd_state)

        self._cmd_buttons = {
            CMD_START_TELEMETRY: self.btn_start_tx,
            CMD_STOP_TELEMETRY:  self.btn_stop_tx,
            CMD_REQUEST_STATUS:  self.btn_status,
            CMD_BUZZER_PLAY:     self.btn_buzzer,
            CMD_SENSOR_ZERO:     self.btn_calibrate,
            CMD_RESET_CONTROLLER: self.btn_reset,
        }

    # ──────────────────────────────────────────────────────────────────
    # Command helpers
    # ──────────────────────────────────────────────────────────────────

    def _confirm_reset(self):
        ok = ConfirmationDialog.confirm(
            self,
            "Reset Controller",
            "Reset the payload controller.\n"
            "Only honored in LAUNCH_PAD state.",
            warning=True,
        )
        if ok:
            self.command_mgr.send_command(CMD_RESET_CONTROLLER)

    def _confirm_gyro_on(self):
        ok = ConfirmationDialog.confirm(
            self,
            "Enable Gyro Stabilizer",
            "This will activate the gyro stabilizer on the CanSat.\n"
            "Ensure the CanSat is in a safe orientation before enabling.",
        )
        if ok:
            self.command_mgr.send_command(CMD_GYRO_STAB_ON)

    def _confirm_burn_nichrome(self):
        ok = ConfirmationDialog.confirm(
            self,
            "Burn Nichrome Wire",
            "⚠  This will PERMANENTLY burn the nichrome wire.\n\n"
            "This action is IRREVERSIBLE and cannot be undone.\n"
            "Ensure the CanSat is at the correct deployment altitude.",
            warning=True,
        )
        if ok:
            self.command_mgr.send_command(CMD_BURN_NICHROME)

    def _toggle_flyout(self, checked):
        self.flyout.setVisible(checked)
        self.btn_additional.setText(
            "Additional_options ▾" if checked else "Additional_options ▸")

    def _toggle_advanced(self, checked):
        self.advanced_frame.setVisible(checked)
        self.btn_advanced_toggle.setText(
            "Advanced ▾" if checked else "Advanced ▸")

    def _send_target_gps(self):
        lat = self.tgt_lat.value()
        lon = self.tgt_lon.value()
        # GS firmware expects float32 lat + float32 lon, then converts to int32 E7
        payload = struct.pack("<ff", lat, lon)
        self.command_mgr.send_command(CMD_SET_TARGET_GPS, payload)
        # Show target on map
        self.gps_map.set_target(lat, lon)

    # ──────────────────────────────────────────────────────────────────
    # CommandManager feedback
    # ──────────────────────────────────────────────────────────────────

    def _on_cmd_state(self, cmd_type: int, state_str: str):
        btn = self._cmd_buttons.get(cmd_type)
        if btn:
            btn.set_state(state_str)

    # ──────────────────────────────────────────────────────────────────
    # Telemetry update
    # ──────────────────────────────────────────────────────────────────

    def update_telemetry(self, active_telem: dict, packet: dict, t: float):
        """Push telemetry into payload panel widgets."""

        # Timeline
        self.timeline.set_state(active_telem.get("state", 0))

        # Plots
        pitch = active_telem.get("pitch", 0.0)
        yaw   = active_telem.get("yaw",   0.0)
        roll  = active_telem.get("roll",  0.0)
        self.plot_pyr.add_point("Pitch", t, pitch)
        self.plot_pyr.add_point("Yaw",   t, yaw)
        self.plot_pyr.add_point("Roll",  t, roll)

        ax = active_telem.get("ax", 0.0)
        ay = active_telem.get("ay", 0.0)
        az = active_telem.get("az", 0.0)
        self.plot_acc.add_point("Ax", t, ax)
        self.plot_acc.add_point("Ay", t, ay)
        self.plot_acc.add_point("Az", t, az)

        # GPS map
        lat = active_telem.get("lat", 0.0)
        lon = active_telem.get("lon", 0.0)
        self.gps_map.update_pos(lat, lon)

        # GPS info block
        g = self._gps_lbl
        g["H"].setText(f"{packet.get('h_raw', active_telem.get('baro_alt', 0.0)):.1f} m")
        g["COG"].setText(f"{packet.get('COG', 0.0):.1f}°")
        g["Lat"].setText(f"{lat:.6f}")
        g["headingErr"].setText(f"{packet.get('headingErr', 0.0):.1f}°")
        g["Lon"].setText(f"{lon:.6f}")
        g["distToTarget"].setText(f"{packet.get('distToTarget', 0.0):.1f} m")
        g["Alt"].setText(f"{active_telem.get('gps_alt', 0.0):.1f} m")
        if "" in g:
            g[""].setText("")
        g["SatCnt"].setText(str(active_telem.get("gps_sats", "--")))
        g["Heading"].setText(f"{packet.get('heading', 0.0):.1f}°")

        # Radio block
        r = self._radio_lbl
        connected = packet.get("radio_connected", True)
        r["Radio_connected"].setText("YES" if connected else "NO")
        r["Radio_connected"].setStyleSheet("color:#00FF88;" if connected else "color:#FF4444;")
        rssi_val = packet.get('rssi', packet.get('signal_strength', '--'))
        r["Rssi"].setText(f"{rssi_val} dBm")
        snr_val = packet.get('snr', '--')
        r["SNR"].setText(f"{snr_val:.1f} dB" if isinstance(snr_val, (int, float)) else f"{snr_val} dB")
        loss_val = packet.get('packet_loss', packet.get('packet_loss_pct', 0.0))
        r["Packet_loss"].setText(f"{loss_val}%")

        # Grey-out reset outside LAUNCH_PAD
        in_launchpad = (active_telem.get("state", 0) == 2)
        self.btn_reset.setEnabled(in_launchpad)

    def feed_debug(self, entries):
        """Append log entries to debug console."""
        self.debug_console.append_entries(entries)
