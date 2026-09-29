"""
Avionics Panel — Ctrl_A / Ctrl_B view (Screen 1).

Layout matches the reference image exactly:
  Left col   : BarGauge (H), SemicircleGauge (A), SemicircleGauge (V),
                GPS block, Radio block, Orientation block, Batt/Env block,
                Debug console
  Center top : FlightStateArc (TimelineWidget)
  Center mid : Accel XYZ line plot (Ax/Ay/Az)
  Center bot : Altitude overlay plot (H_baro / Alt)
  Right col  : Command button stack + Additional_options flyout
               QNH input (just above calibrate)
"""
import struct

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QDoubleSpinBox, QSizePolicy,
    QSpacerItem,
)
from PyQt6.QtCore import Qt, pyqtSignal

from gui.bar_gauge import BarGauge
from gui.semicircle_gauge import SemicircleGauge
from gui.debug_console import DebugConsole
from gui.timeline_widget import TimelineWidget
from gui.plots import LivePlot
from gui.command_button import CommandButton
from gui.confirmation_dialog import ConfirmationDialog

from core.command_manager import (
    CommandManager,
    CMD_START_TELEMETRY, CMD_STOP_TELEMETRY, CMD_REQUEST_STATUS,
    CMD_BUZZER_PLAY, CMD_CALIBRATION_MODE, CMD_SENSOR_ZERO,
    CMD_SERVO_PARACHUTE, CMD_SERVO_PAYLOAD, CMD_RESET_CONTROLLER,
    CMD_SET_QNH,
    CMD_NAMES,
)


def _info_block(title, fields):
    """Build a compact labeled-field block QFrame."""
    frame = QFrame()
    frame.setObjectName("Card")
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(8, 6, 8, 6)
    lay.setSpacing(2)

    title_lbl = QLabel(title)
    title_lbl.setObjectName("CardTitle")
    lay.addWidget(title_lbl)

    labels = {}
    for name in fields:
        row = QHBoxLayout()
        row.setSpacing(4)
        fl = QLabel(f"{name}:")
        fl.setObjectName("FieldLabel")
        fl.setFixedWidth(60)
        vl = QLabel("--")
        vl.setObjectName("ValueLabel")
        row.addWidget(fl)
        row.addWidget(vl)
        lay.addLayout(row)
        labels[name] = vl

    return frame, labels


class AvionicsPanel(QWidget):
    """Ctrl_A / Ctrl_B avionics telemetry and command screen."""

    def __init__(self, command_mgr: CommandManager, parent=None):
        super().__init__(parent)
        self.command_mgr = command_mgr
        self._flyout_visible = False

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
        lay.setSpacing(4)

        # Altitude bar gauge + semicircle gauges side-by-side at the top
        gauges_row = QHBoxLayout()
        gauges_row.setSpacing(4)

        self.bar_alt = BarGauge("H", 1000)
        self.bar_alt.setMaximumWidth(60)
        gauges_row.addWidget(self.bar_alt)

        semi_col = QVBoxLayout()
        semi_col.setSpacing(4)
        self.gauge_accel = SemicircleGauge("A", 1000.0, ("Ax", "Ay", "Az"))
        self.gauge_vel   = SemicircleGauge("V", 1000.0, ("Vx", "Vy", "Vz"))
        self.gauge_accel.setMinimumHeight(120)
        self.gauge_vel.setMinimumHeight(120)
        semi_col.addWidget(self.gauge_accel)
        semi_col.addWidget(self.gauge_vel)
        gauges_row.addLayout(semi_col)

        lay.addLayout(gauges_row)

        # GPS block
        gps_frame, self._gps_lbl = _info_block(
            "GPS", ["Lat", "Lon", "Alt", "SatCnt"])
        lay.addWidget(gps_frame)

        # Radio block
        radio_frame, self._radio_lbl = _info_block(
            "Radio", ["Radio_connected", "Rssi", "SNR", "Packet_loss"])
        lay.addWidget(radio_frame)

        # Orientation block (gyro + euler)
        orient_frame = QFrame()
        orient_frame.setObjectName("Card")
        orient_lay = QVBoxLayout(orient_frame)
        orient_lay.setContentsMargins(8, 6, 8, 6)
        orient_lay.setSpacing(2)
        t = QLabel("Orientation"); t.setObjectName("CardTitle")
        orient_lay.addWidget(t)

        gyro_row = QHBoxLayout()
        euler_row = QHBoxLayout()
        self._orient_lbl = {}
        for name in ("Gx", "Gy", "Gz"):
            fl = QLabel(f"{name}:"); fl.setObjectName("FieldLabel"); fl.setFixedWidth(26)
            vl = QLabel("--"); vl.setObjectName("ValueLabel")
            gyro_row.addWidget(fl); gyro_row.addWidget(vl)
            self._orient_lbl[name] = vl
        for name in ("P", "Y", "R"):
            fl = QLabel(f"{name}:"); fl.setObjectName("FieldLabel"); fl.setFixedWidth(18)
            vl = QLabel("--"); vl.setObjectName("ValueLabel")
            euler_row.addWidget(fl); euler_row.addWidget(vl)
            self._orient_lbl[name] = vl
        orient_lay.addLayout(gyro_row)
        orient_lay.addLayout(euler_row)
        lay.addWidget(orient_frame)

        # Battery / environment block
        batt_frame, self._batt_lbl = _info_block(
            "Batt / Env", ["Batt", "Temp", "Pressure"])
        lay.addWidget(batt_frame)

        # Debug console
        self.debug_console = DebugConsole("Debug")
        self.debug_console.setMinimumHeight(100)
        self.debug_console.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        lay.addWidget(self.debug_console)

        return lay

    # ── Center column ─────────────────────────────────────────────────

    def _build_center_col(self):
        lay = QVBoxLayout()
        lay.setSpacing(6)

        # Flight-state arc
        self.timeline = TimelineWidget()
        self.timeline.setMinimumHeight(140)
        self.timeline.setMaximumHeight(180)
        lay.addWidget(self.timeline)

        # Accel XYZ plot
        self.plot_accel = LivePlot(
            title="Accel (m/s²)",
            curve_names=["Ax", "Ay", "Az"],
            colors=["#FF4444", "#44FF88", "#4488FF"],
        )
        self.plot_accel.setMinimumHeight(160)
        lay.addWidget(self.plot_accel)

        # Altitude overlay plot
        self.plot_alt = LivePlot(
            title="Altitude (m)",
            curve_names=["H_baro", "Alt"],
            colors=["#FFD700", "#FF4444"],
        )
        self.plot_alt.setMinimumHeight(160)
        lay.addWidget(self.plot_alt)

        return lay

    # ── Right column — command stack ───────────────────────────────────

    def _build_right_col(self):
        lay = QVBoxLayout()
        lay.setSpacing(5)
        lay.setContentsMargins(0, 0, 0, 0)

        def _btn(label, cmd_type, warning=False):
            b = CommandButton(label, cmd_type, warning=warning)
            b.setMinimumWidth(148)
            b.setFixedHeight(36)
            return b

        # --- start_Tx ---
        self.btn_start_tx = _btn("start_Tx", CMD_START_TELEMETRY)
        self.btn_start_tx.setToolTip("Begin telemetry transmission")
        lay.addWidget(self.btn_start_tx)

        # --- stop_Tx ---
        self.btn_stop_tx = _btn("stop_Tx", CMD_STOP_TELEMETRY)
        self.btn_stop_tx.setToolTip("Stop telemetry transmission")
        lay.addWidget(self.btn_stop_tx)

        # --- Status ---
        self.btn_status = _btn("Status", CMD_REQUEST_STATUS)
        lay.addWidget(self.btn_status)

        # --- buzzer_play ---
        self.btn_buzzer = _btn("buzzer_play", CMD_BUZZER_PLAY)
        lay.addWidget(self.btn_buzzer)

        # --- QNH input (small, above calibrate) ---
        qnh_frame = QFrame()
        qnh_frame.setObjectName("Card")
        qnh_lay = QHBoxLayout(qnh_frame)
        qnh_lay.setContentsMargins(6, 4, 6, 4)
        qnh_lay.setSpacing(4)
        qnh_lbl = QLabel("QNH hPa:")
        qnh_lbl.setObjectName("FieldLabel")
        qnh_lbl.setFixedWidth(64)
        self.qnh_spin = QDoubleSpinBox()
        self.qnh_spin.setRange(900.0, 1100.0)
        self.qnh_spin.setValue(1013.25)
        self.qnh_spin.setDecimals(2)
        self.qnh_spin.setSingleStep(0.25)
        self.qnh_spin.setFixedWidth(80)
        self.qnh_spin.setStyleSheet(
            "QDoubleSpinBox { background:#1a1d24; color:#fff; "
            "border:1px solid rgba(187,134,252,0.3); border-radius:4px; padding:2px 4px; }"
            "QDoubleSpinBox::up-button,QDoubleSpinBox::down-button { width:14px; }"
        )
        self.btn_set_qnh = QPushButton("Set")
        self.btn_set_qnh.setFixedWidth(36)
        self.btn_set_qnh.setFixedHeight(24)
        self.btn_set_qnh.setStyleSheet(
            "QPushButton { background-color: #4b0082; color:white; "
            "border-radius:4px; font-size:11px; padding:2px 4px; }"
            "QPushButton:hover { background-color: #6a00b8; }"
        )
        qnh_lay.addWidget(qnh_lbl)
        qnh_lay.addWidget(self.qnh_spin)
        qnh_lay.addWidget(self.btn_set_qnh)
        lay.addWidget(qnh_frame)

        # --- calibrate ---
        self.btn_calibrate = _btn("calibrate", CMD_SENSOR_ZERO)
        lay.addWidget(self.btn_calibrate)

        # --- Servo_parachute (warning style) ---
        self.btn_servo_para = _btn("Servo_parachute", CMD_SERVO_PARACHUTE, warning=True)
        self.btn_servo_para.setToolTip("⚠ One-shot parachute servo cycle — confirm required")
        lay.addWidget(self.btn_servo_para)

        # --- Servo_payload ---
        self.btn_servo_payload = _btn("Servo_payload", CMD_SERVO_PAYLOAD)
        self.btn_servo_payload.setToolTip("One-shot payload release servo cycle — confirm required")
        lay.addWidget(self.btn_servo_payload)

        # --- Reset_controller ---
        self.btn_reset = _btn("Reset_controller", CMD_RESET_CONTROLLER, warning=True)
        self.btn_reset.setToolTip("Reset controller — only valid in LAUNCH_PAD state")
        lay.addWidget(self.btn_reset)

        lay.addSpacerItem(QSpacerItem(0, 4))

        # --- Additional_options toggle ---
        self.btn_additional = QPushButton("Additional_options ▸")
        self.btn_additional.setFixedHeight(36)
        self.btn_additional.setMinimumWidth(148)
        self.btn_additional.setCheckable(True)
        self.btn_additional.setStyleSheet(
            "QPushButton { background-color: #1e2128; color: #BB86FC; "
            "border: 1px solid rgba(187,134,252,0.5); border-radius:8px; "
            "padding:7px 12px; font-weight:bold; font-size:12px; }"
            "QPushButton:checked { background-color: rgba(139,43,226,0.3); }"
            "QPushButton:hover { background-color: rgba(139,43,226,0.2); }"
        )
        lay.addWidget(self.btn_additional)

        # --- Flyout panel ---
        self.flyout = QFrame()
        self.flyout.setObjectName("Card")
        flyout_lay = QVBoxLayout(self.flyout)
        flyout_lay.setContentsMargins(6, 6, 6, 6)
        flyout_lay.setSpacing(4)
        self.flyout.hide()

        flyout_btns = [
            ("Save_csv",        "save_csv"),
            ("Save_checkpoint", "save_ckpt"),
            ("Show_map",        "show_map"),
            ("Show_animation",  "show_anim"),
            ("save_data",       "save_data"),
            ("Start_server",    "start_srv"),
            ("Stop_server",     "stop_srv"),
        ]
        self._flyout_btns = {}
        for label, key in flyout_btns:
            b = QPushButton(label)
            b.setFixedHeight(30)
            b.setStyleSheet(
                "QPushButton { background-color: #1a1d24; color: #c5c6c7; "
                "border:1px solid rgba(187,134,252,0.2); border-radius:6px; "
                "font-size: 11px; padding: 4px 8px; }"
                "QPushButton:hover { background-color: rgba(139,43,226,0.25); color:#BB86FC; }"
            )
            flyout_lay.addWidget(b)
            self._flyout_btns[key] = b

        lay.addWidget(self.flyout)
        lay.addStretch()

        return lay

    # ──────────────────────────────────────────────────────────────────
    # Signal connections
    # ──────────────────────────────────────────────────────────────────

    def _connect_cmd_signals(self):
        # Map command buttons to their send logic
        self.btn_start_tx.clicked.connect(self._send_start_tx)
        self.btn_stop_tx.clicked.connect(self._send_stop_tx)
        self.btn_status.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_REQUEST_STATUS))
        self.btn_buzzer.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_BUZZER_PLAY))
        self.btn_calibrate.clicked.connect(
            lambda: self.command_mgr.send_command(CMD_SENSOR_ZERO))
        self.btn_set_qnh.clicked.connect(self._send_qnh)
        self.btn_servo_para.clicked.connect(self._confirm_servo_para)
        self.btn_servo_payload.clicked.connect(self._confirm_servo_payload)
        self.btn_reset.clicked.connect(self._confirm_reset)
        self.btn_additional.toggled.connect(self._toggle_flyout)

        # CommandManager state feedback → button visual state
        self.command_mgr.command_state_changed.connect(self._on_cmd_state)

        # All command buttons registered
        self._cmd_buttons = {
            CMD_START_TELEMETRY: self.btn_start_tx,
            CMD_STOP_TELEMETRY:  self.btn_stop_tx,
            CMD_REQUEST_STATUS:  self.btn_status,
            CMD_BUZZER_PLAY:     self.btn_buzzer,
            CMD_SENSOR_ZERO:     self.btn_calibrate,
            CMD_SERVO_PARACHUTE: self.btn_servo_para,
            CMD_SERVO_PAYLOAD:   self.btn_servo_payload,
            CMD_RESET_CONTROLLER: self.btn_reset,
        }

    # ──────────────────────────────────────────────────────────────────
    # Command send helpers
    # ──────────────────────────────────────────────────────────────────

    def _send_start_tx(self):
        self.command_mgr.send_command(CMD_START_TELEMETRY)
        # Visual: stop_Tx changes to "can resume" styling when Tx has ever started
        self._update_tx_button_styles()

    def _send_stop_tx(self):
        self.command_mgr.send_command(CMD_STOP_TELEMETRY)
        self._update_tx_button_styles()

    def _update_tx_button_styles(self):
        """Visually distinguish never-started vs. stopped-can-resume."""
        if not self.command_mgr.tx_ever_started:
            # Session fresh — stop_Tx dimmed
            self.btn_stop_tx.setEnabled(False)
        elif self.command_mgr.tx_active:
            # Tx running — start dimmed, stop bright
            self.btn_start_tx.setEnabled(False)
            self.btn_stop_tx.setEnabled(True)
        else:
            # Was running, now stopped — start shows "Resume" label
            self.btn_start_tx.setText("resume_Tx")
            self.btn_start_tx.setEnabled(True)
            self.btn_stop_tx.setEnabled(False)

    def _send_qnh(self):
        val = self.qnh_spin.value()
        payload = struct.pack("<f", val)
        self.command_mgr.send_command(CMD_SET_QNH, payload)

    def _confirm_servo_para(self):
        ok = ConfirmationDialog.confirm(
            self,
            "Fire Parachute Servo",
            "This will trigger the parachute bay servo (full 0→90→0° auto-cycle).\n"
            "The avionics firmware handles the entire sequence — this cannot be undone.",
            warning=True,
        )
        if ok:
            self.command_mgr.send_command(CMD_SERVO_PARACHUTE)

    def _confirm_servo_payload(self):
        ok = ConfirmationDialog.confirm(
            self,
            "Fire Payload Release Servo",
            "This will trigger the CanSat release servo (full 0→90→0° auto-cycle).\n"
            "The avionics firmware handles the entire sequence — this cannot be undone.",
        )
        if ok:
            self.command_mgr.send_command(CMD_SERVO_PAYLOAD)

    def _confirm_reset(self):
        ok = ConfirmationDialog.confirm(
            self,
            "Reset Controller",
            "Reset the active flight controller.\n"
            "This command is only honored in LAUNCH_PAD state.",
            warning=True,
        )
        if ok:
            self.command_mgr.send_command(CMD_RESET_CONTROLLER)

    def _toggle_flyout(self, checked):
        self.flyout.setVisible(checked)
        self.btn_additional.setText(
            "Additional_options ▾" if checked else "Additional_options ▸"
        )

    # ──────────────────────────────────────────────────────────────────
    # State updates from CommandManager
    # ──────────────────────────────────────────────────────────────────

    def _on_cmd_state(self, cmd_type: int, state_str: str):
        btn = self._cmd_buttons.get(cmd_type)
        if btn:
            btn.set_state(state_str)

    # ──────────────────────────────────────────────────────────────────
    # Telemetry update (called from MainWindow at 10 Hz)
    # ──────────────────────────────────────────────────────────────────

    def update_telemetry(self, active_telem: dict, packet: dict, t: float):
        """Push fresh telemetry into all widgets on this panel."""

        # Gauges
        ax = active_telem.get("ax", 0.0)
        ay = active_telem.get("ay", 0.0)
        az = active_telem.get("az", 0.0)
        self.gauge_accel.set_values(ax, ay, az)

        vx = active_telem.get("vx", 0.0)
        vy = active_telem.get("vy", 0.0)
        vz = active_telem.get("vz", 0.0)
        self.gauge_vel.set_values(vx, vy, vz)

        self.bar_alt.set_value(active_telem.get("baro_alt", 0.0))

        # Timeline
        self.timeline.set_state(active_telem.get("state", 0))

        # Plots
        self.plot_accel.add_point("Ax", t, ax)
        self.plot_accel.add_point("Ay", t, ay)
        self.plot_accel.add_point("Az", t, az)
        self.plot_alt.add_point("H_baro", t, active_telem.get("baro_alt", 0.0))
        self.plot_alt.add_point("Alt",    t, active_telem.get("gps_alt",  0.0))

        # GPS block
        g = self._gps_lbl
        g["Lat"].setText(f"{active_telem.get('lat', 0.0):.6f}")
        g["Lon"].setText(f"{active_telem.get('lon', 0.0):.6f}")
        g["Alt"].setText(f"{active_telem.get('gps_alt', 0.0):.1f} m")
        g["SatCnt"].setText(str(active_telem.get("gps_sats", "--")))

        # Radio block
        r = self._radio_lbl
        connected = packet.get("radio_connected", True)
        r["Radio_connected"].setText("YES" if connected else "NO")
        r["Radio_connected"].setStyleSheet("color: #00FF88;" if connected else "color: #FF4444;")
        r["Rssi"].setText(f"{packet.get('signal_strength', '--')} dB")
        r["SNR"].setText(f"{packet.get('snr', '--'):.1f} dB")
        loss = packet.get("packet_loss_pct", 0.0)
        r["Packet_loss"].setText(f"{loss:.1f}%")

        # Orientation
        o = self._orient_lbl
        o["Gx"].setText(f"{active_telem.get('gx', 0.0):.1f}")
        o["Gy"].setText(f"{active_telem.get('gy', 0.0):.1f}")
        o["Gz"].setText(f"{active_telem.get('gz', 0.0):.1f}")
        o["P"].setText(f"{active_telem.get('pitch', 0.0):.1f}")
        o["Y"].setText(f"{active_telem.get('yaw',   0.0):.1f}")
        o["R"].setText(f"{active_telem.get('roll',  0.0):.1f}")

        # Batt/Env
        b = self._batt_lbl
        b["Batt"].setText(f"{active_telem.get('voltage', 0.0):.2f} V")
        b["Temp"].setText(f"{active_telem.get('temperature', 0.0):.1f} °C")
        b["Pressure"].setText(f"{active_telem.get('pressure', 0.0):.1f} hPa")

        # Grey out Reset_controller when not in LAUNCH_PAD state (state == 2)
        in_launchpad = (active_telem.get("state", 0) == 2)
        self.btn_reset.setEnabled(in_launchpad)
        self.btn_reset.setToolTip(
            "Reset controller — only valid in LAUNCH_PAD state"
            if in_launchpad else
            "⚠ Reset only valid in LAUNCH_PAD state (currently greyed out)"
        )

        # Tx button state sync
        self._update_tx_button_styles()

    def feed_debug(self, entries):
        """Append new log entries to the debug console."""
        self.debug_console.append_entries(entries)
