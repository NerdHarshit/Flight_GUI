"""
Semicircle Gauge Widget — Multi-arc semicircular gauge showing
individual axis values on concentric arcs (e.g., Ax/Ay/Az or Vx/Vy/Vz).
"""
import math
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt6.QtGui import QPainter, QPen, QColor, QFont
from PyQt6.QtCore import Qt, QRectF


class SemicircleGauge(QWidget):
    """Semicircular gauge with 3 concentric arcs for X/Y/Z axis values."""

    def __init__(self, title="A", max_val=1000.0, labels=("Ax", "Ay", "Az")):
        super().__init__()
        self.title = title
        self.max_val = max_val
        self.labels = labels
        self.values = [0.0, 0.0, 0.0]
        self.peak_val = 0.0

        # Arc colors — match the purple/dark-purple theme from the images
        self.arc_colors = [
            QColor(187, 134, 252),     # bright purple (outer)
            QColor(130, 80, 220),      # mid purple
            QColor(90, 50, 180),       # dark purple (inner)
        ]
        self.setMinimumSize(200, 160)

    def set_values(self, v1, v2, v3):
        """Set the three axis values."""
        self.values = [v1, v2, v3]
        mag = math.sqrt(v1**2 + v2**2 + v3**2)
        if mag > self.peak_val:
            self.peak_val = mag
        self.update()

    def set_magnitude(self, mag):
        """Set a single magnitude value (uses the outer arc only)."""
        self.values[0] = mag
        if mag > self.peak_val:
            self.peak_val = mag
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        # Reserve space for title at top and readouts at bottom
        title_h = 20
        readout_h = 30
        gauge_h = h - title_h - readout_h

        # --- Draw title ---
        painter.setPen(QColor(160, 170, 181))
        title_font = QFont("Inter", 10, QFont.Weight.Bold)
        painter.setFont(title_font)
        painter.drawText(QRectF(0, 0, w, title_h), Qt.AlignmentFlag.AlignCenter,
                         f"{self.title}_MAX: {int(self.max_val)}")

        # --- Draw arcs ---
        arc_thickness = 8
        arc_gap = 4
        base_size = min(w - 20, gauge_h * 2 - 20)

        cx = w / 2
        cy = title_h + gauge_h - 5  # bottom of the semicircle area

        for i in range(3):
            shrink = i * (arc_thickness + arc_gap)
            size = base_size - shrink * 2
            if size < 30:
                continue

            rect = QRectF(cx - size / 2, cy - size / 2, size, size)

            # Background arc (dark track)
            pen_bg = QPen(QColor(40, 44, 52), arc_thickness, Qt.PenStyle.SolidLine,
                          Qt.PenCapStyle.RoundCap)
            painter.setPen(pen_bg)
            painter.drawArc(rect, 0 * 16, 180 * 16)  # bottom semicircle = 0° to 180°

            # Filled arc — proportion of max
            val = abs(self.values[i])
            ratio = min(1.0, val / self.max_val) if self.max_val > 0 else 0
            span_angle = int(180 * ratio)

            pen_fill = QPen(self.arc_colors[i], arc_thickness, Qt.PenStyle.SolidLine,
                            Qt.PenCapStyle.RoundCap)
            painter.setPen(pen_fill)
            painter.drawArc(rect, 0 * 16, span_angle * 16)

        # --- Draw max value in center ---
        painter.setPen(QColor(255, 255, 255))
        val_font = QFont("Consolas", 14, QFont.Weight.Bold)
        painter.setFont(val_font)
        mag = math.sqrt(sum(v**2 for v in self.values))
        painter.drawText(QRectF(0, cy - 35, w, 30), Qt.AlignmentFlag.AlignCenter,
                         f"{mag:.1f}")

        # --- Draw readouts at bottom ---
        painter.setPen(QColor(160, 170, 181))
        readout_font = QFont("Consolas", 9)
        painter.setFont(readout_font)

        parts = []
        for j, label in enumerate(self.labels):
            painter.setPen(self.arc_colors[j])
            parts.append(f"{label}: {self.values[j]:.1f}")

        text = "  |  ".join(parts)
        painter.setPen(QColor(160, 170, 181))
        painter.drawText(QRectF(0, h - readout_h, w, readout_h),
                         Qt.AlignmentFlag.AlignCenter, text)

        painter.end()
