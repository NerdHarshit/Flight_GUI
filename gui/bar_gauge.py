"""
Vertical Bar Gauge Widget — Thin vertical bar for altitude display.
Shows H value, H_MAX label, and a filled bar proportional to value.
"""
from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QColor, QFont, QLinearGradient, QBrush
from PyQt6.QtCore import Qt, QRectF


class BarGauge(QWidget):
    """Vertical bar gauge for altitude (H) display."""

    def __init__(self, label="H", max_val=1000.0):
        super().__init__()
        self.label = label
        self.max_val = max_val
        self.current_val = 0.0
        self.peak_val = 0.0
        self.setMinimumWidth(50)
        self.setMaximumWidth(70)
        self.setMinimumHeight(120)

    def set_value(self, val):
        self.current_val = val
        if val > self.peak_val:
            self.peak_val = val
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        # Layout
        top_text_h = 18
        bottom_text_h = 32
        bar_margin_x = 12
        bar_w = w - bar_margin_x * 2
        bar_h = h - top_text_h - bottom_text_h
        bar_x = bar_margin_x
        bar_y = top_text_h

        # --- Label at top ---
        painter.setPen(QColor(160, 170, 181))
        label_font = QFont("Inter", 10, QFont.Weight.Bold)
        painter.setFont(label_font)
        painter.drawText(QRectF(0, 0, w, top_text_h), Qt.AlignmentFlag.AlignCenter,
                         f"{self.label}:")

        # --- Bar background (dark track) ---
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(40, 44, 52))
        painter.drawRoundedRect(QRectF(bar_x, bar_y, bar_w, bar_h), 4, 4)

        # --- Bar fill (bottom to top, gradient) ---
        ratio = min(1.0, self.current_val / self.max_val) if self.max_val > 0 else 0
        fill_h = bar_h * ratio

        if fill_h > 0:
            fill_rect = QRectF(bar_x, bar_y + bar_h - fill_h, bar_w, fill_h)
            gradient = QLinearGradient(0, bar_y + bar_h, 0, bar_y)
            gradient.setColorAt(0.0, QColor(90, 50, 180))
            gradient.setColorAt(0.5, QColor(130, 80, 220))
            gradient.setColorAt(1.0, QColor(187, 134, 252))
            painter.setBrush(QBrush(gradient))
            painter.drawRoundedRect(fill_rect, 4, 4)

        # --- Peak marker ---
        if self.peak_val > 0:
            peak_ratio = min(1.0, self.peak_val / self.max_val)
            peak_y = bar_y + bar_h - bar_h * peak_ratio
            pen_peak = QPen(QColor(255, 200, 0), 2, Qt.PenStyle.DashLine)
            painter.setPen(pen_peak)
            painter.drawLine(int(bar_x - 2), int(peak_y), int(bar_x + bar_w + 2), int(peak_y))

        # --- Current value text ---
        painter.setPen(QColor(255, 255, 255))
        val_font = QFont("Consolas", 11, QFont.Weight.Bold)
        painter.setFont(val_font)
        painter.drawText(QRectF(0, h - bottom_text_h, w, 16),
                         Qt.AlignmentFlag.AlignCenter, f"{self.current_val:.0f}")

        # --- Max label ---
        painter.setPen(QColor(130, 80, 220))
        max_font = QFont("Inter", 8)
        painter.setFont(max_font)
        painter.drawText(QRectF(0, h - bottom_text_h + 16, w, 14),
                         Qt.AlignmentFlag.AlignCenter, f"{self.label}_MAX: {int(self.max_val)}")

        painter.end()
