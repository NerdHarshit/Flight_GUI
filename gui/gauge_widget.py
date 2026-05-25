import math
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QPainter, QPen, QColor, QFont, QConicalGradient, QBrush, QPolygonF
from PyQt6.QtCore import Qt, QRectF, QPointF

class GaugeWidget(QWidget):
    def __init__(self, title="Gauge", unit="", max_val=100.0):
        super().__init__()
        self.setMinimumSize(150, 150)
        self.title = title
        self.unit = unit
        self.max_val = max_val
        self.current_val = 0.0
        self.peak_val = 0.0

    def set_value(self, val):
        self.current_val = val
        if val > self.peak_val:
            self.peak_val = val
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()
        size = min(width, height) - 20
        rect = QRectF((width - size) / 2, (height - size) / 2, size, size)

        # Draw background track
        pen_bg = QPen(QColor(40, 44, 52), 12, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
        painter.setPen(pen_bg)
        painter.drawArc(int(rect.x()), int(rect.y()), int(rect.width()), int(rect.height()), 210 * 16, -240 * 16)

        # Draw filled track
        span_angle = -240 * min(1.0, self.current_val / self.max_val) if self.max_val > 0 else 0
        pen_fill = QPen(QColor(187, 134, 252), 12, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
        painter.setPen(pen_fill)
        painter.drawArc(int(rect.x()), int(rect.y()), int(rect.width()), int(rect.height()), 210 * 16, int(span_angle * 16))

        # Text: Current Value
        painter.setPen(QColor(255, 255, 255))
        font_val = QFont("Consolas", 18, QFont.Weight.Bold)
        painter.setFont(font_val)
        val_str = f"{self.current_val:.1f}"
        painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, val_str)

        # Text: Title & Unit
        font_title = QFont("Inter", 10)
        painter.setFont(font_title)
        painter.setPen(QColor(160, 170, 181))
        painter.drawText(
            QRectF(rect.x(), rect.y() + size/2 + 20, rect.width(), 30),
            Qt.AlignmentFlag.AlignCenter,
            f"{self.title} ({self.unit})"
        )

        # Text: Max Value
        font_max = QFont("Inter", 9)
        painter.setFont(font_max)
        painter.setPen(QColor(187, 134, 252))
        painter.drawText(
            QRectF(rect.x(), rect.y() + size/2 + 45, rect.width(), 20),
            Qt.AlignmentFlag.AlignCenter,
            f"Max: {self.peak_val:.1f}"
        )
