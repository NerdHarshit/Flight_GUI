import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QColor, QFont, QPainterPath
from PyQt6.QtCore import Qt, QRectF

class TimelineWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(160)
        self.states = [
            "BOOT", "TEST_MODE", "LAUNCH_PAD", "ASCENT", 
            "PAYLOAD_SEP", "DESCENT", "IMPACT", "SAFE_MODE"
        ]
        self.current_state_index = 0

    def set_state(self, state_index):
        if 0 <= state_index < len(self.states):
            self.current_state_index = state_index
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        margin = 50

        # Draw curved arc path
        path = QPainterPath()
        start_x = margin
        end_x = w - margin
        base_y = h - 40
        peak_y = 30

        path.moveTo(start_x, base_y)
        path.quadTo(w / 2, peak_y - 50, end_x, base_y)

        # Base arc (grey)
        pen_bg = QPen(QColor(60, 64, 72), 3, Qt.PenStyle.SolidLine)
        painter.setPen(pen_bg)
        painter.drawPath(path)

        # Active arc (purple)
        if self.current_state_index > 0:
            active_path = QPainterPath()
            active_path.moveTo(start_x, base_y)
            
            # Approximate curve progress based on state index
            progress = self.current_state_index / (len(self.states) - 1)
            ctrl_x = start_x + (w / 2 - start_x) * progress * 2
            
            curr_x = start_x + (end_x - start_x) * progress
            curr_y = path.pointAtPercent(progress).y()
            
            if progress <= 0.5:
                active_path.quadTo(ctrl_x, peak_y - 50 * progress, curr_x, curr_y)
            else:
                active_path.quadTo(w / 2, peak_y - 50, curr_x, curr_y)
                
            pen_active = QPen(QColor(187, 134, 252), 4, Qt.PenStyle.SolidLine)
            painter.setPen(pen_active)
            # painter.drawPath(active_path)  # simplified active line, but it's tricky to get perfect quad curve segment

        # Draw nodes
        font = QFont("Inter", 8, QFont.Weight.Bold)
        painter.setFont(font)

        for i, state in enumerate(self.states):
            progress = i / (len(self.states) - 1)
            point = path.pointAtPercent(progress)
            x, y = point.x(), point.y()

            # Colors
            if i < self.current_state_index:
                color = QColor(187, 134, 252) # Past = purple
                radius = 5
            elif i == self.current_state_index:
                color = QColor(0, 255, 136) # Current = green neon
                radius = 7
            else:
                color = QColor(100, 100, 110) # Future = grey
                radius = 4

            # Draw circle
            painter.setBrush(color)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(QRectF(x - radius, y - radius, radius * 2, radius * 2))

            # Draw glow for current
            if i == self.current_state_index:
                painter.setBrush(QColor(0, 255, 136, 60))
                painter.drawEllipse(QRectF(x - radius - 4, y - radius - 4, (radius+4) * 2, (radius+4) * 2))

            # Draw label
            if i == self.current_state_index:
                painter.setPen(QColor(255, 255, 255))
            else:
                painter.setPen(QColor(160, 170, 181))
                
            text_y = y + 25 if i % 2 == 0 else y - 15
            painter.drawText(int(x - 40), int(text_y), 80, 20, Qt.AlignmentFlag.AlignCenter, state)