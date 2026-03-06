from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QColor, QFont
from PyQt6.QtCore import Qt


class TimelineWidget(QWidget):

    def __init__(self):
        super().__init__()

        # Ordered flight states
        self.states = [
            "Idle",
            "Armed",
            "Liftoff",
            "Ascent",
            "Apogee",
            "Parachute",
            "Descent",
            "Landed"
        ]

        self.current_state_index = 0

        self.setMinimumHeight(140)

    def set_state(self, state_index):
        """Update highlighted state"""
        self.current_state_index = state_index
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        margin = 40
        line_y = int(h * 0.45)

        # spacing between states
        spacing = (w - 2 * margin) / (len(self.states) - 1)

        points = []

        for i in range(len(self.states)):
            x = margin + i * spacing
            y = line_y
            points.append((x, y))

        # draw timeline line
        pen = QPen(QColor(120, 120, 120), 2)
        painter.setPen(pen)

        for i in range(len(points) - 1):
            painter.drawLine(
                int(points[i][0]), int(points[i][1]),
                int(points[i + 1][0]), int(points[i + 1][1])
            )

        # draw states
        font = QFont()
        font.setPointSize(9)
        painter.setFont(font)

        for i, (x, y) in enumerate(points):

            if i == self.current_state_index:
                color = QColor(187, 134, 252)  # purple highlight
                radius = 6
            else:
                color = QColor(220, 220, 220)
                radius = 4

            painter.setBrush(color)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(int(x - radius), int(y - radius), radius * 2, radius * 2)

            # draw label below
            painter.setPen(QColor(220, 220, 220))
            painter.drawText(
                int(x - 35),
                int(y + 25),
                70,
                20,
                Qt.AlignmentFlag.AlignCenter,
                self.states[i]
            )