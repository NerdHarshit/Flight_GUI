from PyQt6.QtWidgets import QFrame, QVBoxLayout, QLabel, QGridLayout
from PyQt6.QtCore import Qt


class DataCard(QFrame):
    def __init__(self, title: str, fields: list[str]):
        super().__init__()

        self.setObjectName("DataCard")

        self.value_labels = {}

        # Main vertical layout
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # Title
        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setObjectName("CardTitle")
        self.main_layout.addWidget(self.title_label)

        # Divider line
        self.divider = QFrame()
        self.divider.setFrameShape(QFrame.Shape.HLine)
        self.divider.setFrameShadow(QFrame.Shadow.Sunken)
        self.divider.setStyleSheet("color:#FFFFFF;")
        self.main_layout.addWidget(self.divider)

        # Grid for fields
        self.grid = QGridLayout()
        self.main_layout.addLayout(self.grid)

        # Add fields dynamically
        for row, field in enumerate(fields):
            label = QLabel(field)
            value = QLabel("0")

            value.setAlignment(Qt.AlignmentFlag.AlignRight)

            self.grid.addWidget(label, row, 0)
            self.grid.addWidget(value, row, 1)

            self.value_labels[field] = value

    def update_value(self, field: str, value):
        if field in self.value_labels:
            self.value_labels[field].setText(str(value))