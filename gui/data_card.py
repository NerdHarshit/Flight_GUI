from PyQt6.QtWidgets import QFrame, QVBoxLayout, QLabel, QGridLayout
from PyQt6.QtCore import Qt

class DataCard(QFrame):
    def __init__(self, title: str, fields: list[str], columns=1):
        super().__init__()
        self.setObjectName("Card")
        self.value_labels = {}

        self.main_layout = QVBoxLayout()
        self.main_layout.setContentsMargins(12, 12, 12, 12)
        self.main_layout.setSpacing(8)
        self.setLayout(self.main_layout)

        # Title
        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setObjectName("CardTitle")
        self.main_layout.addWidget(self.title_label)

        # Grid for fields
        self.grid = QGridLayout()
        self.grid.setHorizontalSpacing(15)
        self.grid.setVerticalSpacing(8)
        self.main_layout.addLayout(self.grid)

        # Add fields dynamically
        for idx, field in enumerate(fields):
            row = idx // columns
            col = (idx % columns) * 2

            label = QLabel(field)
            label.setObjectName("FieldLabel")
            
            value = QLabel("0.0")
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            value.setObjectName("ValueLabel")

            self.grid.addWidget(label, row, col)
            self.grid.addWidget(value, row, col + 1)

            self.value_labels[field] = value

    def update_value(self, field: str, value):
        if field in self.value_labels:
            if isinstance(value, float):
                self.value_labels[field].setText(f"{value:.2f}")
            else:
                self.value_labels[field].setText(str(value))